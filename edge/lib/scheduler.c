//
// Copyright (c) 2018 IOTech
//
// SPDX-License-Identifier: Apache-2.0
//
#include "iot/data.h"
#include "iot/container.h"
#include "iot/scheduler.h"
#include "iot/thread.h"
#include "iot/time.h"

#define IOT_NS_TO_SEC(s) ((s) / IOT_BILLION)
#define IOT_NS_REMAINING(s) ((s) % IOT_BILLION)
#define IOT_SCHEDULER_DEFAULT_WAKE (IOT_HOUR_TO_NS (24))

#ifdef IOT_BUILD_COMPONENTS
#define IOT_SCHEDULER_FACTORY iot_scheduler_factory ()
#else
#define IOT_SCHEDULER_FACTORY NULL
#endif

/* Schedule */
struct iot_schedule_t
{
  iot_schedule_t * next;             /* The next schedule */
  iot_schedule_t * previous;         /* The previous schedule */
  iot_schedule_fn_t function;        /* The function called by the schedule */
  iot_schedule_free_fn_t freefn;     /* The function to clear the arguments when schedule is deleted */
  void * arg;                        /* Function input arg */
  iot_threadpool_t * threadpool;     /* Thread pool used to run scheduled function */
  int priority;                      /* Schedule priority (pool override) */
  uint64_t period;                   /* The period of the schedule, in ns */
  uint64_t start;                    /* The start time of the schedule, in ns, */
  uint64_t repeat;                   /* The number of repetitions, 0 = infinite */
  atomic_uint_fast64_t dropped;      /* Number of events dropped */
  bool scheduled;                    /* A flag to indicate schedule status */
};

/* Schedule Queue */
typedef struct iot_schd_queue_t
{
  struct iot_schedule_t * front;     /* Pointer to the front of queue */
  uint32_t length;                   /* Number of jobs in the queue   */
} iot_schd_queue_t;

/* Scheduler */
struct iot_scheduler_t
{
  iot_component_t component;      /* Component base type */
  iot_schd_queue_t queue;         /* Active schedule queue */
  iot_schd_queue_t idle_queue;    /* Idle schedule queue */
  iot_logger_t * logger;          /* Optional logger */
  struct timespec schd_time;      /* Time for next schedule */
};

/* ========================== PROTOTYPES ============================ */

static void iot_schedule_enqueue (iot_schd_queue_t * queue, iot_schedule_t * schedule);
static void iot_schedule_dequeue (iot_schd_queue_t * queue, iot_schedule_t * schedule);

static inline void iot_schedule_requeue (iot_schd_queue_t * from, iot_schd_queue_t * to, iot_schedule_t * schedule)
{
  iot_schedule_dequeue (from, schedule);
  iot_schedule_enqueue (to, schedule);
}

/* ========================== Scheduler ============================ */

/* Convert time in ns to timespec */
static inline void nsToTimespec (uint64_t ns, struct timespec * ts)
{
  ts->tv_sec = (time_t) IOT_NS_TO_SEC (ns);
  ts->tv_nsec = (long) IOT_NS_REMAINING (ns);
}

/* Get the current time as an unsigned 64bit int */
static uint64_t getTimeAsUInt64 (void)
{
  struct timespec ts;
  clock_gettime (CLOCK_REALTIME, &ts);
  return (uint64_t) ts.tv_sec * IOT_BILLION + ts.tv_nsec;
}

/* Scheduler thread function */
static void * iot_scheduler_thread (void * arg)
{
  iot_component_state_t state;
  uint64_t ns;
  iot_scheduler_t * scheduler = (iot_scheduler_t*) arg;
  iot_schd_queue_t * queue = &scheduler->queue;
  iot_schd_queue_t * idle_queue = &scheduler->idle_queue;

  clock_gettime (CLOCK_REALTIME, &scheduler->schd_time);

  while (true)
  {
    state = iot_component_wait_and_lock (&scheduler->component, (uint32_t) IOT_COMPONENT_DELETED | (uint32_t) IOT_COMPONENT_RUNNING);

    if (state == IOT_COMPONENT_DELETED)
    {
      iot_ULOG_DEBUG (scheduler->logger, "Scheduler thread terminating");
      break; // Exit thread on deletion
    }
    int ret = pthread_cond_timedwait (&scheduler->component.cond, &scheduler->component.mutex, &scheduler->schd_time);
    state = scheduler->component.state;
    if (state == IOT_COMPONENT_DELETED)
    {
      iot_ULOG_DEBUG (scheduler->logger, "Scheduler thread terminating");
      break; // Exit thread on deletion
    }
    if (state == IOT_COMPONENT_STOPPED)
    {
      iot_ULOG_DEBUG (scheduler->logger, "Scheduler thread stopping");
      iot_component_unlock (&scheduler->component);
      continue; // Wait for thread to be restarted or deleted
    }
    if (ret == 0)
    {
      ns = (queue->length > 0) ? queue->front->start : (getTimeAsUInt64 () + IOT_SCHEDULER_DEFAULT_WAKE);
    }
    else
    {
      /* Check if the queue is populated */
      if (queue->length > 0)
      {
        /* Get the schedule at the front of the queue */
        iot_schedule_t *current = queue->front;

        /* Post the work to the thread pool or run as thread */
        if (current->threadpool)
        {
          iot_log_trace (scheduler->logger, "Running schedule from threadpool");
          if (! iot_threadpool_try_work (current->threadpool, current->function, current->arg, current->priority))
          {
            if (atomic_fetch_add (&current->dropped, 1u) == 0)
            {
              iot_log_warn (scheduler->logger, "Scheduled event dropped for schedule @ %p", current);
            }
          }
        }
        else
        {
          iot_log_trace (scheduler->logger, "Running schedule as thread");
          iot_thread_create (NULL, current->function, current->arg, current->priority, IOT_THREAD_NO_AFFINITY, scheduler->logger);
        }

        /* Recalculate the next start time for the schedule */
        current->start = getTimeAsUInt64 () + current->period;

        if (current->repeat != 0)
        {
          current->repeat -= 1;
          /* If the number of repetitions has just become 0 */
          if (current->repeat == 0)
          {
            iot_log_trace (scheduler->logger, "Move Schedule to idle queue");
            iot_schedule_requeue (queue, idle_queue, current);
            current->scheduled = false;
          }
          else
          {
            iot_log_trace (scheduler->logger, "Re-queue schedule");
            iot_schedule_requeue (queue, queue, current);
          }
        }
        else
        {
          iot_log_trace (scheduler->logger, "Re-schedule schedule");
          iot_schedule_requeue (queue, queue, current);
        }
        ns = (queue->length > 0) ? queue->front->start : (getTimeAsUInt64 () + IOT_SCHEDULER_DEFAULT_WAKE);
      }
      else
      {
        /* Set the wait time to 1 second if the queue is not populated */
        ns = getTimeAsUInt64 () + IOT_SCHEDULER_DEFAULT_WAKE;
      }
    }
    nsToTimespec (ns, &scheduler->schd_time); /* Calculate next execution time */
    iot_component_unlock (&scheduler->component);
  }
  iot_component_unlock (&scheduler->component);
  return NULL;
}

/* Initialise the schedule queue and processing thread */
iot_scheduler_t * iot_scheduler_alloc (int priority, int affinity, iot_logger_t * logger)
{
  iot_scheduler_t * scheduler = (iot_scheduler_t*) calloc (1, sizeof (*scheduler));
  iot_component_init (&scheduler->component, IOT_SCHEDULER_FACTORY, (iot_component_start_fn_t) iot_scheduler_start, (iot_component_stop_fn_t) iot_scheduler_stop);
  scheduler->logger = logger;
  iot_logger_add_ref (logger);
  iot_ULOG_INFO (logger, "iot_scheduler_alloc (priority: %d affinity: %d)", priority, affinity);
  iot_thread_create (NULL, iot_scheduler_thread, scheduler, priority, affinity, logger);
  return scheduler;
}

void iot_scheduler_add_ref (iot_scheduler_t * scheduler)
{
  if (scheduler) iot_component_add_ref (&scheduler->component);
}

/* Start the scheduler thread */
void iot_scheduler_start (iot_scheduler_t * scheduler)
{
  assert (scheduler);
  iot_log_trace (scheduler->logger, "iot_scheduler_start()");
  iot_component_set_running (&scheduler->component);
}

/* Stop the scheduler thread */
void iot_scheduler_stop (iot_scheduler_t * scheduler)
{
  assert (scheduler);
  iot_log_trace (scheduler->logger, "iot_scheduler_stop()");
  iot_component_set_stopped (&scheduler->component);
}

/* Create a schedule and insert it into the queue */
iot_schedule_t * iot_schedule_create (iot_scheduler_t * scheduler, iot_schedule_fn_t func, iot_schedule_free_fn_t free_func, void * arg, uint64_t period, uint64_t start, uint64_t repeat, iot_threadpool_t * pool, int priority)
{
  assert (scheduler && func);
  iot_log_trace (scheduler->logger, "iot_schedule_create (period: %" PRId64 " repeat: %" PRId64 ")", period, repeat);
  iot_schedule_t * schedule = (iot_schedule_t*) calloc (1, sizeof (*schedule));
  schedule->function = func;
  schedule->freefn = free_func;
  schedule->arg = arg;
  schedule->period = period;
  schedule->start = getTimeAsUInt64 () + start;
  schedule->repeat = repeat;
  schedule->threadpool = pool;
  schedule->priority = priority;
  iot_threadpool_add_ref (pool);
  iot_component_lock (&scheduler->component);
  iot_schedule_enqueue (&scheduler->idle_queue, schedule);
  iot_component_unlock (&scheduler->component);

  return schedule;
}

/* Add a schedule to the queue */
bool iot_schedule_add (iot_scheduler_t * scheduler, iot_schedule_t * schedule)
{
  assert (scheduler && schedule);
  bool ret;
  iot_log_trace (scheduler->logger, "iot_schedule_add()");
  iot_component_lock (&scheduler->component);
  if ((ret = !schedule->scheduled))
  {
    /* Remove from idle queue, add to scheduled queue */
    iot_schedule_requeue (&scheduler->idle_queue, &scheduler->queue, schedule);
    /* If the schedule was placed and the front of the queue & the scheduler is running */
    if (scheduler->queue.front == schedule && (scheduler->component.state == IOT_COMPONENT_RUNNING))
    {
      pthread_cond_signal (&scheduler->component.cond);
    }
    schedule->scheduled = true;
  }
  iot_component_unlock (&scheduler->component);
  return ret;
}

/* Remove a schedule from the queue */
bool iot_schedule_remove (iot_scheduler_t * scheduler, iot_schedule_t * schedule)
{
  assert (scheduler && schedule);
  bool ret;
  iot_log_trace (scheduler->logger, "iot_schedule_remove()");
  iot_component_lock (&scheduler->component);
  if ((ret = schedule->scheduled))
  {
    iot_schedule_requeue (&scheduler->queue, &scheduler->idle_queue, schedule);
    schedule->scheduled = false;
  }
  iot_component_unlock (&scheduler->component);
  return ret;
}

/* Reset schedule timeout */
void iot_schedule_reset (iot_scheduler_t * scheduler, iot_schedule_t * schedule)
{
  assert (scheduler && schedule);
  iot_log_trace (scheduler->logger, "iot_schedule_reset()");
  iot_component_lock (&scheduler->component);

  /* Recalculate the next start time for the schedule */
  schedule->start = getTimeAsUInt64 () + schedule->period;

  if (schedule->scheduled)
  {
    iot_schedule_requeue (&scheduler->queue, &scheduler->queue, schedule);
    if (scheduler->queue.front == schedule && (scheduler->component.state == IOT_COMPONENT_RUNNING))
    {
      pthread_cond_signal (&scheduler->component.cond);
    }
  }
  iot_component_unlock (&scheduler->component);
}

/* Delete a schedule */
void iot_schedule_delete (iot_scheduler_t * scheduler, iot_schedule_t * schedule)
{
  assert (scheduler && schedule);
  iot_log_trace (scheduler->logger, "iot_schedule_delete()");
  iot_component_lock (&scheduler->component);
  iot_schedule_dequeue (schedule->scheduled ? &scheduler->queue : &scheduler->idle_queue, schedule);
  iot_component_unlock (&scheduler->component);
  iot_threadpool_free (schedule->threadpool);
  (schedule->freefn) ? schedule->freefn (schedule->arg) : 0;
  free (schedule);
}

extern uint64_t iot_schedule_dropped (const iot_schedule_t * schedule)
{
  assert (schedule);
  return atomic_load (&schedule->dropped);
}

/* Delete all remaining scheduler resources */
void iot_scheduler_free (iot_scheduler_t * scheduler)
{
  if (scheduler && iot_component_dec_ref (&scheduler->component))
  {
    iot_log_trace (scheduler->logger, "iot_scheduler_free()");
    iot_component_set_stopped (&scheduler->component);
    iot_component_lock (&scheduler->component);
    clock_gettime (CLOCK_REALTIME, &scheduler->schd_time);
    iot_component_unlock (&scheduler->component);
    iot_component_set_deleted (&scheduler->component);
    iot_wait_secs (1u);
    while (scheduler->queue.length > 0)
    {
      iot_schedule_delete (scheduler, scheduler->queue.front);
    }
    while (scheduler->idle_queue.length > 0)
    {
      iot_schedule_delete (scheduler, scheduler->idle_queue.front);
    }
    iot_logger_free (scheduler->logger);
    iot_component_fini (&scheduler->component);
    free (scheduler);
  }
}

/* Add a schedule to the queue */
static void iot_schedule_enqueue (iot_schd_queue_t * queue, iot_schedule_t * schedule)
{
  /* Search for the correct schedule location */
  iot_schedule_t * next_schedule = NULL;
  iot_schedule_t * previous_schedule = NULL;
  iot_schedule_t * current_sched = queue->front;
  
  while (current_sched)
  {
    if (schedule->start < current_sched->start)
    {
      next_schedule = current_sched;
      previous_schedule = current_sched->previous;
      break;
    }
    else
    {
      previous_schedule = current_sched;
    }
    current_sched = current_sched->next;
  }

  /* Insert new schedule in correct location */
  if (queue->length == 0)
  {
    schedule->next = NULL;
    schedule->previous = NULL;
  }
  else
  {
    /* Set references in new entry */
    schedule->next = next_schedule;
    schedule->previous = previous_schedule;

    /* Update existing references, check if either is at the front or back */
    if (previous_schedule != NULL)
    {
      previous_schedule->next = schedule;
    }
    if (next_schedule != NULL)
    {
      next_schedule->previous = schedule;
    }
  }
  queue->length += 1;

  /* If no previous schedule, set as front */
  if (previous_schedule == NULL)
  {
    queue->front = schedule;
  }
}

/* Remove a schedule from the queue */
static void iot_schedule_dequeue (iot_schd_queue_t * queue, iot_schedule_t * schedule)
{
  if (schedule->next == NULL)
  {
    if (schedule->previous == NULL) /* If only one schedule in the queue */
    {
      queue->front = NULL;
    }
    else /* If the schedule is at the back of the queue */
    {
      schedule->previous->next = NULL;
    }
  }
  else if (schedule->previous == NULL) /* If the schedule is at the front of the queue */
  {
    schedule->next->previous = NULL;
    queue->front = schedule->next;
  }
  else /* If the schedule is in the middle of the queue */
  {
    schedule->next->previous = schedule->previous;
    schedule->previous->next = schedule->next;
  }
  schedule->next = NULL;
  schedule->previous = NULL;
  queue->length -= 1;
}

#ifdef IOT_BUILD_COMPONENTS

static iot_component_t * iot_scheduler_config (iot_container_t * cont, const iot_data_t * map)
{
  iot_logger_t * logger = (iot_logger_t*) iot_container_find_component (cont, iot_data_string_map_get_string (map, "Logger"));
  int affinity = (int) iot_data_string_map_get_i64 (map, "Affinity", IOT_THREAD_NO_AFFINITY);
  int prio = (int) iot_data_string_map_get_i64 (map, "Priority", IOT_THREAD_NO_PRIORITY);
  return (iot_component_t*) iot_scheduler_alloc (prio, affinity, logger);
}

const iot_component_factory_t * iot_scheduler_factory (void)
{
  static iot_component_factory_t factory = { IOT_SCHEDULER_TYPE, iot_scheduler_config, (iot_component_free_fn_t) iot_scheduler_free, NULL };
  return &factory;
}

#endif
