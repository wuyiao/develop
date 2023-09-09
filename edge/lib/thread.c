/* Thread management routine
 * writen by xuzhou
 */
#include <edge.h>
#include <libubox/runqueue.h>
#include <thread.h>


void thread_q_done(struct runqueue *q)
{
}

void thread_type_cancel(struct runqueue *q, struct runqueue_task *t, int type)
{
    runqueue_task_complete(t);
}

void thread_q_complete(struct runqueue *q, struct runqueue_task *t)
{
}

static void thread_kill_instance(struct runqueue *q, struct runqueue_task *p)
{
    runqueue_process_kill_cb(q, p);
}

void thread_schedule(struct schedule *s, struct thread *t)
{
    runqueue_task_add(&s->q, &t->task, false);
}

void thread_complete(struct thread *t)
{
    runqueue_task_complete(&t->task);
}

void thread_instance(struct runqueue *q, struct runqueue_task *t)
{
    struct thread *th = container_of(t, struct thread, task);
    if (th->thread)
        th->thread(th);
}

void thread_init(struct thread *thread)
{
    /* default should enough for most cases */
    thread->type.run = thread_instance;
    thread->type.cancel = thread_type_cancel;
    thread->type.kill = thread_kill_instance;

    thread->task.type = &thread->type;
    if (!thread->task.run_timeout)
        thread->task.run_timeout = 1000;

    if (!thread->task.complete)
        thread->task.complete = thread_q_complete;
}

void schedule_init(struct schedule *s)
{
    runqueue_init(&s->q);
    if (!s->q.empty_cb)
        s->q.empty_cb = thread_q_done;
    if (!s->q.max_running_tasks)
        s->q.max_running_tasks = 10;
}
