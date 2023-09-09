#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <threadpool.h>


static void listdata_delete(void *val)
{
    if (val)
        free(val);
}

static int threadpool_tune_num(int target)
{
    if (target > MAX_THREAD_NUM)
        target = MAX_THREAD_NUM;
    if (target < MIN_THREAD_NUM)
        target = MIN_THREAD_NUM;
    return target;
}

void threadpool_do_job(void * threadpool)
{
    int best_workernum;
    worker_t *curr, *prev;

    pthread_t thr = pthread_self();
    threadpool_t *tp = (threadpool_t *)threadpool;

    for (;;) {
        job_t *job;

        if (pthread_mutex_lock(&(tp->job_lock)))
            ULOG_ERR("lock: %s", strerror(errno));

        while ((listcount(tp->jobs) == 0) && (!tp->shutdown))
            pthread_cond_wait(&(tp->notify), &(tp->job_lock));

        pthread_mutex_lock(&(tp->worker_lock));
        if (tp->shutdown == shutdown_immediate || 
            (tp->shutdown == shutdown_waitall && tp->jobsnum == 0)) {
            break;
        }
        if (tp->dynamic == fix_num) {
            if (tp->target_workernum < listcount(tp->workers)) {
                break;
            }
        }
        else {
            if (tp->last_workerchange + TIME_INTERVAL < time(NULL)) {
                best_workernum = (int)(listcount(tp->jobs) / JOB_WORKER_RATIO);
                if (best_workernum < listcount(tp->workers) &&
                        MIN_THREAD_NUM < listcount(tp->workers)) {
                    break;
                }
            }
        }

        ULOG_INFO("jobnum is %d, thread_num is %d", tp->jobsnum, tp->workernum);

        pthread_mutex_unlock(&(tp->worker_lock));
        if (listcount(tp->jobs) == 0)
            continue;

        listnode_head(tp->jobs);
        /* delete head job */
        listdata_delete(job);

        pthread_mutex_unlock(&(tp->job_lock));

        (*(job->jobfun))(job->args);

        // fprintf(stderr, "free %d\n", *((int*)(job->args)));
        /* free memory */
        /* this is shoud be malloc variable */
        if (job->args)
            free(job->args);
        listnode_delete(job);
    }

    pthread_mutex_unlock(&(tp->worker_lock));
    pthread_mutex_unlock(&(tp->job_lock));
}

static void threadpool_add_worker_withoutlock(threadpool_t *tp)
{
    worker_t *worker;

    worker = (worker_t *)malloc(sizeof(worker_t));
    if (!worker) {
        ULOG_ERR("create worker");
        goto err;
    }

    if (pthread_create(&(worker->thread), NULL, &threadpool_do_job, (void *)tp) != 0) {
        ULOG_ERR("pthread create");
        goto err;
    }
    
    listnode_add(tp->workers, worker);

    tp->last_workerchange = time(NULL);
    ULOG_INFO("woker num %d, time at %ld", tp->workernum, (long)tp->last_workerchange);
}

static void threadpool_add_worker(threadpool_t *tp)
{
    pthread_mutex_lock(&(tp->worker_lock));
    threadpool_add_worker_withoutlock(tp);
    pthread_mutex_unlock(&(tp->worker_lock));
}

threadpool_t *threadpool_init(int workernum, threadpool_dynamic_t dynamic)
{
    int i;
    threadpool_t *tp;

    tp = (threadpool_t*)malloc(sizeof(threadpool_t));
    if (!tp) {
        ULOG_ERR("malloc threadpool"); 
        goto out;
    }

    memset(tp, 0, sizeof(threadpool_t));

    tp->target_workernum = threadpool_tune_num(workernum);
    tp->dynamic = dynamic;

    if (pthread_mutex_init(&(tp->worker_lock), NULL) ||
            pthread_mutex_init(&(tp->job_lock), NULL) ||
            pthread_cond_init(&(tp->notify), NULL)) {
        goto free_threadpool;
    }

    tp->workers = list_new();
    if (!tp->workers)
        goto free_threadpool;

    tp->workers->del = listdata_delete;

    tp->jobs = list_new();
    if (!tp->jobs)
        goto free_list;

    tp->jobs->del = listdata_delete;

    if (dynamic == fix_num) {
        for (i = 0; i < tp->target_workernum; i++) {
            threadpool_add_worker(tp);
        }
        return tp;
    }
    return tp;

free_list:
    if (tp->workers)
        free(tp->workers);
free_threadpool:
    /* if tp is null, it will reach this step */
    if (tp)
        free(tp);
out:
    return NULL;
}


int threadpool_add_job(threadpool_t *tp, job_t *job)
{
    int i;
    int err = 0;
    int best_workernum;

    if (tp == NULL || job->jobfun == NULL)
        return threadpool_invalid;

    if (pthread_mutex_lock(&(tp->job_lock)) != 0) {
        ULOG_ERR("lock error");
        return threadpool_lock_failure;
    }

    listnode_add(tp->jobs, job);

    /* whether need to add worker */
    if (tp->dynamic && tp->last_workerchange + TIME_INTERVAL < time(NULL)) {
        pthread_mutex_lock(&(tp->worker_lock));
        best_workernum = threadpool_tune_num((int)(tp->jobsnum / JOB_WORKER_RATIO));

        ULOG_INFO("best worker num is %d\n", best_workernum);
        for (i = listcount(tp->workers); i < best_workernum; ++i) {
            threadpool_add_worker_withoutlock(tp);
        }
        pthread_mutex_unlock(&(tp->worker_lock));
    }

    if (pthread_cond_signal(&(tp->notify)) != 0) {
        err = threadpool_lock_failure;
    }

    if (pthread_mutex_unlock(&(tp->job_lock)) != 0) {
        err = threadpool_lock_failure;
    }

    return err;
}

void threadpool_destory(threadpool_t *tp, threadpool_shutdown_t shutdown_type)
{
    worker_t *worker;
    struct listnode *node, *nextnode;

    if (tp == NULL || tp->shutdown)
        return;

    /* lock job_lock */
    if (pthread_mutex_lock(&(tp->job_lock)) != 0) {
        ULOG_INFO("thread %ld function %s error %s", (long)pthread_self() ,__FUNCTION__, "lock job_lock" );
        ULOG_ERR("lock job_lock");
    }

    tp->shutdown = shutdown_type;

    /* wake up all wokers */
    if (pthread_cond_broadcast(&(tp->notify)) != 0)
        ULOG_ERR("broadcast error");

    if (pthread_mutex_unlock(&(tp->job_lock)) != 0)
        ULOG_ERR("unlcok job_lock");

    while (listcount(tp->workers) > 0) {
        /* wait */
        ;
    }

    pthread_mutex_destroy(&(tp->worker_lock));
    pthread_mutex_destroy(&(tp->job_lock));
    pthread_cond_destroy(&(tp->notify));
    list_delete(tp->workers);
    list_free(tp->jobs);

    free(tp);
    return;
}

void threadpool_change_target_workernum(threadpool_t *tp, int target)
{
    int i;

    if (tp == NULL || tp->shutdown)
        return;

    target = threadpool_tune_num(target);

    if (pthread_mutex_lock(&(tp->job_lock)) != 0) {
        ULOG_INFO("thread %ld function %s error %s", (long)pthread_self() ,__FUNCTION__, "lock job_lock" );
        ULOG_ERR("lock job_lock");
    }
    if (pthread_mutex_lock(&(tp->worker_lock)) != 0) {
        pthread_mutex_unlock(&(tp->job_lock));
        ULOG_ERR("lock woker_lock");
    }

    tp->target_workernum = target;
    if (tp->target_workernum >= listcount(tp->workers)) {
        while (tp->target_workernum > listcount(tp->workers)) {
            threadpool_add_worker_withoutlock(tp);
        }
    }
    else {
        /* wakeup */
        if (pthread_cond_broadcast(&(tp->notify)) != 0)
            ULOG_ERR("broadcast error");
    }

    if (pthread_mutex_unlock(&(tp->worker_lock)) != 0)
        ULOG_ERR("unlock worker_lock");
    if (pthread_mutex_unlock(&(tp->job_lock)) != 0)
        ULOG_ERR("unlock job_lock");
}
