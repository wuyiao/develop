#ifndef THREAD_H
#define THREAD_H

//#include <edge.h>
#include <libubox/runqueue.h>

struct thread {
    struct runqueue_task_type type;
    struct runqueue_task task;
    void (*thread)(struct thread *t);
    //void (*complete)(struct runqueue *q, struct runqueue_task *t);
};

struct schedule {
    struct runqueue q;
};

void schedule_init(struct schedule *t);
void thread_init(struct thread *thread);
void thread_schedule(struct schedule *s, struct thread *t);
void thread_complete(struct thread *t);

void thread_type_cancel(struct runqueue *q, struct runqueue_task *t, int type);
void thread_q_done(struct runqueue *q);
void thread_q_complete(struct runqueue *q, struct runqueue_task *t);

#endif
