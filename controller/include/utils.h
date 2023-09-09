#ifndef UTILS_H
#define UTILS_H
#include <stdint.h>

typedef enum {
    TICK_USER = 0,
    TICK_NICE,
    TICK_SYSTEM,
    TICK_IDLE,
    TICK_IOWAIT,
    TICK_IRQ,
    TICK_SOFTIRQ,
    TICK_STEAL,
    TICK_GUEST,
    TICK_GUEST_NICE,
    NUM_TICK_TYPES
} cpu_tick_type;

typedef struct cpu_tick {
    char name[16];
    long unsigned int ticks[NUM_TICK_TYPES];
} cpu_tick_t;

typedef unsigned short fix5_t;

fix5_t fix5_from_float(float a);
float fix5_to_float(fix5_t a);
uint64_t idle_ticks(cpu_tick_t *stat);
uint64_t total_ticks(cpu_tick_t *stat);
void read_cpustat(cpu_tick_t *cpu_stat);
int utils_monitor_cpustat(void);
void cpusage(cpu_tick_t *prev, cpu_tick_t *curr);
int utils_cpustat_timeout_remaining(void);
int utils_cpustat_timeout_set(void);
int utils_cpustat_timeout_cancel(void);
int utils_cpustat_curr_index(void);
#endif
