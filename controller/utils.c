#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/sysinfo.h>
#include <libubox/ulog.h>
#include <libubox/uloop.h>

#include "utils.h"

typedef unsigned short fix5_t;
const fix5_t fix5_one = 0x0100;

struct cpu_tick *cpustat[2];
static int cpustat_index_curr = 0;

static struct uloop_timeout get_cpustat_timeout;

fix5_t fix5_from_float(float a)
{
    float temp = a * fix5_one;

    temp += (temp >= 0) ? 0.5f : -0.5f;

    return (fix5_t)temp;
}

float fix5_to_float(fix5_t a)
{
    return (float)a / fix5_one;
}

uint64_t idle_ticks(cpu_tick_t *stat)
{
    return stat->ticks[TICK_IDLE] + stat->ticks[TICK_IOWAIT];
}

uint64_t total_ticks(cpu_tick_t *stat)
{
    uint64_t total = 0;
    for (int i = 0; i < NUM_TICK_TYPES; i++)
        total += stat->ticks[i];
    return total;
}

void read_cpustat(cpu_tick_t *cpu_stat)
{
    int i;
    FILE *stat_fp = fopen("/proc/stat", "r");

    int nprocs = get_nprocs();
    for (i = 0; i <= nprocs; i++) {
        fscanf(
            stat_fp,
            "%s %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu\n",
            cpu_stat[i].name,
            &(cpu_stat[i].ticks[TICK_USER]),
            &(cpu_stat[i].ticks[TICK_NICE]),
            &(cpu_stat[i].ticks[TICK_SYSTEM]),
            &(cpu_stat[i].ticks[TICK_IDLE]),
            &(cpu_stat[i].ticks[TICK_IOWAIT]),
            &(cpu_stat[i].ticks[TICK_IRQ]),
            &(cpu_stat[i].ticks[TICK_SOFTIRQ]),
            &(cpu_stat[i].ticks[TICK_STEAL]),
            &(cpu_stat[i].ticks[TICK_GUEST]),
            &(cpu_stat[i].ticks[TICK_GUEST_NICE])
        );

#if 0
        ULOG_INFO("%s %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu",
                cpus_tat[i].name,
                cpus_tat[i].ticks[TICK_USER],
                cpus_tat[i].ticks[TICK_NICE],
                cpus_tat[i].ticks[TICK_SYSTEM],
                cpus_tat[i].ticks[TICK_IDLE],
                cpus_tat[i].ticks[TICK_IOWAIT],
                cpus_tat[i].ticks[TICK_IRQ],
                cpus_tat[i].ticks[TICK_SOFTIRQ],
                cpus_tat[i].ticks[TICK_STEAL],
                cpus_tat[i].ticks[TICK_GUEST],
                cpus_tat[i].ticks[TICK_GUEST_NICE]);
#endif
    }

    fclose(stat_fp);
}

void cpusage(cpu_tick_t *prev, cpu_tick_t *curr)
{
    int nprocs = get_nprocs();

    for (int i = 0; i <= nprocs; i++) {
        uint64_t total = total_ticks(curr + i) - total_ticks(prev + i);
        uint64_t idle = idle_ticks(curr + i) - idle_ticks(prev + i);
        uint64_t active = total - idle;
        ULOG_INFO("%s - load %.1f%%\n", curr[i].name, active * 100.f / total );
    }
}

static void get_cpustat_timeout_handler(struct uloop_timeout *timeout)
{
    read_cpustat(cpustat[cpustat_index_curr ^ 1]);

    /* rotate index */
    cpustat_index_curr ^= 1;

    uloop_timeout_set(&get_cpustat_timeout, 1 * 1000);
}

int utils_cpustat_curr_index(void)
{
    return cpustat_index_curr;
}

int utils_cpustat_timeout_remaining(void)
{
    return uloop_timeout_remaining(&get_cpustat_timeout);
}

int utils_cpustat_timeout_cancel(void)
{
    return uloop_timeout_cancel(&get_cpustat_timeout);
}

int utils_cpustat_timeout_set(void)
{
    return uloop_timeout_set(&get_cpustat_timeout, 1 * 1000);
}

int utils_monitor_cpustat(void)
{
    uint16_t nprocs = 0;

    memset(&cpustat, 0, sizeof(struct cpu_tick *) * 2);

    memset(&get_cpustat_timeout, 0, sizeof(get_cpustat_timeout));
    get_cpustat_timeout.cb = get_cpustat_timeout_handler;
    uloop_timeout_set(&get_cpustat_timeout, 1 * 1000);

    nprocs = get_nprocs_conf();

    cpustat[0] = malloc(sizeof(struct cpu_tick) * (nprocs + 1));
    cpustat[1] = malloc(sizeof(struct cpu_tick) * (nprocs + 1));

    if (cpustat[0] == NULL || cpustat[1] == NULL) {
        ULOG_ERR("out of memory\n");
        goto out_free;
    }

    read_cpustat(cpustat[0]);
    read_cpustat(cpustat[1]);

    ULOG_INFO("monitor cpu stat\n");

    return 0;

out_free:
    if (cpustat[0])
        free(cpustat[0]);
    if (cpustat[1])
        free(cpustat[1]);

    return 0;
}
