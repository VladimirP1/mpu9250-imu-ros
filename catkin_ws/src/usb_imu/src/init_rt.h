#include <limits.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

class realtime_priority{
public:
    static void init() {
        /* Set affinity */
        /*cpu_set_t cpus;
        CPU_ZERO(&cpus); 
        CPU_SET(3, &cpus);
        if(sched_setaffinity(0, sizeof(cpu_set_t), &cpus)) {
            printf("sched_setaffinity failed: %m\n");
            exit(-2);
        }*/
        /* Lock memory */
        if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
            printf("mlockall failed: %m\n");
            exit(-2);
        }
        /* Set scheduler */
        struct sched_param param;
        param.sched_priority = 99;
        if(sched_setscheduler(0, SCHED_FIFO, &param)){
            printf("sched_setscheduler failed: %m\n");
            exit(-2);
        }
    }
};