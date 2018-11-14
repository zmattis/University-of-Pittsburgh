#include "priorityInversion.h"

/**
 * Use Pthreads to create a working example of priority inversion and inheritance
 */

#define NUM_THREADS 3
#define POLICY SCHED_RR
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER, dispThreadMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutexattr_t mutexattr;

int tail = 0;
int A[2000];

// from http://man7.org/linux/man-pages/man3/pthread_setschedparam.3.html
static void display_sched_attr(int policy, struct sched_param *param) {
    printf("\t    policy=%s, priority=%d\n",
           (policy == SCHED_FIFO) ? "SCHED_FIFO" :
           (policy == SCHED_RR) ? "SCHED_RR" :
           (policy == SCHED_OTHER) ? "SCHED_OTHER" :
           "???",
           param->sched_priority);
}

static void display_thread_sched_attr(const char *msg) {
    pthread_mutex_lock(&dispThreadMutex);

    int policy, s;
    struct sched_param param;

    pthread_t ptid = pthread_self();

    s = pthread_getschedparam(ptid, &policy, &param);
    if (s != 0)
        handle_error_en(s, "pthread_getschedparam");


    printf("\tid: %x %s\n", (unsigned int) ptid, msg);
    display_sched_attr(policy, &param);

    pthread_mutex_unlock(&dispThreadMutex);
}



void *startFun(const char *calledBy) {
    pthread_mutex_lock(&mutex);
    printf("\t\t\tstartFun start: called by %s\n", calledBy);

    int i;
    for (i = 0; i < 2; ++i) {
        int foo = i * i;
        printf("\t\t\tcaller %s to set A[%d] = %d\n", calledBy, tail, foo);
        A[tail] = foo;
        printf("\t\t\tcaller %s has set A[%d] = %d\n", calledBy, tail, foo);
        tail++;
        printf("\t\t\tcaller %s has incremented tail to %d\n", calledBy, tail);
    }

    printf("\t\t\tstartFun end by %s\n", calledBy);
    pthread_mutex_unlock(&mutex);
    return NULL;
}

//function 1 for thread 1.
//do some work here
void *funtion1() {
    display_thread_sched_attr("function 1");

    int i;
    for (i = 0; i < 10; ++i) {
        int x = i * i;
        startFun("function 1");
    }

    printf("function 1 end\n");
    return NULL;
}

//function 2 for thread 2.
//do some work here and call function()
void *function2() {
    display_thread_sched_attr("function 2");

    int i;
    for (i = 0; i < 10; ++i) {
        int x = i * i;
        startFun("function 2");
    }

    printf("function 2 end \n");
    return NULL;
}

//function 3 for thread 3.
//do some work here
void *function3() {
    display_thread_sched_attr("function 3");

    startFun("function 3");
    int i;
    for (i = 0; i < 10000; ++i) {
        int x = i * i;
    }


    printf("function 3 end \n");
    return NULL;
}


//    //create mutex and initialize it.
////create thread 1, thread 2 and thread 3
////set different priority for each thread
////set handler for thread 1, thread 2 and thread 3 respectively. For instance, function 1 for thread 1, function 2 for thread 2 and function 3 for thread 3
//    3. Please think carefully on what priority inheritance is e.g., advent order of each thread. For example, if thread 1 has the lowest priority and thread 3 has the highest priority.
// The definition of priority inheritance is that thread 1 should be earlier than thread 3 and
// thread 2 is later than thread 3.
// Priority is thread 1, 3, 2;

/**
* dfdffd
*/
int runPriorityInversion() {
    pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);

    pthread_mutex_init(&mutex, &mutexattr);

    pthread_mutex_init(&dispThreadMutex, NULL);

    void *status;

    pthread_attr_t attr;

    pthread_attr_init(&attr);

    pthread_attr_setschedpolicy(&attr, POLICY);

    pthread_t threads[NUM_THREADS];

    int threadMax = sched_get_priority_max(POLICY);
    int threadMin = sched_get_priority_min(POLICY);
    int threadMiddle = (threadMax - threadMin) / 2 + threadMin;

    pthread_attr_setschedparam(&attr, &(struct sched_param) {.sched_priority= threadMin});
    if (pthread_create(&threads[0], &attr, (void *(*)(void *)) funtion1, NULL)) {

        fprintf(stderr, "Error creating thread\n");
        return EXIT_FAILURE;
    }

    pthread_attr_setschedparam(&attr, &(struct sched_param) {.sched_priority=threadMax});
    if (pthread_create(&threads[2], &attr, (void *(*)(void *)) function3, NULL)) {

        fprintf(stderr, "Error creating thread\n");
        return EXIT_FAILURE;

    }

    pthread_attr_setschedparam(&attr, &(struct sched_param) {.sched_priority=threadMiddle});
    if (pthread_create(&threads[1], &attr, (void *(*)(void *)) function2, NULL)) {

        fprintf(stderr, "Error creating thread\n");
        return EXIT_FAILURE;

    }


    pthread_attr_destroy(&attr);

//    int which = PRIO_PROCESS; // PRIO_PROCESS for priority of process
//    id_t pid;
//    int priority = -20;
//    int ret;
//    pid_t pidt = getpid(); // get process id
//    ret = getpriority(which, pid); // get process priority
//    ret = setpriority(which, pid, priority); // set process priority

    int i;
    for (i = 0; i < NUM_THREADS; i++) {
        pthread_join(threads[i], &status);
    }

    pthread_mutex_destroy(&mutex);

    pthread_exit(NULL);
}



int main() {

    runPriorityInversion();

    return EXIT_SUCCESS;
}
