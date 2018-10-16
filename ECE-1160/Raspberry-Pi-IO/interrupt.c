/**
 * @author Zachary M. Mattis
 * CoE 1160
 * Lab 2
 * October 15, 2018
 * 
 * This program acts as a system-call based interrupt over Linux on a Raspberry Pi.
 * When the timer expires, a signal is sent to the kernel which runs a specific
 * interrupt handling function. This program was successfully compiled using gcc 
 * on a raspberry pi 3 using the following command: gcc -o interrupt interrupt.c
 */ 
 
/* Header Files */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/time.h>

/* Macros */
#define TIMEOUT 3
#define BUFFER_SIZE 80
 
/* Function to handle timer interrupt */
void timer_handler(int signum) {
    time_t epoch;
    struct tm ts;
    char buf[BUFFER_SIZE];
    
    time(&epoch);
    ts = *localtime(&epoch);
    strftime(buf, sizeof buf, "%a %Y-%m-%d %H:%M:%S %Z", &ts);
    
    printf("Alarm expired:\n\t");
    printf("%s\n", buf);
    
    exit(EXIT_SUCCESS);
}
 
 
int main(void) {

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = timer_handler;
    sa.sa_flags   = 0;
    
    struct itimerval timeset;
    memset(&timeset, 0, sizeof timeset);
    timeset.it_interval.tv_sec  = TIMEOUT;
    timeset.it_interval.tv_usec = 0;
    timeset.it_value.tv_sec     = TIMEOUT;
    timeset.it_value.tv_usec    = 0;
   
    signal(SIGALRM, timer_handler);             //register signal handler
    setitimer(ITIMER_REAL, &timeset, NULL);     //set timer
    
    for(;;) {}
    
    return EXIT_SUCCESS;
}
