/**
 * @author Zachary M. Mattis
 * COE 1550
 * Traffic Simulator
 * June 28, 2018
 *
 * This C file is a traffic simulation program that utilizes
 * Linux system calls to maintain syncronization between processes
 * via Semaphores.
 */

/* Header Files */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>

/* Macros */
#define TRAFFIC_BUFFER_SIZE 10      //10 cars = full buffer
#define TRAFFIC_SLEEP 20            //20 seconds before new car arrives
#define CAR_SLEEP 2                 //2 seconds for car to travel through construction zone

#define __NR_cs1550_down    325     //CoE 1550 Project 2
#define __NR_cs1550_up      326     //CoE 1550 Project 2


//Node struct to use as linked list process queue implementation
struct cs1550_node {
  struct cs1550_node *next; //Reference to the next node in the doubly linked list
  struct task_struct *process_info;
};

//Semaphore struct - Counting
struct cs1550_sem {
  int value; 				        	//Value contained within the counting semaphore
  struct cs1550_node *head; 	//Process queue -- head of the LL, 0(1) remove
  struct cs1550_node *tail; 	//Process queue -- tail of the LL, 0(1) add
};

/* Function Prototypes */
void down(struct cs1550_sem*);
void up(struct cs1550_sem*);
int is_new_car(void);


int main(void) {

    //variable declaration
    int *car_num;
    struct cs1550_sem *n_mutex, *n_full, *n_empty;      //north semaphores
    struct cs1550_sem *s_mutex, *s_full, *s_empty;      //south semaphores
    int *n_buff, *s_buff;                               //north & south buffers
    int *n_curr_prod, *n_curr_cons;
    int *s_curr_prod, *s_curr_cons;
    struct cs1550_sem* semaphore_memory;
    int *variable_memory;

    //Allocate 6 semaphores in shared memory
    semaphore_memory = (struct cs1550_sem*) mmap(NULL, sizeof(struct cs1550_sem)*6, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, 0, 0);
    //Allocate buffer x2 (N & S)
    variable_memory = (int*) mmap(NULL, ((TRAFFIC_BUFFER_SIZE*2)+4)*sizeof(int), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, 0, 0);


    //1st semaphore = mutex
    n_mutex = semaphore_memory;
    n_mutex->value = 1;
    n_mutex->head = NULL;
    n_mutex->tail = NULL;

    //2nd semaphore = mutex
    s_mutex = semaphore_memory+1;
    s_mutex->value = 1;
    s_mutex->head = NULL;
    s_mutex->tail = NULL;

    //3rd semaphore = number of resources used
    n_full = semaphore_memory+2;
    n_full->value = 0;
    n_full->head = NULL;
    n_full->tail = NULL;

    //4th semaphore = number of resources used
    s_full = semaphore_memory+3;
    s_full->value = 0;
    s_full->head = NULL;
    s_full->tail = NULL;

    //5th semaphore = number of resources available
    n_empty = semaphore_memory+4;
    n_empty->value = TRAFFIC_BUFFER_SIZE;
    n_empty->head = NULL;
    n_empty->tail = NULL;

    //6th semaphore = number of resources available
    s_empty = semaphore_memory+5;
    s_empty->value = TRAFFIC_BUFFER_SIZE;
    s_empty->head = NULL;
    s_empty->tail = NULL;

    // N.S. Buffers and variables
    n_buff = variable_memory;
    s_buff = variable_memory + TRAFFIC_BUFFER_SIZE;
    n_curr_prod = variable_memory + (TRAFFIC_BUFFER_SIZE*2);
    s_curr_prod = variable_memory + (TRAFFIC_BUFFER_SIZE*2) + 1;
    n_curr_cons = variable_memory + (TRAFFIC_BUFFER_SIZE*2) + 2;
    s_curr_cons = variable_memory + (TRAFFIC_BUFFER_SIZE*2) + 3;

    //North producer
    if (fork() == 0) {
        for(;;) {
            down(n_empty);
            down(n_mutex);
            if (is_new_car() == 0) {
                sleep(TRAFFIC_SLEEP);
            }
            n_buff[*n_curr_prod % TRAFFIC_BUFFER_SIZE] = *n_curr_prod;
            printf("Car %d coming from North arrived in the queue\n", *n_curr_prod);
            *n_curr_prod += 1;
            up(n_mutex);
            up(n_full);
        }
    }

    //South producer
    if (fork() == 0) {
        for(;;) {
            down(s_empty);
            down(s_mutex);
            if (is_new_car() == 0) {
                sleep(TRAFFIC_SLEEP);
            }
            s_buff[*s_curr_prod % TRAFFIC_BUFFER_SIZE] = *s_curr_prod;
            printf("Car %d coming from South arrived in the queue\n", *s_curr_prod);
            *s_curr_prod += 1;
            up(s_mutex);
            up(s_full);
        }
    }


    //Flag person
    if (fork() == 0) {
        char flag = '\0';
        int asleep = 't';
        for(;;) {
            down(n_mutex);
            down(s_mutex);

            if (asleep = 't' && (n_full->value || s_full->value)) {
                printf("The flagperson is now awake.");
                asleep='f';
            }

            if (flag != 'N' && flag != 'S') {       //No cars previously from either direction
                if (n_full->value) {                //Consume from N
                    down(n_full);
                    printf("Flagperson allowed car %d to enter zone from North\n", n_buff[*n_curr_cons % TRAFFIC_BUFFER_SIZE]);
                    sleep(CAR_SLEEP);
                    printf("Car %d exited construction zone\n", n_buff[*n_curr_cons % TRAFFIC_BUFFER_SIZE]);
                    *n_curr_cons += 1;
                    n_full->value ? (flag = 'N') : (flag = '\0');      //Continue to consume from North if len>0
                    if (!n_full->value && !s_full->value) {
                        printf("The flagperson is now asleep.");
                        asleep='t';
                    }
                    up(n_empty);
                } else if (s_full->value) {         //Consume from S
                    down(s_full);
                    printf("Flagperson allowed car %d to enter zone from South\n", s_buff[*s_curr_cons % TRAFFIC_BUFFER_SIZE]);
                    sleep(CAR_SLEEP);
                    printf("Car %d exited construction zone\n", s_buff[*s_curr_cons % TRAFFIC_BUFFER_SIZE]);
                    *s_curr_cons += 1;
                    s_full->value ? (flag = 'S') : (flag = '\0');      //Continue to consue from South if len>0
                    if (!n_full->value && !s_full->value) {
                        printf("The flagperson is now asleep.");
                        asleep='t';
                    }
                    up(s_empty);
                }
            } else if (flag == 'N' && n_full->value) {
                down(n_full);
                printf("Flagperson allowed car %d to enter zone from North\n", n_buff[*n_curr_cons % TRAFFIC_BUFFER_SIZE]);
                sleep(CAR_SLEEP);
                printf("Car %d exited construction zone\n", n_buff[*n_curr_cons % TRAFFIC_BUFFER_SIZE]);
                *n_curr_cons += 1;
                n_full->value ? (flag = 'N') : (flag = '\0');      //Continue to consume from North if len>0
                if (!n_full->value && !s_full->value) {
                    printf("The flagperson is now asleep.");
                    asleep='t';
                }
                up(n_empty);
            } else if (flag == 'S' && s_full->value) {
                down(s_full);
                printf("Flagperson allowed car %d to enter zone from South\n", s_buff[*s_curr_cons % TRAFFIC_BUFFER_SIZE]);
                sleep(CAR_SLEEP);
                printf("Car %d exited construction zone\n", s_buff[*s_curr_cons % TRAFFIC_BUFFER_SIZE]);
                *s_curr_cons += 1;
                s_full->value ? (flag = 'N') : (flag = '\0');      //Continue to consume from North if len>0
                if (!n_full->value && !s_full->value) {
                    printf("The flagperson is now asleep.");
                    asleep='t';
                }
                up(s_empty);
            }

            up(n_mutex);
            up(s_mutex);
        }
    }

    int status;
    wait(&status);

    return 0;
}


//Semaphore.down() wrapper function
void down(struct cs1550_sem *sem){
    syscall(__NR_cs1550_up, sem);
}

//Semaphore.up() wrapper function
void up(struct cs1550_sem *sem){
    syscall(__NR_cs1550_up, sem);
}

//Generate random next car w/ 80% chance
int is_new_car (void) {
    time_t t;
    int n;
    srand((unsigned) time(&t));
    n = rand() % 10;

    if (n<=7) {         //80% chance
        return 1;
    }
    else {              //20% chance
        return 0;
    }
}