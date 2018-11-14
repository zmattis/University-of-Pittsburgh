#include <stdio.h>
#include <stdlib.h>

void runRMS(int serveA, int cycA, int serveB, int cycB);

int main() {

    runRMS(10, 25, 15, 60);
    runRMS(20, 50, 35, 100);

    return EXIT_SUCCESS;
}

void runRMS(int serveA, int cycA, int serveB, int cycB) {

    int A, B;                           //arrival time of process A and B
    //    int cycA, cycB, serveA, serveB;         //period and execution for A and B processes
    float m;
    int i, j, a = 0, b = 0, ka = 0, kb = 0;                 /*ka,kb is a switch and
 											i, j, a, b, to record status of each process*/
    int numa = 0, numb = 0;                         //accumulated execution time
    int T;
    printf("\t\t\t------------------------------------------------\n");
    printf("\t\t\t\trate monotonic schedule algorithm\n");
    printf("\t\t\t------------------------------------------------\n");
    printf("Process A runs every %d seconds and needs %d seconds to finish\n", cycA, serveA);
    printf("Process B runs every %d seconds and needs %d seconds to finish\n", cycB, serveB);

    //    printf("please input period and execution for A process\n");
    //    scanf("%d%d", &cycA, &serveA);
    //    printf("please input period and execution for B process\n");
    //    scanf("%d%d", &cycB, &serveB);

    //    cycA = INPUT_PERIOD_P1;
    //    serveA = EXECUTION_P1;
    //
    //    cycB = INPUT_PERIOD_P2;
    //    serveB = EXECUTION_P2;

    m = (float) serveA / cycA + (float) serveB / cycB;


    for (T = 0; T <= 100; T++) {

        /* this block is to check if CPU can schedule*/
        // if the period of A is less than B then it should have priority.
        if (cycA < cycB) {
            kb = 0;
            ka = 1;
        } else {
            kb = 1;
            ka = 0;
        }

        ///////// DON'T MODIFY THE CODE BELOW //////////

        /* this block is to say that process A has been done*/
        /* but process B has not been done yet*/
        if (numa == serveA)                                        //process A is done
        {
            numa = serveA + 1;
            printf("when T=%d, ", T);
            printf("process A%d is done\n", a);
            if (numb < serveB) {
                printf("run process B%d\n", b);
                kb = 1;
            }
            ka = 0;
        }

        /* this block is to say that process B has been done*/
        /* but process A has not been done yet*/
        if (numb == serveB) {
            numb = serveB + 1;
            printf("when T=%d, ", T);
            printf("process B%d is done\n", b);
            if (numa < serveA) {
                printf("run process A%d\n", a);
                ka = 1;
            }
            kb = 0;
        }

        /* this block start running the process*/
        if (T % cycA == 0 && T % cycB == 0) {
            A = B = T;
            j = ++a;
            i = ++b;
            printf("when T=%d, process A%d and process B%d are generated together\n", T, j, i);
            if (cycA <= cycB) // deadline is cycle
            {
                printf("run process A%d and suspend process B%d\n", j, i);
                printf("---------------------------------------\n");
                ka = 1;
                kb = 0;
            } else {
                printf("run process B%d and suspend process A%d\n", i, j);
                ka = 0;
                kb = 1;
            }
            numa = numb = 0;
        }


        /* this block initializes A another new period*/
        if (T % cycA == 0 && T % cycB != 0) {
            A = T;
            printf("when T=%d, ", T);
            printf("process A%d is generated\n", ++a);   //impossible to compete with A
            numa = 0;
            if (numb < serveB) {                //process B is unfinished yet
                if (cycB > cycA) {
                    printf("run process A%d\n", a);
                    ka = 1;
                    kb = 0;
                } else {                     //period of B is earlier than period of A
                    printf("process B%d is moving forward\n", b);
                }
            } else                                   //process B is done, just run A
            {
                printf("process A%d is run\n", a);
                ka = 1;
            }
        }


        /* this block initializes B another new period*/
        if (T % cycA != 0 && T % cycB == 0) {
            B = T;
            printf("when T=%d, ", T);
            printf("process B%d is generated\n", ++b);            //impossible to compete with B
            numb = 0;
            if (numa < serveA) {                         //process A is undone yet
                if (cycB >= cycA) {
                    printf("process A%d is on run\n", a);
                } else {
                    printf("process B%d is to run\n", b);
                    kb = 1;
                    ka = 0;
                }
            } else                                //process A is done
            {
                printf("process B%d is on run\n", b);
                kb = 1;
            }
        }


        /* this block is to accumulate running time*/
        if (ka) {
            numa++;
        }
        if (kb) {
            numb++;
        }
    }
}
