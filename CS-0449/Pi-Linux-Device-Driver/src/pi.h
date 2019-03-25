/**
 * @author Zachary M. Mattis
 * CS 0449
 * Pi Linux Device Driver
 * July 26, 2017
 *
 * This C header file provides the function
 * prototypes and macros for producing
 * the digits of pi.
 */

#ifndef __PI_H
#define __PI_H

#define SCALE 10000
#define ARRINIT 2000


//I define malloc as a macro so you can test out the code without building
//it into the kernel module, just to get your feet wet.
#ifdef GFP_KERNEL
#define MALLOC(s) kmalloc(s, GFP_KERNEL)
#define FREE(s) kfree(s)
#else
#define MALLOC(s) malloc(s)
#define FREE(s) free(s)
#endif /* GFP_KERNEL */

void pi(char *buffer, int m)
{
    int i, j;
    int carry = 0;
    //it seems we need 14 'iterations' to get 4 digits correctly.
    int max = (m/4 ) * 14;
    int *arr = (int *)MALLOC(sizeof(int)*(max+15));
    strcpy(buffer,"");

    //Here there be dragons...
    //I have no clue how this algorithm works
    for (i = 0; i <= max; ++i)
        arr[i] = ARRINIT;
    for (i = max; i>0; i -= 14)
    {
        int sum = 0;
        char temp[5];

        for (j = i; j > 0; --j)
        {
            sum = sum*j + SCALE*arr[j];
            arr[j] = sum % (j*2-1);
            sum /= (j*2-1);
        }
        //we seem to generate 4 digits at a time.
        sprintf(temp, "%04d", carry + sum/SCALE);
        strcat(buffer, temp);
        carry = sum % SCALE;
    }
    FREE(arr);
}

#endif /* __PI_H */
