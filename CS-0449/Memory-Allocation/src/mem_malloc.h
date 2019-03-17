/**
 * @author Zachary M. Mattis
 * CS 0449
 * Memory Allocation
 * July 10, 2017
 *
 * This C header file provides the function
 * prototypes for two heap operators:
 *
 *    mem_nextfit_malloc - allocates memory using the next-fit algorithm
 *    mem_free           - deallocates a pointer that was originally allocated mem
 */

#ifndef __MEM_MALLOC_H
#define __MEM_MALLOC_H

void *mem_nextfit_malloc( int );
void mem_free( void * );

#endif /* __MEM_MALLOC_H */
