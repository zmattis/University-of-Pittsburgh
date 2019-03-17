/**
 * @author Zachary M. Mattis
 * CS 0449
 * Memory Allocation
 * July 10, 2017
 *
 * This C file provides the implementation for
 * the custom memory heap allocation functions
 * via manipulation of the brk ptr.
 */


/* Libraries */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "mem_malloc.h"

/* Uncomment this definition to use as a driver */
//#define MAIN
/* Uncomment this definition to use as a debugger */
//#define DEBUG

/* Node Structure to hold LL parameters */
struct Node {
  int free;  /* Space is free if flag != 0 */
  int size;
  struct Node* previous_node;
  struct Node* next_node;
};

/* Function Prototypes */
struct Node* new_node(int);
struct Node* find_nextfit(int);
void print_list();


/*
 * ---------------------------------------------------------------------------------
 * Global variable "head" to keep track of the beginning position of the linked list
 * Global variable "curr" to keep track of the position where search left off
 * Global variable "tail" to keep track of the end position of the linked list
 * ---------------------------------------------------------------------------------
 */
struct Node *head;
struct Node *curr;
struct Node *tail;


void *mem_nextfit_malloc(int size) {
  struct Node* node_addr;
  if(head==NULL) {  //the heap is empty, add a node
    head=(struct Node *)sbrk(size+sizeof(struct Node)); //node start
    head->free=0;
    head->previous_node=NULL;
    head->next_node=NULL;
    head->size=size;
    curr=head; //nextfit curr
    tail=head;
    #ifdef DEBUG
    printf("creating the first node...\n");
    #endif
    return (void*)head+sizeof(struct Node);             //only return usable space
  }
  else {
    node_addr=find_nextfit(size);
    if(node_addr==NULL) {  //if cant find best fit, create new node/move brk up
      #ifdef DEBUG
      printf("creating a new node...\n");
      #endif
      return (void*)new_node(size)+sizeof(struct Node);  //only return usable space of the new node
    }
    else {  //if it finds a best fit
      node_addr->free=0;  //mark as not free
      return (void*)node_addr+sizeof(struct Node); //only return usable space
    }

  }
}

/*
 * --------------------------------------------------------------------------
 * If no empty space is big enough, allocate more space by extending the heap
 * via sbrk(). Move brk and initialize a new block.
 * --------------------------------------------------------------------------
 */
struct Node* new_node(int size) {
  tail->next_node = (struct Node*)sbrk(sizeof(struct Node)+size);
  tail->next_node->previous_node=tail;
  tail->next_node->size=size;
  tail->next_node->next_node=NULL;  //new tail
  tail->next_node->free=0;          //new node is always not free
  tail=tail->next_node;             //pass to the tail
  return tail;
}

/*
 * -------------------------------------------------------------------------------------------------------------
 * Find chunk of using nextfit algorithm, starting from where the least search left off ("*curr"), wrapping
 * back around the heap if necessary. Continue testing each block to see if it can accommodate the size request,
 * or else reach end of the heap.
 * -------------------------------------------------------------------------------------------------------------
 */
struct Node* find_nextfit(int size) {
  struct Node* node_addr=curr;

  while(node_addr!=NULL) {
    if( node_addr->free && (node_addr->size >= size)) {  //if node has enough free space
      curr=node_addr;
      return node_addr;
    }
    else {
      node_addr = node_addr -> next_node;

      if (node_addr==NULL) {
        node_addr=head;
      }
      if (node_addr == curr) {
        return NULL;
      }
    }
  }

  return node_addr;

}


void mem_free(void *ptr) {
  struct Node* node_to_free=(struct Node*)(ptr-sizeof(struct Node));
  int neighbours;

  /* Check to see if node is already free */
  if(node_to_free->free || ptr==NULL) {
    return;
  }

  /* Base Case ( Middle Node ) */
  if( (node_to_free!=head) && (node_to_free!=tail) ) {
    #ifdef DEBUG
    printf("freeing normal case\n");
    #endif
    neighbours=(( node_to_free->previous_node->free)<<1)|(node_to_free->next_node->free);  //00 01 10 11
    switch(neighbours) {
      case 3:  //both are free
        #ifdef DEBUG
        printf("x merge three nodes\n");
        #endif
        node_to_free->previous_node->size=node_to_free->previous_node->size+node_to_free->size+node_to_free->next_node->size+2*sizeof(struct Node);
        node_to_free->previous_node->next_node=node_to_free->next_node->next_node;
        node_to_free->next_node->next_node->previous_node=node_to_free->previous_node;
      break;
      case 2:  //previous node is free
        #ifdef DEBUG
        printf("x merge previous free\n");
        #endif
        node_to_free->previous_node->size=node_to_free->previous_node->size+node_to_free->size+sizeof(struct Node);
        node_to_free->previous_node->next_node=node_to_free->next_node;
        node_to_free->next_node->previous_node=node_to_free->previous_node;
      break;
      case 1:  //next node is free
        #ifdef DEBUG
        printf("x merge next free\n");
        #endif
        node_to_free->size=node_to_free->size+node_to_free->next_node->size+sizeof(struct Node);
        node_to_free->next_node->next_node->previous_node=node_to_free;
        node_to_free->next_node=node_to_free->next_node->next_node;
        node_to_free->free=1;
      break;
      case 0:  //none is free
        node_to_free->free=1;  //only free itself
        #ifdef DEBUG
        printf("x free itself\n");
        #endif
      break;
    }
  }
  else {
    #ifdef DEBUG
    printf("corner cases\n");
    #endif
    if(head==tail) {  //only one node
      #ifdef DEBUG
      printf("freeing the last node\n");
      #endif
      //sbrk(0 - (head->size) - ( sizeof(struct Node) ) );//shrink heap
      brk(head);
      head=NULL;
      tail=head;
      #ifdef DEBUG
      printf("x heap is gone\n");
      #endif
    }
    else {	//more than one node
      if(node_to_free==head) {
        #ifdef DEBUG
        printf("freeing head\n");
        #endif
        if(node_to_free->next_node->free) {  //merge next
          #ifdef DEBUG
          printf("freeing head and merge next\n");
          #endif
          if(node_to_free->next_node==tail) {
            sbrk(0 - (head->size) - (tail->size) - ( 2*sizeof(struct Node) ) );  //shrink heap
            #ifdef DEBUG
            printf("x heap is gone\n");
            #endif
          }
          head->size=head->size+head->next_node->size+sizeof(struct Node);
          head->free=1;
          head->next_node->next_node->previous_node=head;
          head->next_node=head->next_node->next_node;
        }
        else {
          #ifdef DEBUG
          printf("x freeing head only\n");
          #endif
          head->free=1;//only free head node
        }
      }
      if(node_to_free==tail) {
        #ifdef DEBUG
        printf("freeing tail\n");
        #endif
        if(tail->previous_node->free) {
          #ifdef DEBUG
          printf("combine previous with tail and free both\n");
          #endif
          if(tail->previous_node==head) {
            //sbrk(0 - (tail->size) - (tail->previous_node->size) - ( 2*sizeof(struct Node) ) );//shrink heap
            brk( tail -> previous_node - sizeof(struct Node) );
            head=NULL;//heap is gone
            tail=head;
            #ifdef DEBUG
            printf("x heap is gone");
            #endif
          }
          else {
            #ifdef DEBUG
            printf("x previous is not the head\n");
            #endif
            sbrk(0 - (tail->size) - (tail->previous_node->size ) - ( 2*sizeof(struct Node) ) );//shrink heap and combine previous_node
            //brk( tail -> previous_node - sizeof(struct Node ) );
            tail=tail->previous_node->previous_node;
            tail->next_node=NULL;

          }
        }
        else {
          #ifdef DEBUG
          printf("x previous is not free\n");
          #endif
          sbrk(0 - (tail->size) - ( sizeof(struct Node) ) );//shrink heap
          //brk( tail - sizeof(struct Node) );
          tail=tail->previous_node;
          tail->next_node=NULL;
        }
      }
    }
  }

}


/* Print the LL for the heap */
void print_list() {
  struct Node *node=head;
  printf("Forward:");
  while(node!=NULL) {
    if(node->free) {
      printf("free %d",node->size);
    }
    else {
      printf("occupied %d",node->size);
    }
    if(node->next_node!=NULL) {
      printf(" --> ");
    }
    node=node->next_node;
  }
  printf("\n");

  node=tail;
  printf("Reverse: ");
  while(node!=NULL) {
    if(node->free) {
      printf("free %d",node->size);
    }
    else {
      printf("occupied %d",node->size);
    }
    if(node->previous_node!=NULL) {
      printf(" --> ");
    }
    node=node->previous_node;
  }
  printf("\n");


}



/* Define MAIN to use as a driver */
#ifdef MAIN
int main() {
  //init_list();
  void* addr1;void* addr2;void* addr3;void* addr4;void* addr5;
  printf("The bottom of heap is %d\n",sbrk(0));
  //initialize the first dummy node

  addr1=mem_nextfit_malloc(10);
  printf("current addr is %d\n",addr1);
  printf("current brk is %d\n",sbrk(0));

  addr2=mem_nextfit_malloc(10);
  printf("current addr is %d\n",addr2);
  printf("current brk is %d\n",sbrk(0));

  addr3=mem_nextfit_malloc(10);
  printf("current addr is %d\n",addr3);
  printf("current brk is %d\n",sbrk(0));

  addr4=mem_nextfit_malloc(10);
  printf("current addr is %d\n",addr4);
  printf("current brk is %d\n",sbrk(0));

  addr5=mem_nextfit_malloc(10);
  printf("current addr is %d\n",addr5);
  printf("current brk is %d\n",sbrk(0));

  print_list();

  printf("\nfreeing\n");


  my_free(addr4);//free the fourth node

  print_list();

  my_free(addr3);//free the third node
  print_list();




  my_free(addr2);//free the second node
  print_list();
  my_free(addr5);//free the fifth node
  print_list();

  my_free(addr1);//free the first node
  print_list();


  printf("current brk is %d\n",sbrk(0));

  return 0;
}
#endif
