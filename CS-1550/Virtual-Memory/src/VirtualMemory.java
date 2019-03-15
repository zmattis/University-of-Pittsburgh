/**
 * @author Zachary M. Mattis
 * COE 1550
 * Virtual Memory Algorithms
 * July 12, 2018
 *
 * This class provides the implementations of 4 different
 * page replacement algorithms:
 *          Optimal
 *          Clock
 *          FIFO
 *          NRU
 */

import java.io.*;
import java.util.*;

public class VirtualMemory {
  private final int PAGE_SIZE = (int) Math.pow(2, 12); //Page size is 4KB
  private int NumberFrames = 0;
  private String TraceFile = "";

  public VirtualMemory(int numFrames, String traceFile) {
    NumberFrames = numFrames;
    TraceFile = traceFile;
  }


  /**
   * Simulates an optimal page replacement algorithm
   * by parsing future memory accesses to make
   * optimizations to the paging.
   */
  public void optimal(){
    File f = new File(TraceFile);
    PTE[] RAM = new PTE[NumberFrames]; //Physical memory that holds the frames
    PTE[] pageTable = generateNewPageTable();
    LinkedList[] nextAccessed = new LinkedList[pageTable.length]; //Used for preprocessing the optimal algorithm so we can find the page that is accessed the furthest in the future
    Scanner sc;
    try{
      sc = new Scanner(f);
    } catch(FileNotFoundException e){
      System.out.println("File doesn't exist!");
      return;
    }

    String algorithm = "Optimal";
    int memoryAccesses = 0;
    int pageFaults = 0;
    int diskWrites = 0;
    int currFramesLoaded = 0; //We have loaded 0 pages into our NumberFrames frames

    int instructionNumber = 0;
    //Pre-parse the memory addresses and optimize
    while(sc.hasNextLine()){
      //Parse the address of the instruction
      String line = sc.nextLine();
      String addrString = "0x" + line.substring(0, 8);
      long addr = Long.decode(addrString);

      int pageNumber = (int) (addr/PAGE_SIZE);

      if(nextAccessed[pageNumber] == null) nextAccessed[pageNumber] = new LinkedList(); //This list of future addresses doesn't exist yet, so set it to a new object instance
      nextAccessed[pageNumber].add(instructionNumber); //Next instruction that will use this page
      instructionNumber++;
    }

    try{
      sc = new Scanner(f);
    } catch(FileNotFoundException e){
      System.out.println("File doesn't exist!");
      return;
    }

    while(sc.hasNextLine()){
      //Parse the memory address and operation ('R' or 'W')
      String line = sc.nextLine();
      String addrString = "0x" + line.substring(0, 8);

      long addr = Long.decode(addrString);
      char operation = line.charAt(9);
      memoryAccesses++;

      String actionTaken;
      int pageNumber = (int) addr/PAGE_SIZE;
      if(pageNumber >= pageTable.length || pageNumber < 0){
        actionTaken = "page fault - no eviction";
      } else if(currFramesLoaded < NumberFrames){ //Just insert or directly modify pages already in physical memory
        boolean pageAlreadyLoaded = pageTable[pageNumber].getFrame() != -1; //If the frame number of the PTE is not -1, then it is currently in RAM
        nextAccessed[pageNumber].remove(); //Remove this current address so we can find when this page is accessed next in the page replacement algorithm

        if(pageAlreadyLoaded){
          int frameNumberOfPage = pageTable[pageNumber].getFrame();
          if(operation == 'W') RAM[frameNumberOfPage].setDirty(true);
          //If it's already loaded, then the referenced and valid bits will already be 'true'
          actionTaken = "hit";
        } else{
          //The page is not loaded yet and the RAM still has empty slots, so load the page into the next free frame
          RAM[currFramesLoaded] = pageTable[pageNumber];
          if(operation == 'W') RAM[currFramesLoaded].setDirty(true);
          RAM[currFramesLoaded].setReferenced(true);
          RAM[currFramesLoaded].setValid(true);
          RAM[currFramesLoaded].setFrame(currFramesLoaded);
          actionTaken = "page fault - no action";
          currFramesLoaded++;
          pageFaults++;
        }
      } else {
        boolean pageAlreadyLoaded = pageTable[pageNumber].getFrame() != -1; //If the frame number of the PTE is not -1, then it is currently in RAM
        nextAccessed[pageNumber].remove(); //Remove this current address so we can find when this page is accessed next in the page replacement algorithm

        if(pageAlreadyLoaded){
          int frameNumberOfPage = pageTable[pageNumber].getFrame();
          if(operation == 'W') RAM[frameNumberOfPage].setDirty(true);
          //If it's already loaded, then the referenced and valid bits will already be 'true'
          actionTaken = "hit";
        } else{
          int evictedFrameNumber = 0; //Start at frame 0 and iterate over the frames to find the one that is accessed furthest in the future
          for(int i = 0; i < RAM.length; i++){ //Search for page that is references furthest in the future; i = the frame number to check the PTE's occurrances
            int thisPageNumber = RAM[i].getIndex(); //Get the index of this frame's page in order to find its linked list of memory references
            if(nextAccessed[thisPageNumber].peek() == null){ //Find the next time this page is referenced
              evictedFrameNumber = i; //If it has a null linked list, there are no more references to it, so it's the optimal eviction
              break;
            }
            int nextAppearance = (int) nextAccessed[thisPageNumber].peek(); //Next instruction that this page is accessed
            int farthestInstructionsForNextReference = (int) nextAccessed[RAM[evictedFrameNumber].getIndex()].peek(); //The current farthest instruction in the future to compare this current page to
            if(nextAppearance > farthestInstructionsForNextReference){ //Set the new page that will be accessed furthest in the future
              evictedFrameNumber = i; //Set new min to this frame's page index
            }
          }

          if(RAM[evictedFrameNumber].getDirty()){
            diskWrites++; //If the page is dirty, we have to write the page data to disk
            actionTaken = "page fault - evict dirty";
          } else{
            actionTaken = "page fault - evict clean";
          }
          //Reset the evicted PTE's properties
          RAM[evictedFrameNumber].setDirty(false);
          RAM[evictedFrameNumber].setReferenced(false);
          RAM[evictedFrameNumber].setValid(false);
          RAM[evictedFrameNumber].setFrame(-1);

          //Set the new PTE's properties
          RAM[evictedFrameNumber] = pageTable[pageNumber]; //Replace evicted page with new page
          if(operation == 'W') RAM[evictedFrameNumber].setDirty(true);
          RAM[evictedFrameNumber].setReferenced(true);
          RAM[evictedFrameNumber].setValid(true);
          RAM[evictedFrameNumber].setFrame(evictedFrameNumber);
          pageFaults++;
        }
      }

      System.out.println(addrString + " -- action: " + actionTaken);
    }

    sc.close();
    display(algorithm, memoryAccesses, pageFaults, diskWrites);
  }

  /**
   * Simulates NRU page replacement algorithm. Utilizes R and D
   * bits to prioritize pages to evict and evicts page with the
   * highest priority.
   */
  public void nru(int refreshRate){   //Number of instructions until all of the referenced bits flip to 0
    File f = new File(TraceFile);
    PTE[] RAM = new PTE[NumberFrames]; //Physical memory
    PTE[] pageTable = generateNewPageTable();
    if(refreshRate <= 0){
      refreshRate = 50;       //default
    }
    Scanner sc;
    try{
      sc = new Scanner(f);
    } catch(FileNotFoundException e){
      System.out.println("File doesn't exist!");
      return;
    }

    String algorithm = "NRU";
    int memoryAccesses = 0;
    int pageFaults = 0;
    int diskWrites = 0;
    int currFramesLoaded = 0; //We have loaded 0 pages into our NumberFrames frames; helps with quicker inserts into RAM at the beginning of the simulation

    while(sc.hasNextLine()){
      //Parse the memory address and operation ('R' or 'W')
      String line = sc.nextLine();
      String addrString = "0x" + line.substring(0, 8);

      long addr = Long.decode(addrString);
      char operation = line.charAt(9);
      memoryAccesses++;

      String actionTaken;
      int pageNumber = (int) addr/PAGE_SIZE;
      if(pageNumber >= pageTable.length || pageNumber < 0){ //Invalid page number; should never happen
        actionTaken = "page fault - no eviction";
      } else if(currFramesLoaded < NumberFrames){ //Just insert or directly modify pages already in physical memory
        boolean pageAlreadyLoaded = pageTable[pageNumber].getFrame() != -1; //If the frame number of the PTE is not -1, then it is currently in RAM

        if(pageAlreadyLoaded){
          int frameNumberOfPage = pageTable[pageNumber].getFrame();
          if(operation == 'W') RAM[frameNumberOfPage].setDirty(true);
          //If it's already loaded, then the referenced and valid bits will already be 'true'
          actionTaken = "hit";
        } else{ //The page is not yet loaded but the physical memory isn't full yet so just insert into the next free slot
          RAM[currFramesLoaded] = pageTable[pageNumber];
          if(operation == 'W') RAM[currFramesLoaded].setDirty(true);
          RAM[currFramesLoaded].setReferenced(true);
          RAM[currFramesLoaded].setValid(true);
          RAM[currFramesLoaded].setFrame(currFramesLoaded);
          actionTaken = "page fault - no action";
          currFramesLoaded++;
          pageFaults++;
        }
      } else{
        boolean pageAlreadyLoaded = pageTable[pageNumber].getFrame() != -1; //If the frame number of the PTE is not -1, then it is currently in RAM

        if(pageAlreadyLoaded){
          int frameNumberOfPage = pageTable[pageNumber].getFrame();
          if(operation == 'W') RAM[frameNumberOfPage].setDirty(true);
          //If it's already loaded, then the referenced and valid bits will already be 'true'
          actionTaken = "hit";
        } else{
          int evictedFrameNumber = 0; //Choose a random frame number to evict its page
          int priorityOfEvicted = 0; //0 = highest priority, 3 = lowest priority

          //Set the priority of the initial frame
          if(RAM[evictedFrameNumber].getDirty()){
            priorityOfEvicted |= 1; //OR with 01
          }
          if(RAM[evictedFrameNumber].getReferenced()){
            priorityOfEvicted |= 2; //OR with 10
          }

          //Generate the priorities of all of the frames and evict the one with the highest priority
          for(int i = 0; i < RAM.length; i++){
            int priorityOfFrame = 0;
            if(RAM[i].getDirty()){
              priorityOfFrame |= 1; //OR with 01
            }
            if(RAM[i].getReferenced()){
              priorityOfFrame |= 2; //OR with 10
            }

            if(priorityOfFrame < priorityOfEvicted){ //Set new highest priority frame
              evictedFrameNumber = i;
              priorityOfEvicted = priorityOfFrame;
            }
            if(priorityOfEvicted == 0) break; //Found a frame with the lowest priority, so evict it
          }

          if(RAM[evictedFrameNumber].getDirty()){
            diskWrites++; //If the page is dirty, we have to write the page data to disk
            actionTaken = "page fault - evict dirty";
          } else{
            actionTaken = "page fault - evict clean";
          }
          //Reset the evicted PTE's properties
          RAM[evictedFrameNumber].setDirty(false);
          RAM[evictedFrameNumber].setReferenced(false);
          RAM[evictedFrameNumber].setValid(false);
          RAM[evictedFrameNumber].setFrame(-1);

          //Set the new PTE's properties
          RAM[evictedFrameNumber] = pageTable[pageNumber]; //Replace evicted page with new page
          if(operation == 'W') RAM[evictedFrameNumber].setDirty(true);
          RAM[evictedFrameNumber].setReferenced(true);
          RAM[evictedFrameNumber].setValid(true);
          RAM[evictedFrameNumber].setFrame(evictedFrameNumber);
          pageFaults++;
        }
      }

      //Reset all of the referenced bits after the reset time interval is over
      if(memoryAccesses % refreshRate == 0){
        for(int i = 0; i < RAM.length && i < currFramesLoaded; i++){
          RAM[i].setReferenced(false);
        }
      }

      System.out.println(addrString + " -- action: " + actionTaken);
    }

    sc.close();
    display(algorithm, memoryAccesses, pageFaults, diskWrites);
  }

  /**
     * Simulates clocking page replacement algorithm. Utilizes R bit of
     * index to determine which page to evict
     */
  public void clock(){ //Number of instructions until all of the referenced bits flip to 0
    File f = new File(TraceFile);
    PTE[] RAM = new PTE[NumberFrames]; //Physical memory that holds the frames
    PTE[] pageTable = generateNewPageTable();
    int pointer = 0; //Current pointer in the "circular queue"-like data structure for the Clock algorithm
    Scanner sc;
    try{
      sc = new Scanner(f);
    } catch(FileNotFoundException e){
      System.out.println("File doesn't exist!");
      return;
    }

    String algorithm = "Clock";
    int memoryAccesses = 0;
    int pageFaults = 0;
    int diskWrites = 0;
    int currFramesLoaded = 0; //We have loaded 0 pages into our NumberFrames frames

    while(sc.hasNextLine()){
      //Parse the memory address and operation ('R' or 'W')
      String line = sc.nextLine();
      String addrString = "0x" + line.substring(0, 8);

      long addr = Long.decode(addrString);
      char operation = line.charAt(9);
      memoryAccesses++;

      String actionTaken;
      int pageNumber = (int) addr/PAGE_SIZE;

      if(pageNumber >= pageTable.length || pageNumber < 0){ //Invalid page number; should never happen
        actionTaken = "page fault - no eviction";
      } else if(currFramesLoaded < NumberFrames){ //Just insert or directly modify pages already in physical memory
        boolean pageAlreadyLoaded = pageTable[pageNumber].getFrame() != -1; //If the frame number of the PTE is not -1, then it is currently in RAM

        if(pageAlreadyLoaded){
          int frameNumberOfPage = pageTable[pageNumber].getFrame();
          if(operation == 'W') RAM[frameNumberOfPage].setDirty(true);
          RAM[frameNumberOfPage].setReferenced(true); //Since this is the Clock algorithm, make sure the page is definitely referenced again
          //If it's already loaded, then the referenced and valid bits will already be 'true'
          actionTaken = "hit";
        } else{
          //If the page isn't already loaded, load it into the next slot in RAM since it's not full yet
          RAM[currFramesLoaded] = pageTable[pageNumber];
          if(operation == 'W') RAM[currFramesLoaded].setDirty(true);
          RAM[currFramesLoaded].setReferenced(true);
          RAM[currFramesLoaded].setValid(true);
          RAM[currFramesLoaded].setFrame(currFramesLoaded);
          actionTaken = "page fault - no action";
          currFramesLoaded++;
          pageFaults++;
        }
      } else{
        boolean pageAlreadyLoaded = pageTable[pageNumber].getFrame() != -1; //If the frame number of the PTE is not -1, then it is currently in RAM

        if(pageAlreadyLoaded){
          int frameNumberOfPage = pageTable[pageNumber].getFrame();
          if(operation == 'W') RAM[frameNumberOfPage].setDirty(true);
          RAM[frameNumberOfPage].setReferenced(true); //Since this is the Clock algorithm, make sure the page is definitely referenced again
          //If it's already loaded, then the referenced and valid bits will already be 'true'
          actionTaken = "hit";
        } else{
          int evictedFrameNumber = -1; //Choose a random frame number to evict its page
          while(evictedFrameNumber == -1){ //Haven't found a page to evict yet
            if(RAM[pointer].getReferenced()){
              RAM[pointer].setReferenced(false); //Give the frame a second-chance and set its referenced bit to 0
            } else{
              evictedFrameNumber = pointer; //If it's not referenced, evict it
            }

            if(pointer < NumberFrames-1){ //Move to the next spot in the circular queue
              pointer++;
            } else{ //Reset the index we're at in the circular queue because we "looped around" to the front
              pointer = 0;
            }
          }

          if(RAM[evictedFrameNumber].getDirty()){
            diskWrites++; //If the page is dirty, we have to write the page data to disk
            actionTaken = "page fault - evict dirty";
          } else{
            actionTaken = "page fault - evict clean";
          }
          //Reset the evicted PTE's properties
          RAM[evictedFrameNumber].setDirty(false);
          RAM[evictedFrameNumber].setReferenced(false);
          RAM[evictedFrameNumber].setValid(false);
          RAM[evictedFrameNumber].setFrame(-1);

          //Set the new PTE's properties
          RAM[evictedFrameNumber] = pageTable[pageNumber]; //Replace evicted page with new page
          if(operation == 'W') RAM[evictedFrameNumber].setDirty(true);
          RAM[evictedFrameNumber].setReferenced(true);
          RAM[evictedFrameNumber].setValid(true);
          RAM[evictedFrameNumber].setFrame(evictedFrameNumber);
          pageFaults++;
        }
      }

      System.out.println(addrString + " -- action: " + actionTaken);
    }

    sc.close();
    display(algorithm, memoryAccesses, pageFaults, diskWrites);
  }


  /**
   * Simulates the FIFO page replacement algorithm. Implements
   * a standard queue to decide which frame to replace.
   */
  public void fifo(){
    File f = new File(TraceFile);
    PTE[] RAM = new PTE[NumberFrames];          //Physical Memory
        PTE[] pageTable = generateNewPageTable();   //Page Table

    Scanner sc;
    try{
      sc = new Scanner(f);
    } catch(FileNotFoundException e){
      System.out.println("File doesn't exist!");
      return;
    }

    String algorithm = "FIFO";
    int memoryAccesses = 0;
    int pageFaults = 0;
    int diskWrites = 0;
        int currFramesLoaded = 0;
        LinkedList<Integer> queue = new LinkedList<Integer>();

    while(sc.hasNextLine()){
      //Parse the memory address and operation ('R' or 'W')
      String line = sc.nextLine();
      String addrString = "0x" + line.substring(0, 8);

      long addr = Long.decode(addrString);
      char operation = line.charAt(9);
      memoryAccesses++;

      String actionTaken;
      int pageNumber = (int) addr/PAGE_SIZE;
      if(pageNumber >= pageTable.length || pageNumber < 0){ //Invalid page number
        actionTaken = "page fault - no eviction";
      } else if(currFramesLoaded < NumberFrames){     // modify pages already in physical memory
        boolean pageAlreadyLoaded = pageTable[pageNumber].getFrame() != -1; //If the frame number of the PTE is not -1, then it is currently in RAM

        if(pageAlreadyLoaded){
          int frameNumberOfPage = pageTable[pageNumber].getFrame();
          if(operation == 'W') RAM[frameNumberOfPage].setDirty(true);
          //If it's already loaded, then the referenced and valid bits will already be 'true'
          actionTaken = "hit";
        } else{ //The page is not yet loaded but the physical memory isn't full yet so just insert into the next free slot
          RAM[currFramesLoaded] = pageTable[pageNumber];
          if(operation == 'W') {
                        RAM[currFramesLoaded].setDirty(true);
                    }
          RAM[currFramesLoaded].setReferenced(true);
          RAM[currFramesLoaded].setValid(true);
                    RAM[currFramesLoaded].setFrame(currFramesLoaded);
                    queue.add(currFramesLoaded);
          actionTaken = "page fault - no action";
          currFramesLoaded++;
          pageFaults++;
        }
      } else {
        boolean pageAlreadyLoaded = pageTable[pageNumber].getFrame() != -1; //If the frame number of the PTE is not -1, then it is currently in RAM

        if(pageAlreadyLoaded){
          int frameNumberOfPage = pageTable[pageNumber].getFrame();
          if(operation == 'W') {
                        RAM[frameNumberOfPage].setDirty(true);
                    }
          actionTaken = "hit";
        } else {
                    int evictedFrameNumber = 0; //Take from queue

                    evictedFrameNumber = queue.poll();
                    queue.add(evictedFrameNumber);

                    if(RAM[evictedFrameNumber].getDirty()){
            diskWrites++; //If the page is dirty, we have to write the page data to disk
            actionTaken = "page fault - evict dirty";
          } else{
            actionTaken = "page fault - evict clean";
          }
                    //Reset the evicted PTE's properties
          RAM[evictedFrameNumber].setDirty(false);
          RAM[evictedFrameNumber].setReferenced(false);
          RAM[evictedFrameNumber].setValid(false);
          RAM[evictedFrameNumber].setFrame(-1);

          //Set the new PTE's properties
          RAM[evictedFrameNumber] = pageTable[pageNumber]; //Replace evicted page with new page
          if(operation == 'W') RAM[evictedFrameNumber].setDirty(true);
          RAM[evictedFrameNumber].setReferenced(true);
          RAM[evictedFrameNumber].setValid(true);
          RAM[evictedFrameNumber].setFrame(evictedFrameNumber);
                    pageFaults++;

        }
      }

      System.out.println(addrString + " -- action: " + actionTaken);
    }

    sc.close();
    display(algorithm, memoryAccesses, pageFaults, diskWrites);
  }

  /**
   * Displays algorithm statistics to standard output
   * @param algorithm         Algorithm used for Page Replacment
   * @param memoryAccesses    Total number of memory accesses
   * @param pageFaults        Number of page faults
   * @param diskWrites        Number of writes back to disk
   */
  private void display(String algorithm, int memoryAccesses, int pageFaults, int diskWrites){
    System.out.println();
    System.out.println("Algorithm:              " + algorithm);
    System.out.println("Number of frames:       " + NumberFrames);
    System.out.println("Total memory accesses:  " + memoryAccesses);
    System.out.println("Total page faults:      " + pageFaults);
    System.out.println("Total writes to disk:   " + diskWrites);
  }

  /**
   * Generates and initializes a new page table
   * @return New Page Table
   */
  private PTE[] generateNewPageTable(){
    int addrWidth = 32;
    int numEntries = (int) (Math.pow(2, addrWidth)/PAGE_SIZE); //Number of entries in the page table
    PTE[] table = new PTE[numEntries];

    for(int i = 0; i < table.length; i++){
      PTE newEntry = new PTE();
      newEntry.setIndex(i);
      table[i] = newEntry;
    }

    return table;
  }
}
