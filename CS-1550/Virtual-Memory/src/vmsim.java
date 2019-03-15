/**
 * @author Zachary M. Mattis
 * COE 1550
 * Assignment 3
 * July 12, 2018
 *
 * This Java file is a virtual memory simulator that
 * evaluates 4 different page replacement algorithms
 *
 *      Opt   = Optimal paging algorithm if perfect knowledge
 *      Clock = Second-chance
 *      FIFO  = First in, First out
 *      NRU   = Not recently used; R,D bits
 *
 * Page Table implementation supports a physical memory size
 * of 4MB, assuming a 32-bit virtual address space with 4 KB
 * pages. This implies a maximum of 1024 frames. These do not
 * support any kind of R/W protections
 */

public class vmsim {
    private static final String USAGE = "USAGE:\n\tjava vmsim -n <numframes> -a <opt|clock|fifo|nru> [-r <refresh>] <tracefile>";


  public static void main(String[] args){
    int numFrames = -1;
    int refresh = -1;
    String algorithm = "";
    String traceFile = "";

    if(args.length == 5){ //Arguments without the optional refresh argument
      if(!args[0].equals("-n")) fail();
      numFrames = Integer.parseInt(args[1]);
      if(!args[2].equals("-a")) fail();
      algorithm = args[3];
      traceFile = args[4];
    } else if(args.length == 7){ //Arguments with the optional refresh argument
      if(!args[0].equals("-n")) fail();
      numFrames = Integer.parseInt(args[1]);
      if(!args[2].equals("-a")) fail();
      algorithm = args[3];
      if(!args[4].equals("-r")) fail();
      refresh = Integer.parseInt(args[5]);
      traceFile = args[6];
    } else {
      fail();
    }

    VirtualMemory simulation = new VirtualMemory(numFrames, traceFile);

        if( algorithm.equals("opt") ){
            simulation.optimal();
        } else if ( algorithm.equals("clock") ) {
            simulation.clock();
        } else if ( algorithm.equals("fifo") ) {
            simulation.fifo();
        } else if ( algorithm.equals("nru") ) {
            simulation.nru(refresh);
        } else {
            fail();
        }

    }

    /**
     * Print command line usage information to standard error
     */
    private static void fail() {
        System.err.println(USAGE);
        System.exit(-1);
    }
}