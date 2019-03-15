/**
 * @author Zachary M. Mattis
 * COE 1550
 * Page Table
 * July 12, 2018
 *
 * This class functions as a page table,
 * indexing page table entries (PTE.java)
 */

public class PageTable {
  private final int ADDRESS_WIDTH = 32;
  private final int PAGE_SIZE = (int) Math.pow(2, 12); //Page Size = 4KB
  private PTE[] Table;

  /**
   * Constructor: create new page table
   */
  public PageTable(){
    int numberOfEntries = (int) Math.pow(2, ADDRESS_WIDTH)/PAGE_SIZE; //Number of entries in the page table
    Table = new PTE[numberOfEntries];

    for(int i = 0; i < Table.length; i++){
      PTE newEntry = new PTE();
      Table[i] = newEntry;
    }
  }

  /**
   * Returns Page Table Entry at given index
   * @param index     page table index
   * @return          page table entry
   */
  public PTE get(int index){
    return Table[index];
  }

  /**
   * Returns number of entries in Page Table
   * @return  page table size
   */
  public int size(){
    return Table.length;
  }

}
