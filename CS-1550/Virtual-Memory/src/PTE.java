/**
 * @author Zachary M. Mattis
 * COE 1550
 * Page Table Entry
 * July 12, 2018
 *
 * This class functions as a page table entry,
 * providing an index, frame number, as well as
 * additional bit support.
 */

public class PTE {
  private int     index;
  private int     frame;
  private boolean dirty;
  private boolean valid;
  private boolean referenced;

  /**
   * Constructor: initialize values
   */
  public PTE() {
    index = -1;
    frame = -1;
    dirty = false;
    valid = false;
    referenced = false;
  }

  /**
   * Returns dirty bit
   * @return dirty bit
   */
  public boolean getDirty() {
    return dirty;
  }

  /**
   * Returns reference bit
   * @return reference bit
   */
  public boolean getReferenced() {
    return referenced;
  }

  /**
   * Returns valid bit
   * @return valid bit
   */
  public boolean getValid() {
    return valid;
  }

  /**
   * Returns index of entry in page table
   * @return PTE index
   */
  public int getIndex() {
    return index;
  }

  /**
   * Returns index of frame in physical memory
   * @return frame index
   */
  public int getFrame() {
    return frame;
  }

  /**
   * Sets dirty bit
   * @param val   dirty bit
   */
  public void setDirty(boolean val) {
    dirty = val;
  }

  /**
   * Sets reference bit
   * @param val   reference bit
   */
  public void setReferenced(boolean val) {
    referenced = val;
  }

  /**
   * Sets valid bit
   * @param val   valid bit
   */
  public void setValid(boolean val) {
    valid = val;
  }

  /**
   * Sets PTE index
   * @param val   page table index
   */
  public void setIndex(int val) {
    index = val;
  }

  /**
   * Sets frame in physical memory
   * @param val   frame index
   */
  public void setFrame(int val) {
    frame = val;
    }

}
