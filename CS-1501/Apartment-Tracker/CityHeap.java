/**
 * @author Zachary M. Mattis
 * CS 1501
 * Assignment 3
 * November 6, 2017
 *
 * This class functions as Priority Queue similar to Heap.java,
 * but it does not use an indirection data structure. Does not
 * contain sink methods as only worried about highest priority items.
 *
 */

public class CityHeap {

  private ApartmentInfo[] heap;   //Apartment Info heap array
  private int cnt;           // where to insert next entry into heap
  private boolean minFlag;   // true if min heap, false if max heap


	/**
	 * Creates a new Heap by creating a new, heap
	 */
	public CityHeap(boolean f) {

    heap = new ApartmentInfo[32];		// Initialize a new heap
    cnt = 0;
    minFlag = f;
  }


  /**
	 * Gets the object of highest priority
	 * @return The apartment with the lowest price
	 */
	public ApartmentInfo getMinPrice() {

		return heap[0];
	}

  /**
	 * Gets the object of highest priority
	 * @return The apartment with the highest sq ft
	 */
	public ApartmentInfo getMaxSqFt() {

		return heap[0];
	}

  /**
	 * Adds an ApartmentInfo object to the heap; also updates the trie with the apartment's index in the heap
	 * @param apt The Apartment to add to the heap
	 */
	public void addApt(ApartmentInfo apt) {

		if (cnt == heap.length) {			// Resizes the array if there is no room available
			heap = resizeHeap(heap);
		}

		heap[cnt] = apt;				// Adds the Apartment to the next available leaf position

    int parentIndex = (int)Math.floor((cnt - 1) / 2.0);			// Calculates the parent index using the leaf index

    if ( getMode() ){   //min heap
      floatByPrice(parentIndex, cnt);
    }
    else{   //max heap
      floatBySqFt(parentIndex, cnt);
    }

		cnt++;							// Increments the index of the next available leaf position
	}

    /**
	 * Floats a AptInfo object by price
	 * @param parentIndex The index of the apt's parent
	 * @param insertedIndex The index of the current apt
	 */
	private void floatByPrice(int parentIndex, int insertedIndex) {

		if (parentIndex < 0) {					// If the parent index is less than zero, then the current apt is located at the root
			return;
		}

		double parentPrice = heap[parentIndex].getCost();				// Gets the price of the parent apt
		double insertedPrice = heap[insertedIndex].getCost();			// Gets the price of the current apt

		if (parentPrice > insertedPrice) {						// Swaps the parent and current apts if the parent price is greater than the current price
			ApartmentInfo tempApt = heap[parentIndex];
			heap[parentIndex] = heap[insertedIndex];
			heap[insertedIndex] = tempApt;

			insertedIndex = parentIndex;								// Sets the current apt's index as the parent index
			parentIndex = (int)Math.floor((parentIndex - 1) / 2.0);		// Calculates a new parent index from the current apt's new index

			floatByPrice(parentIndex, insertedIndex);				// Recursively calls the float method
		}

	}


  /**
   * Floats an ApartmentInfo object by sq ft
   * @param parentIndex The index of the apt's parent
   * @param insertedIndex The index of the current apt
   */
    private void floatBySqFt(int parentIndex, int insertedIndex) {

    	if (parentIndex < 0) {
    		return;
    	}

    	int parentSqFt = heap[parentIndex].getSqFt();
    	int insertedSqFt = heap[insertedIndex].getSqFt();

    	if (parentSqFt < insertedSqFt) {
    		ApartmentInfo tempApt = heap[parentIndex];
    		heap[parentIndex] = heap[insertedIndex];
    		heap[insertedIndex] = tempApt;

    		insertedIndex = parentIndex;
    		parentIndex = (int)Math.floor((parentIndex - 1) / 2.0);

    		floatBySqFt(parentIndex, insertedIndex);
    	}

    }

    /**
  	 * Gets the apt from the specified index in the heap array
  	 * @param index The index of the apt
  	 * @return The ApartmentInfo object at the index in the array
  	 */
  	public ApartmentInfo getApt(int index) {

  		return heap[index];
  	}


  /**
  * Sets the apt at the specified index in the heap array
  * @param apt The ApartmentInfo object to set
  * @param index The index to set the apt in the array
  */
  public void setApt(ApartmentInfo apt, int index) {

  heap[index] = apt;
  }

  /**
  	 * Gets the heap's method of determining priority
  	 * @return The flag representing the priority mode; 'true' for min or 'false' for max
  	 */
  	public boolean getMode() {

  		return minFlag;
  	}


    /**
  	 * Resizes the heap's array its maximum size has been exceeded
  	 * @param oldHeap The current array that has not been resized
  	 * @return The new array of doubled size
  	 */
  	private ApartmentInfo[] resizeHeap(ApartmentInfo[] oldHeap) {

  		int doubledSize = oldHeap.length * 2;
  		ApartmentInfo[] newHeap = new ApartmentInfo[doubledSize];		// Creates a new array by doubling the size of the old array

  		for (int i = 0; i < oldHeap.length; i++) {			// Copies all objects from the old array to the new array
  			newHeap[i] = oldHeap[i];
  		}

  		return newHeap;
  	}

}
