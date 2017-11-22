/**
 * @author Zachary M. Mattis
 * CS 1501
 * Assignment 3
 * October 28, 2017
 *
 * This class functions as Priority Queue that is capable of
 * of functioning as both a min and max heap, using the parameters
 * of price as a min, and square footage as a max.
 *
 */

public class Heap {

  private ApartmentInfo[] heap;   //Apartment Info heap array
  private int cnt;           // where to insert next entry into heap
  private boolean minFlag;   // true if min heap, false if max heap


	/**
	 * Creates a new Heap by creating a new, heap
	 */
	public Heap(boolean f) {

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
	 * @param trie The trie to update with the apartment's heap index
	 */
	public void addApt(ApartmentInfo apt, AWayTrie trie) {

		if (cnt == heap.length) {			// Resizes the array if there is no room available
			heap = resizeHeap(heap);
		}

		heap[cnt] = apt;				// Adds the Apartment to the next available leaf position

    int parentIndex = (int)Math.floor((cnt - 1) / 2.0);			// Calculates the parent index using the leaf index

    if ( getMode() ){   //min heap
      floatByPrice(parentIndex, cnt, trie);
    }
    else{   //max heap
      floatBySqFt(parentIndex, cnt, trie);
    }

		cnt++;							// Increments the index of the next available leaf position
	}

  /**
  	 * Removes a apartment object from the heap; also removes its index position from the trie
  	 * @param index The index of the apt to remove from the heap
  	 * @param trie The trie to update
  	 */
  	public void removeApt(int index, AWayTrie trie) {

  		cnt--;							// Decrements the index of the next available leaf position
  		heap[index] = heap[cnt];			// Overwrites the apt in the specified index with the apt in the last leaf

  		if (index == cnt) {					// Conditional if the index is the same as the last leaf
  			trie.removeHeapIndex(heap[index].getTrieAddr());		// Removes the apt's heap index from the trie
  			heap[index] = null;						// Sets the apt in the heap to null
  			return;
  		}
  		else {
  			heap[cnt] = null;				// Sets the last leaf to null

  			if ( getMode() ) {   //min heap
  				sinkByPrice(index, trie);			// Calls the method to sink the AptInfo object if the heap is based on price
  			}
  			else {   //max heap
  				sinkBySqFt(index, trie);			// Calls the method to sink the AptInfo object if the heap is based on sq ft
  			}
  		}
  	}

    /**
	 * Floats a AptInfo object by price
	 * @param parentIndex The index of the apt's parent
	 * @param insertedIndex The index of the current apt
	 * @param trie The trie to update
	 */
	private void floatByPrice(int parentIndex, int insertedIndex, AWayTrie trie) {

		if (parentIndex < 0) {					// If the parent index is less than zero, then the current apt is located at the root
			trie.storeHeapIndex(heap[0].getTrieAddr(), 0);			// Updates the trie for the current apt
			return;
		}

		double parentPrice = heap[parentIndex].getCost();				// Gets the price of the parent apt
		double insertedPrice = heap[insertedIndex].getCost();			// Gets the price of the current apt

		if (parentPrice > insertedPrice) {						// Swaps the parent and current apts if the parent price is greater than the current price
			ApartmentInfo tempApt = heap[parentIndex];
			heap[parentIndex] = heap[insertedIndex];
			heap[insertedIndex] = tempApt;

			trie.storeHeapIndex(heap[insertedIndex].getTrieAddr(), insertedIndex);		// Updates the current apt's index in the trie
			trie.storeHeapIndex(heap[parentIndex].getTrieAddr(), parentIndex);			// Updates the parent apt's index in the trie

			insertedIndex = parentIndex;								// Sets the current apt's index as the parent index
			parentIndex = (int)Math.floor((parentIndex - 1) / 2.0);		// Calculates a new parent index from the current apt's new index

			floatByPrice(parentIndex, insertedIndex, trie);				// Recursively calls the float method
		}
		else {
			trie.storeHeapIndex(heap[insertedIndex].getTrieAddr(), insertedIndex);		// Otherwise updates the current apt's index in the trie
		}
	}


  /**
   * Floats an ApartmentInfo object by sq ft
   * @param parentIndex The index of the apt's parent
   * @param insertedIndex The index of the current apt
   * @param trie The trie to update
   */
    private void floatBySqFt(int parentIndex, int insertedIndex, AWayTrie trie) {

    	if (parentIndex < 0) {
    		trie.storeHeapIndex(heap[0].getTrieAddr(), 0);
    		return;
    	}

    	int parentSqFt = heap[parentIndex].getSqFt();
    	int insertedSqFt = heap[insertedIndex].getSqFt();

    	if (parentSqFt < insertedSqFt) {
    		ApartmentInfo tempApt = heap[parentIndex];
    		heap[parentIndex] = heap[insertedIndex];
    		heap[insertedIndex] = tempApt;

    		trie.storeHeapIndex(heap[insertedIndex].getTrieAddr(), insertedIndex);
    		trie.storeHeapIndex(heap[parentIndex].getTrieAddr(), parentIndex);

    		insertedIndex = parentIndex;
    		parentIndex = (int)Math.floor((parentIndex - 1) / 2.0);

    		floatBySqFt(parentIndex, insertedIndex, trie);
    	}
    	else {
    		trie.storeHeapIndex(heap[insertedIndex].getTrieAddr(), insertedIndex);
    	}
    }

    /**
  	 * Sinks an ApartmentInfo object by its price
  	 * @param parentIndex The parent apt's index
  	 * @param trie The trie to update
  	 */
  	private void sinkByPrice(int parentIndex, AWayTrie trie) {

  		int leftIndex = 2 * parentIndex + 1;		// Calculates the left child's index
  		int rightIndex = 2 * parentIndex + 2;		// Calculates the right child's index
  		int comparedIndex;						// The index to eventually compare to the parent
  		ApartmentInfo leftApt, rightApt;				// ApartmentInfo objects of the left and right children

  		if (leftIndex > heap.length - 1) {		// Sets the left child to null if the index exceeds the heap size
  			leftApt = null;
  		}
  		else {
  			leftApt = heap[leftIndex];			// Otherwise, retrieves the left child AptInfo object
  		}

  		if (rightIndex > heap.length - 1) {		// Sets the right child to null if the index exceeds the heap size
  			rightApt = null;
  		}
  		else {
  			rightApt = heap[rightIndex];		// Otherwise, retrieves the right child AptInfo object
  		}

  		if (leftApt == null && rightApt == null) {			// Conditional if both children are null
  			trie.storeHeapIndex(heap[parentIndex].getTrieAddr(), parentIndex);		// Updates the parent apt's index in the trie
  			return;
  		}
  		else if (leftApt != null && rightApt == null) {		// Conditional if only the right child is null
  			comparedIndex = leftIndex;				// Sets the compared index to the left child index
  		}
  		else if (leftApt == null && rightApt != null) {		// Conditional if only the left child is null (contingency; shouldn't happen normally)
  			comparedIndex = rightIndex;				// Sets the compared index to the right child index
  		}
  		else {							// Otherwise, gets the left and right children's prices
  			double leftPrice = leftApt.getCost();
  			double rightPrice = rightApt.getCost();

  			if (leftPrice <= rightPrice) {			// Sets the compared index to the left child if the left child price is greater than or equal to the right child
  				comparedIndex = leftIndex;
  			}
  			else {
  				comparedIndex = rightIndex;			// Otherwise, sets the compared index to the right child index
  			}
  		}

  		double parentPrice = heap[parentIndex].getCost();			// Gets prices of the parent and child apts
  		double childPrice = heap[comparedIndex].getCost();

  		if (parentPrice > childPrice) {							// Swaps AptInfo objects if the parent price is greater than the child price
  			ApartmentInfo tempApt = heap[comparedIndex];
  			heap[comparedIndex] = heap[parentIndex];
  			heap[parentIndex] = tempApt;

  			trie.storeHeapIndex(heap[comparedIndex].getTrieAddr(), comparedIndex);		// Updates the child apt's index in the trie
  			trie.storeHeapIndex(heap[parentIndex].getTrieAddr(), parentIndex);			// Updates the parent apt's index in the trie

  			sinkByPrice(comparedIndex, trie);						// Recursively calls the sink method
  		}
  		else {
  			trie.storeHeapIndex(heap[parentIndex].getTrieAddr(), parentIndex);		// Otherwise, updates the parent apt's index in the trie
  		}
  	}



  /**
   * Sinks an ApartmentInfo object by its sq ft
   * @param parentIndex The parent apt's index
   * @param trie The trie to update
   */
    private void sinkBySqFt(int parentIndex, AWayTrie trie) {

    	int leftIndex = 2 * parentIndex + 1;
    	int rightIndex = 2 * parentIndex + 2;
    	int comparedIndex;
    	ApartmentInfo leftApt, rightApt;

    	if (leftIndex > heap.length - 1) {
    		leftApt = null;
    	}
    	else {
    		leftApt = heap[leftIndex];
    	}

    	if (rightIndex > heap.length - 1) {
    		rightApt = null;
    	}
    	else {
    		rightApt = heap[rightIndex];
    	}

    	if (leftApt == null && rightApt == null) {
    		trie.storeHeapIndex(heap[parentIndex].getTrieAddr(), parentIndex);
    		return;
    	}
    	else if (leftApt != null && rightApt == null) {
    		comparedIndex = leftIndex;
    	}
    	else if (leftApt == null && rightApt != null) {
    		comparedIndex = rightIndex;
    	}
    	else {
    		int leftSqFt = leftApt.getSqFt();
    		int rightSqFt = rightApt.getSqFt();

    		if (leftSqFt >= rightSqFt) {
    			comparedIndex = leftIndex;
    		}
    		else {
    			comparedIndex = rightIndex;
    		}
    	}

    	int parentSqFt = heap[parentIndex].getSqFt();
    	int childSqFt = heap[comparedIndex].getSqFt();

    	if (parentSqFt < childSqFt) {
    		ApartmentInfo tempApt = heap[comparedIndex];
    		heap[comparedIndex] = heap[parentIndex];
    		heap[parentIndex] = tempApt;

    		trie.storeHeapIndex(heap[comparedIndex].getTrieAddr(), comparedIndex);
    		trie.storeHeapIndex(heap[parentIndex].getTrieAddr(), parentIndex);

    		sinkBySqFt(comparedIndex, trie);
    	}
    	else {
    		trie.storeHeapIndex(heap[parentIndex].getTrieAddr(), parentIndex);
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
  * Checks the ApartmentInfo object at the index specified and determines if it needs to be floated or sunk in the heap
  * @param index The index of the apt
  * @param trie The trie to update the apt's position in the heap
  */
  public void checkAptUpdates(int index, AWayTrie trie) {

  int parentIndex = (int)Math.floor((index - 1) / 2.0);			// Calculates the parent index of the aptInfo object

  if ( getMode() ) {							// Floats or sinks the apt if the priority queue is based on price (min heap)
    floatByPrice(parentIndex, index, trie);
    sinkByPrice(index, trie);
  }
  else {											// Floats or sinks the apt if the priority queue is based on sq ft (max heap)
    floatBySqFt(parentIndex, index, trie);
    sinkBySqFt(index, trie);
  }
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
