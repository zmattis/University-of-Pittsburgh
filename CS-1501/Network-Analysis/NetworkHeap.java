/**
 * @author Zachary M. Mattis
 * CS 1501
 * Assignment 4
 * November 18, 2017
 *
 * This class functions as a non-indexable minimum heap (I.e., a "lazy" heap) for
 * NetworkConnection objects.  Priority is based on lowest value latency and has the
 * key functions of retrieving the highest priority latency, adding an edge, and
 * removing the minimum edge.
 */

public class NetworkHeap {

	private NetworkConnection[] heap;					// NetworkConnection objects are stored in an array
	private int cnt;				// Determines where the next entry is added to the heap


	/**
	 * Creates a new non-indexable NetworkConnection heap
	 */
	public NetworkHeap() {

		heap = new NetworkConnection[32];
		cnt = 0;
	}


	/**
	 * Gets the edge of highest priority
	 * @return The edge with the highest priority (lowest latency)
	 */
	public NetworkConnection getMin() {

		return heap[0];
	}


	/**
	 * Adds an edge to the heap; floats the edge if necessary
	 * @param edge The NetworkConnection object to add to the heap
	 */
	public void add(NetworkConnection edge) {

		if (cnt == heap.length) {			// Resizes the array if there is no room available
			heap = resizeHeap(heap);
		}

		heap[cnt] = edge;				// Adds the edge to the next available leaf position
		floatEdge();							// Floats the edge (if necessary) in the heap
		cnt++;							// Increments the index of the next available leaf position
	}


	/**
	 * Removes the minimum edge from the heap
	 * @return The minimum edge that was removed
	 */
	public NetworkConnection popMin() {

		if (cnt == 0) {
			return null;
		}

		cnt--;							// Decrements the index of the next available leaf position

		NetworkConnection minLatency = heap[0];
		heap[0] = heap[cnt];			// Overwrites the edge with the highest priority
		heap[cnt] = null;				// Sets the last leaf to null

		sinkEdge(0);					// Sinks the edge (if necessary)

		return minLatency;
	}


	/**
	 * Checks to see if the heap is empty
	 * @return A boolean representing the state of the heap
	 */
	public boolean isEmpty() {

		return heap[0] == null;
	}


	/**
	 * Calculates the parentIndex before calling the recursive float function
	 */
	private void floatEdge() {

		int parentIndex = (int)Math.floor((cnt - 1) / 2.0);			// Calculates the parent index using the leaf index

		floatRecurse(parentIndex, cnt);			// Calls the recursive float function
	}


	/**
	 * Floats an edge up the heap
	 * @param parentIndex The index of the NetworkConnection object's parent
	 * @param insertedIndex The index of the current NetworkConnection object
	 */
	private void floatRecurse(int parentIndex, int insertedIndex) {

		if (parentIndex < 0) {					// If the parent index is less than zero, then the current edge is located at the root
			return;
		}

		NetworkConnection parentEdge = heap[parentIndex];
		NetworkConnection insertedEdge = heap[insertedIndex];

		if ( parentEdge.getLatency() > insertedEdge.getLatency() ) {			// Swaps the parent and current edges if the parent's distance is greater than the child's
			NetworkConnection tempEdge = heap[parentIndex];
			heap[parentIndex] = heap[insertedIndex];
			heap[insertedIndex] = tempEdge;

			insertedIndex = parentIndex;								// Sets the current edge's index as the parent index
			parentIndex = (int)Math.floor((parentIndex - 1) / 2.0);		// Calculates a new parent index from the current edge's new index

			floatRecurse(parentIndex, insertedIndex);				// Recursively calls the float method
		}
	}


	/**
	 * Sinks an edge down the heap
	 * @param parentIndex The parent NetworkConnection object's index
	 */
	private void sinkEdge(int parentIndex) {

		int leftIndex = 2 * parentIndex + 1;		// Calculates the left child's index
		int rightIndex = 2 * parentIndex + 2;		// Calculates the right child's index
		int comparedIndex;						// The index to eventually compare to the parent
		NetworkConnection leftEdge, rightEdge;					// Edges of the left and right children

		if (leftIndex > heap.length - 1) {		// Sets the left child to null if the index exceeds the heap size
			leftEdge = null;
		}
		else {
			leftEdge = heap[leftIndex];			// Otherwise, retrieves the left child edge
		}

		if (rightIndex > heap.length - 1) {		// Sets the right child to null if the index exceeds the heap size
			rightEdge = null;
		}
		else {
			rightEdge = heap[rightIndex];		// Otherwise, retrieves the right child edge
		}

		if (leftEdge == null && rightEdge == null) {			// Conditional if both children are null
			return;
		}
		else if (leftEdge != null && rightEdge == null) {		// Conditional if only the right child is null
			comparedIndex = leftIndex;				// Sets the compared index to the left child index
		}
		else if (leftEdge == null && rightEdge != null) {		// Conditional if only the left child is null (contingency; shouldn't happen normally)
			comparedIndex = rightIndex;				// Sets the compared index to the right child index
		}
		else if (leftEdge.getLatency() <= rightEdge.getLatency()) {		// Sets the compared index to the left child if its latency is less than or equal to the right child's
			comparedIndex = leftIndex;
		}
		else {
				comparedIndex = rightIndex;			// Otherwise, sets the compared index to the right child index
		}

		NetworkConnection parentEdge = heap[parentIndex];			// Gets the parent and compared child edges
		NetworkConnection childEdge = heap[comparedIndex];

		if (parentEdge.getLatency() > childEdge.getLatency()) {			// Swaps edges if the parent's latency is greater than the child's
			NetworkConnection tempEdge = heap[comparedIndex];
			heap[comparedIndex] = heap[parentIndex];
			heap[parentIndex] = tempEdge;

			sinkEdge(comparedIndex);						// Recursively calls the sink method
		}
	}


	/**
	 * Resizes the heap's array its maximum size has been exceeded
	 * @param oldHeap The current array that has not been resized
	 * @return The new array of doubled size
	 */
	private NetworkConnection[] resizeHeap(NetworkConnection[] oldHeap) {

		int doubledSize = oldHeap.length * 2;
		NetworkConnection[] newHeap = new NetworkConnection[doubledSize];		// Creates a new array by doubling the size of the old array

		for (int i = 0; i < oldHeap.length; i++) {			// Copies all objects from the old array to the new array
			newHeap[i] = oldHeap[i];
		}

		return newHeap;
	}
}
