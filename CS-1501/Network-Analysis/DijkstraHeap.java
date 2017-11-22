/**
 * @author Zachary M. Mattis
 * CS 1501
 * Assignment 4
 * November 20, 2017
 *
 * This class functions as a non-indexable minimum heap (I.e., a "lazy" heap) for
 * DijkstraNode objects.  Priority is based on lowest value latencies and has the
 * key functions of retrieving the highest priority node, adding a node, and
 * removing the minimum node.
 */

public class DijkstraHeap {

	private DijkstraNode[] heap;					// DijkstraNode objects are stored in an array
	private int cnt;				// Determines where the next entry is added to the heap


	/**
	 * Creates a new non-indexable DijkstraNode heap
	 */
	public DijkstraHeap() {

		heap = new DijkstraNode[32];
		cnt = 0;
	}


	/**
	 * Gets the node of highest priority
	 * @return The node of the highest priority (lowest latency)
	 */
	public DijkstraNode getMin() {

		return heap[0];
	}


	/**
	 * Adds a node to the heap; floats the node if necessary
	 * @param node The DijkstraNode object to add to the heap
	 */
	public void add(DijkstraNode node) {

		if (cnt == heap.length) {			// Resizes the array if there is no room available
			heap = resizeHeap(heap);
		}

		heap[cnt] = node;				// Adds the node to the next available leaf position
		floatNode();							// Floats the node (if necessary) in the heap
		cnt++;							// Increments the index of the next available leaf position
	}


	/**
	 * Removes the minimum node from the heap
	 * @return The minimum node that was removed
	 */
	public DijkstraNode popMin() {

		if (cnt == 0) {
			return null;
		}

		cnt--;							// Decrements the index of the next available leaf position

		DijkstraNode minNode = heap[0];
		heap[0] = heap[cnt];			// Overwrites the node with the highest priority
		heap[cnt] = null;				// Sets the last leaf to null

		sinkNode(0);					// Sinks the node (if necessary)

		return minNode;
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
	private void floatNode() {

		int parentIndex = (int)Math.floor((cnt - 1) / 2.0);			// Calculates the parent index using the leaf index

		floatRecurse(parentIndex, cnt);			// Calls the recursive float function
	}


	/**
	 * Floats a node up the heap
	 * @param parentIndex The index of the DijkstraNode object's parent
	 * @param insertedIndex The index of the current DijkstraNode object
	 */
	private void floatRecurse(int parentIndex, int insertedIndex) {

		if (parentIndex < 0) {					// If the parent index is less than zero, then the current node is located at the root
			return;
		}

		DijkstraNode parentNode = heap[parentIndex];
		DijkstraNode insertedNode = heap[insertedIndex];

		if (parentNode.getLatency() > insertedNode.getLatency()) {			// Swaps the parent and current nodes if the parent's latency is greater than the child's
			DijkstraNode tempNode = heap[parentIndex];
			heap[parentIndex] = heap[insertedIndex];
			heap[insertedIndex] = tempNode;

			insertedIndex = parentIndex;								// Sets the current node's index as the parent index
			parentIndex = (int)Math.floor((parentIndex - 1) / 2.0);		// Calculates a new parent index from the current node's new index

			floatRecurse(parentIndex, insertedIndex);				// Recursively calls the float method
		}
	}


	/**
	 * Sinks a node down the heap
	 * @param parentIndex The parent DijkstraNode object's index
	 */
	private void sinkNode(int parentIndex) {

		int leftIndex = 2 * parentIndex + 1;		// Calculates the left child's index
		int rightIndex = 2 * parentIndex + 2;		// Calculates the right child's index
		int comparedIndex;						// The index to eventually compare to the parent
		DijkstraNode leftNode, rightNode;					// Nodes of the left and right children

		if (leftIndex > heap.length - 1) {		// Sets the left child to null if the index exceeds the heap size
			leftNode = null;
		}
		else {
			leftNode = heap[leftIndex];			// Otherwise, retrieves the left child node
		}

		if (rightIndex > heap.length - 1) {		// Sets the right child to null if the index exceeds the heap size
			rightNode = null;
		}
		else {
			rightNode = heap[rightIndex];		// Otherwise, retrieves the right child node
		}

		if (leftNode == null && rightNode == null) {			// Conditional if both children are null
			return;
		}
		else if (leftNode != null && rightNode == null) {		// Conditional if only the right child is null
			comparedIndex = leftIndex;				// Sets the compared index to the left child index
		}
		else if (leftNode == null && rightNode != null) {		// Conditional if only the left child is null (contingency; shouldn't happen normally)
			comparedIndex = rightIndex;				// Sets the compared index to the right child index
		}
		else if (leftNode.getLatency() <= rightNode.getLatency()) {		// Sets the compared index to the left child if its latency is less than or equal to the right child's
			comparedIndex = leftIndex;
		}
		else {
				comparedIndex = rightIndex;			// Otherwise, sets the compared index to the right child index
		}

		DijkstraNode parentNode = heap[parentIndex];			// Gets the parent and compared child nodes
		DijkstraNode childNode = heap[comparedIndex];

		if (parentNode.getLatency() > childNode.getLatency()) {			// Swaps nodes if the parent's latency is greater than the child's
			DijkstraNode tempNode = heap[comparedIndex];
			heap[comparedIndex] = heap[parentIndex];
			heap[parentIndex] = tempNode;

			sinkNode(comparedIndex);						// Recursively calls the sink method
		}
	}


	/**
	 * Resizes the heap's array its maximum size has been exceeded
	 * @param oldHeap The current array that has not been resized
	 * @return The new array of doubled size
	 */
	private DijkstraNode[] resizeHeap(DijkstraNode[] oldHeap) {

		int doubledSize = oldHeap.length * 2;
		DijkstraNode[] newHeap = new DijkstraNode[doubledSize];		// Creates a new array by doubling the size of the old array

		for (int i = 0; i < oldHeap.length; i++) {			// Copies all objects from the old array to the new array
			newHeap[i] = oldHeap[i];
		}

		return newHeap;
	}
}
