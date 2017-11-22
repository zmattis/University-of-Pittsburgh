/**
 * @author Brandon S. Hang
 * @version 1.100
 * CS 1501
 * Assignment 4
 * April 7, 2016
 *
 * This class functions as a simple data structure to use for Dijkstra's
 * algorithm of finding the shortest weighted path in a graph.  It holds
 * indices representing the node's place in a tentative latency array
 * and where the node was traversed from previously.  It also holds the
 * tentative latency calculated thus far in the algorithm.
 *
 */

public class DijkstraNode {

	private int index, previous;			// Indices used in the tentative latency array
	private double latency, bandwidth;				// The tentative latency


	/**
	 * Creates a new node with the tentative latency set to the maximum value of a int
	 * @param idx The node's index in the tentative latency array
	 */
	public DijkstraNode(int idx) {

		setIndex(idx);
		setLatency(Double.POSITIVE_INFINITY);
		setMinBandwidth(Double.POSITIVE_INFINITY);
		setPrevious(-1);				// Initializes the previous value to -1 (I.e., not having a previous index)
	}


	/**
	 * Sets the tentative latency of the node
	 * @param lt The tentative latency
	 */
	public void setLatency(double lt) {

		latency = lt;
	}

	/**
	 * Sets the minimum bandwidth of the node
	 * @param bw The minimum bandwidth
	 */
	public void setMinBandwidth(double bw) {

		bandwidth = bw;
	}


	/**
	 * Sets where the node was previously linked to as a path
	 * @param prev The previous index
	 */
	public void setPrevious(int prev) {

		previous = prev;
	}


	/**
	 * Sets the index of the node in the tentative latency array
	 * @param idx The array index
	 */
	private void setIndex(int idx) {

		index = idx;
	}


	/**
	 * Gets the tentative latency of the node
	 * @return The tentative latency
	 */
	public double getLatency() {

		return latency;
	}

	/**
	 * Gets the minimum bandwidth of the node
	 * @return The miimum bandwidth
	 */
	public double getMinBandwidth() {

		return bandwidth;
	}

	/**
	 * Gets the previous index where the node was linked to
	 * @return The previous index
	 */
	public int getPrevious() {

		return previous;
	}


	/**
	 * Gets the index of the node in the tentative latency array
	 * @return The array index
	 */
	public int getIndex() {

		return index;
	}
}
