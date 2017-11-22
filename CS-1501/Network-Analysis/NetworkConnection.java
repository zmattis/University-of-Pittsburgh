/**
 * @author Zachary M. Mattis
 * CS 1501
 * Assignment 4
 * November 18, 2017
 *
 * This class stores information about an computer network connection as
 * an edge of a graph. It contains information about its origin and destination
 *  vertices, the length of the cable (as an integer meters), the bandwidth of
 * the connection (as a integer Megabits / second), and whether the connection
 * is an copper or optical wire. Edges are assumed to be bidirectional
 * and that the graph is undirected.
 */

public class NetworkConnection implements Comparable<NetworkConnection> {

	private int from, to;			// Vertices of the graph
	private int length;					// Length of the edge
  private int bandwidth;      // Bandwidth of the connection
	private double latency;			// latency of the connection
	private boolean copper;				// Boolean: true = copper, false = optical

	private static final int COPPER_SPEED = 230000000;
	private static final int OPTICAL_SPEED = 200000000;



	/**
	 * Creates a new computer network graph edge
	 * @param org The origin vertex
	 * @param dest The destination vertex
	 * @param len The length between vertices
	 * @param bw The bandwitdth of the connection
	 * @param cop Whether the edge is copper or optical
	 */
	public NetworkConnection(int org, int dest, int len, int bw, boolean cop) {

		setOrigin(org);
		setDestination(dest);
		setLength(len);
		setBandwidth(bw);
		setCopper(cop);

		if (cop){
			latency = ((double)len / (double)COPPER_SPEED);
		}
		else{
			latency = ((double)len / (double)OPTICAL_SPEED) ;
		}
	}


	/**
	 * Gets the origin vertex
	 * @return The origin vertex
	 */
	public int getOrigin() {

		return from;
	}


	/**
	 * Gets the destination vertex
	 * @return The destination vertex
	 */
	public int getDestination() {

		return to;
	}


	/**
	 * Gets the length between the origin and destination vertices
	 * @return The length between vertices
	 */
	public int getLength() {

		return length;
	}


	/**
	 * Gets the bandwidth of the connection between two vertices
	 * @return The bandwidth between vertices
	 */
	public int getBandwidth() {

		return bandwidth;
	}

	/**
	 * Gets whether the edge is reversed or not (in relation to the input file)
	 * @return A boolean representing reversal
	 */
	public boolean isCopper() {

		return copper;
	}

	/**
	 * Gets the latency of the connection between two vertices
	 * @return The latency between vertices
	 */
	public double getLatency() {

		return latency;
	}


	/**
	 * Compares the origin and destination vertices between two edges without regard to other
	 * fields.  If they are the same vertices, returns 0.  Otherwise, returns 1.
	 */
	public int compareTo(NetworkConnection cNetwork) {

		if (cNetwork == null) {
			throw new NullPointerException("The computer network being compared to is null.\n");
		}

		if ( this.getOrigin() == cNetwork.getOrigin() && this.getDestination() == cNetwork.getDestination()) {
			return 0;				// Returns 0 if the origin and destination vertices match
		}
		else {
			return 1;			// Otherwise, returns 1
		}
	}


	/**
	 * Sets the origin vertex
	 * @param org The origin vertex
	 */
	private void setOrigin(int org) {

		from = org;
	}


	/**
	 * Sets the destination vertex
	 * @param dest The destination vertex
	 */
	private void setDestination(int dest) {

		to = dest;
	}


	/**
	 * Sets the distance of the edge between vertices
	 * @param len The edge length
	 */
	private void setLength(int len) {

		length = len;
	}


	/**
	 * Sets the bandwidth of the connection
	 * @param bw The bandwidth of the connection
	 */
	private void setBandwidth(int bw) {

		bandwidth = bw;
	}

	/**
	 * Sets if connection is copper
   * @param cop A boolean denoting if the netowrk is copper or optical
	 */
	private void setCopper(boolean cop) {

		copper = cop;
	}

}
