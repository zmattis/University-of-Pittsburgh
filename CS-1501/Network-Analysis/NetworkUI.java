/**
 * @author Zachary M. Mattis
 * CS 1501
 * Assignment 4
 * November 17, 2017
 *
 * This class functions as the UI for the NetworkAnalysis.java executable.
 * Methods of this class simply print out different messages to the
 * console screen.
 */

 public class NetworkUI{

   /**
	 * Prints the welcome message
	 */
	public static void displayWelcome() {

		System.out.printf("\n"
				+ "================================================\n"
				+ "   Welcome to the Zeta Network Analysis App!\n"
				+ "================================================\n\n");
	}


  /**
   * Prints the main menu
   */
  public static void displayMenu() {

  	System.out.printf("\t\t-------- MAIN MENU --------\n"
  			+ "Please select an option by entering the number next to the option name.\n"
  			+ "\t1. Find the lowest latency path between two vertices\n"
  			+ "\t2. Is the network copper-only?\n"
  			+ "\t3. Find the maximum amount of data transferred between vertices\n"
  			+ "\t4. Find the lowest average latency spanning tree\n"
  			+ "\t5. Can any two pairs of vertices be removed?\n"
  			+ "\tQ. Exit\n");
  }


  /**
	 * Prints the lowest latency spanning tree
	 */
	public static void displayLowestSpanningTree() {

		System.out.printf("-------- LOWEST LATENCY SPANNING TREE --------\n");
	}

  /**
	 * Prints the lowest latency spanning tree
	 */
	public static void displayCopperConnected() {

		System.out.printf("-------- COPPER CONNECTED GRAPH --------\n");
	}

  /**
   * Prints the lowest latency spanning tree
   */
  public static void displayMaxFlow() {

    System.out.printf("-------- MAX DATA TRANSFER --------\n");
  }

  /**
	 * Prints the lowest latency path between vertices
	 */
	public static void displayLowestLatencyPath() {

		System.out.printf("-------- LOWEST LATENCY PATH BETWEEN VERTICES --------\n");
	}

  /**
	 * Prints the 2 vertex failure
	 */
	public static void displayVertexFailure() {

		System.out.printf("-------- VERTEX FAILURE GRAPH CONNECTION --------\n");
	}

  /**
	 * Prints the exit message
	 */
	public static void displayExit() {

		System.out.printf("\nThank you for using the Zeta Network Analysis App!\n"
				+ "Good-bye!\n");
	}




 }
