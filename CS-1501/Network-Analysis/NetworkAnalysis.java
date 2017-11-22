/**
 * @author Zachary M. Mattis
 * CS 1501
 * Assignment 4
 * November 17, 2017
 *
 * This class is the main executable that analyzes networks through
 * biconnected graphs. This class used the following alogorithms:
 *    Lazy Prim's    : minimum spanning tree
 *    Dijkstra's     : shortest latency path between vertices
 *    Ford-Fulkerson : maximum data flow
 *
 */

import java.util.*;
import java.lang.*;
import java.io.*;


public class NetworkAnalysis{

  private static LL<NetworkConnection>[] cNetwork;
  private static int NumVertices;
  private static int spanningTreeNum, copperTreeNum;
  private static Scanner input;


  public static void main( String args[] ){

    // Correct Command line
    if(args.length!=1){
      System.out.printf("Usage:\n\tjava NetworkAnalysis network_data.txt");
      System.exit(1);
    }
    //Create Graph
    try{
      createGraph(args[0]);
    }
    catch(IOException e){
      System.out.printf("Error: Invalid Input File Format\nUsage:\n\n\tnum_vertices\n\torg_vert dest_vert material bandwidth length\n\n\t5\n\t0 3 copper 10000 10\n\t2 4 optical 100 8");
      System.exit(1);
    }

    //User Menu Loop
    input = new Scanner(System.in);
    NetworkUI.displayWelcome();
    NetworkUI.displayMenu();
    do{

      System.out.printf("Enter \"M\" to show the menu.\nSelect an option: ");
      String menuOption = input.nextLine();							// Gets input from the user
      System.out.printf("\n");

      switch(menuOption){

        case "1":						// If the input is "1", displays the lowest latency path
					getLowestLatencyPath();
					break;
				case "2":						// If the input is "2", displays the if the graph if copper-only connected
					getCopperConnected();
					break;
				case "3":						// If the input is "3", displays the maximum data transfer between two vertices
          getMaxFlow();
					break;
				case "4":						// If the input is "4", displays the lowest average latency spanning tree of the graph
					getLowestLatencySpanningTree();
					break;
				case "5":						// If the input is "5", displays whether the graph is connected if any two vertices fail
					getFailedConnection();
					break;
				case "q":						// If the input is "q" or "Q", exits the program
				case "Q":
					NetworkUI.displayExit();				// Displays the exit txt
					System.exit(0);
				case "m":
				case "M":						// If the input is "m" or "M", prints the menu
				  NetworkUI.displayMenu();
					break;
				default:								// Otherwise, an invalid command was entered
					System.out.printf("The command entered could not be recognized.  Please enter a valid integer.\n\n");
					break;
      }

    }while(true);


  }


  /**
	 * Creates the graph as an adjancy list by parsing the command line file
	 * @param file The fuke conating graph information
	 */
  @SuppressWarnings("unchecked")
  private static void createGraph(String file) throws IOException{

    FileReader fr = new FileReader(file);
    BufferedReader br = new BufferedReader(fr);                   //file parsing
    NumVertices = Integer.parseInt(br.readLine());              // total num of vertices

    cNetwork = (LL<NetworkConnection>[])new LL[NumVertices];    //Initialize adjency list

    String tokens[];
    int to, from, bw, len;
    while( br.ready() ){
      tokens = br.readLine().split(" ");
      to = Integer.parseInt(tokens[0]);
      from = Integer.parseInt(tokens[1]);
      bw = Integer.parseInt(tokens[3]);
      len = Integer.parseInt(tokens[4]);

      NetworkConnection c = new NetworkConnection(to, from, len, bw, tokens[2].equals("copper") );
      NetworkConnection r = new NetworkConnection(from, to, len, bw, tokens[2].equals("copper") );

      addEdge( to, c );
      addEdge( from, r );
    }

    br.close();
  }

  /**
	 * Adds an network connection edge to the adjacency list
	 * @param vertex The index to place the NetworkConnection object in the array
	 * @param edge The NetworkConnection object
	 */
  	private static void addEdge(int vertex, NetworkConnection edge) {

  		if (cNetwork[vertex] == null) {				// Creates a new linked list if one does not exist in the array
  			cNetwork[vertex] = new LL<NetworkConnection>();
  		}

  		cNetwork[vertex].add(edge);				// Adds the NetworkConnection object to the vertex's linked list
  	}



    /**
  	 * Builds a shortest path tree of the entered starting and destination vertices by latency
  	 */
  	private static void getLowestLatencyPath() {

      NetworkUI.displayLowestLatencyPath();   //UI

  		int to=-1, from=-1;         // User selected
      boolean valid;

      //Starting Vertex
      do{
        valid = true;
        System.out.printf("Enter the starting vertex: ");
        try{
          from = Integer.parseInt(input.nextLine());
        }
        catch(NumberFormatException nfe){
          System.out.print("Error invalid vertex: please enter a valid integer.\n");
          valid=false;
        }

        if ( (from < 0 || from > (NumVertices-1) ) && valid ){
          System.out.printf("Error: entered value does not lie within range of vertices\n");
          valid=false;
        }

      }while(!valid);

      //Destination Vertex
      do{
        valid=true;
  		  System.out.printf("Enter the ending vertex: ");
        try{
          to = Integer.parseInt(input.nextLine());
        }
        catch(NumberFormatException nfe){
          System.out.print("Error invalid vertex: please enter a valid integer.\n");
          valid=false;
        }

        if ( (to < 0 || to > (NumVertices-1) ) && valid){
          System.out.printf("Error: entered value does not lie within range of vertices\n");
          valid=false;
        }

      }while(!valid);


			// Gets the shortest path based on distance
			DijkstraNode[] spt = new DijkstraNode[NumVertices];				// Declares a new shortest path tree of tentative distances
			spt = dijkstra(to, from, spt);			// Builds the shortest path tree
			int curr = from;
			double latency = 0.0;                      //latency
      double bw =  Double.POSITIVE_INFINITY;					//  bandwidth

			if (spt == null) {						// If the shortest path tree is null, the graph is disconnected
				System.out.printf("No path exists from %d to %d.  The graph is disconnected.\n\n", from, to);
				return;
			}

			while (curr != to) {			// Iterates through the tree until the full path is displayed
				DijkstraNode djNode = spt[curr];
				int prev = djNode.getPrevious();

				if (prev < 0) {								// Contingency if a previous value is less than 0
          System.out.printf("No path exists from %d to %d.  The graph is disconnected.\n\n", from, to);
					return;
				}

				double tentLat = djNode.getLatency();
				double prevTentLat = spt[prev].getLatency();

				latency += (tentLat - prevTentLat);			// Gets the true edge distance rather than the tentative distance
				System.out.printf("\n%d --> %d: %.4f ms", curr, prev,	(tentLat - prevTentLat)*1000000 );

				curr = prev;
        if (djNode.getMinBandwidth() < bw ){
          bw = djNode.getMinBandwidth();
        }

			}

			System.out.printf("\n\n\tTotal Latency: %f ms", latency*1000000);
      System.out.printf("\n\tAvailable Bandwidth: %.0f Mbps\n\n", bw);

    }

  /**
	 * Builds a shortest path tree between vertices using Dijkstra's algorithm
	 * @param endVert  ending vertex of path
   * @param startVertex   starting vertex of path
	 * @param tentative The array of unfilled tentative latencies
	 * @return The array of filled tentative latencies
	 */
	private static DijkstraNode[] dijkstra(int endIndex, int startIndex, DijkstraNode[] tentative) {
    DijkstraHeap heap;						// Heap for selecting the minimum unvisited vertices
		boolean[] visited = new boolean[NumVertices];			// Tracks unvisited vertices
		int currentIndex = endIndex;

		for (int i = 0; i < NumVertices; i++) {			// Initializes all vertices as having a maximum latency and unvisited
			tentative[i] = new DijkstraNode(i);
			visited[i] = false;
		}

		tentative[endIndex].setLatency(0);			// Sets the starting vertex with a distance of 0

		while (currentIndex != startIndex) {			// Builds tentative latency for all vertices
			LLNode<NetworkConnection> node = cNetwork[currentIndex].getListHead();
			heap = new DijkstraHeap();					// Clear the heap for updated entries

			while (node.hasData()) {					// Gets all neighbors of the unvisited vertex
				NetworkConnection edge = node.getData();
				int destVertex = edge.getDestination();			// Gets the destination of the vertex's edge
        double bw = edge.getBandwidth();      //Gets the available bw

				if (!visited[destVertex]) {			// Calculates the tentative latency if the destination is unvisited
					double tentativeLat = edge.getLatency() + tentative[currentIndex].getLatency();

          if( tentativeLat < tentative[destVertex].getLatency() ){    // Updates the tentative distance if it is less than the current distance
            tentative[destVertex].setLatency(tentativeLat);
            tentative[destVertex].setPrevious(currentIndex);      // Updates where the previous vertex is (along the path)
            tentative[destVertex].setMinBandwidth(bw);
          }

				}

				node = node.getNextNode();
			}
			visited[currentIndex] = true;				// Marks the current vertex as visited

			for (int i = 0; i < NumVertices; i++) {				// Fills the heap with unvisited edges
				if (!visited[i]) {
					heap.add(tentative[i]);
				}
			}

			if (heap.isEmpty()) {			// If the heap is empty before the path is completed, returns null
				return null;
			}

			DijkstraNode djNode = heap.popMin();				// Gets the highest priority unvisited edge
			int nextIndex = djNode.getIndex();

			currentIndex = nextIndex;					// Sets the next vertex to do work on
		}

		return tentative;
  }

  /**
   * Determines if the graph is fully connected using copper-only wires
   */
  private static void getCopperConnected(){

    NetworkUI.displayCopperConnected();
    NetworkConnection[] copperSpanTree = new NetworkConnection[NumVertices];   //Min spanning tree
    NetworkHeap minPQ = new NetworkHeap();		// Creates a minimum priority queue for edge latencies
    boolean[] visitedIndices = new boolean[NumVertices];			// Tracks which vertices have been visited
    copperTreeNum=0;


    copperSpanTree = lazyPrim(copperSpanTree, minPQ, visitedIndices, 0, true);
    if (copperTreeNum < NumVertices-1 ){
      System.out.printf("The graph IS NOT fully connected using only copper wire.\n\n");
    }
    else{
      System.out.printf("The graph IS fully connected using only copper wire.\n\n");
    }


  }

  /**
	 * Retrieves and prints the lowest latency spanning tree; creates one if no such tree exists
	 */
	private static void getLowestLatencySpanningTree() {

    NetworkUI.displayLowestSpanningTree();    //UI
		NetworkConnection[] minSpanTree = new NetworkConnection[NumVertices];   //Min spanning tree
    NetworkHeap minPQ = new NetworkHeap();		// Creates a minimum priority queue for edge latencies
    boolean[] visitedIndices = new boolean[NumVertices];			// Tracks which vertices have been visited
    spanningTreeNum=0;

    for (int i = 0; i < NumVertices; i++) {				// Sets all visited vertices to false
			visitedIndices[i] = false;
		}

    for (int v=0; v<NumVertices; v++){
      if(!visitedIndices[v]){
        minSpanTree = lazyPrim(minSpanTree, minPQ, visitedIndices, v, false);			// Create a minimum spanning tree if none exists
      }

    }

		for (int i = 0; i < spanningTreeNum; i++) {		// Iterates through the array containing the graph edges
			NetworkConnection edge = minSpanTree[i];

			System.out.printf( "%d <--> %d : Latency = %.4f ms\n", edge.getOrigin(), edge.getDestination(), (edge.getLatency()*1000000) );
		}
		if ( spanningTreeNum<(NumVertices-1) ) {							// Displays a message if the graph is disconnected
			System.out.printf("A gap exists in the minimum spanning tree.  The graph is disconnected.\n");
		}
		System.out.printf("\n");
	}

  /**
	 * Builds the minimum spanning tree from the current graph data using lazy Prim's algorithm
   * @param minSpanTree   minimum spanning tree
   * @param minPQ   heap storing the minimum edge latencies
   * @param visitedIndices  boolean array storing which indices have been visited
   * @param vertex  starting vertex for the spanning tree
   * @param mode 0 = standard; 1 = copper only
   * @return spanning tree of the graph
	 */
	private static NetworkConnection[] lazyPrim(NetworkConnection[] minSpanTree, NetworkHeap minPQ, boolean[] visitedIndices, int vertex, boolean mode) {

    if(mode){
      spanningTreeNum=0;
    }
    visitedIndices[vertex] = true;					// Sets the vertex as visited
		LLNode<NetworkConnection> node = cNetwork[vertex].getListHead();

		while (node.hasData()) {					// Adds all neighbors of the vertex to the priority queue
			NetworkConnection edge = node.getData();
			if (mode && edge.isCopper() ) {
        minPQ.add(edge);
      }
      else if (!mode){
        minPQ.add(edge);
      }

      //System.out.print("adding " + edge.getOrigin() + " " + edge.getDestination() + " to pq\n");
      node = node.getNextNode();
		}

		while (!minPQ.isEmpty()) {				// Loops while there exists something in pq

			NetworkConnection minEdge = minPQ.popMin();					// Gets the minimum edge from the priority queue
      int destVertex = minEdge.getDestination();				// Gets the destination from the edge

			if (!visitedIndices[destVertex]) {			// Conditional if the destination is unvisited
				minSpanTree[spanningTreeNum] = minEdge;				// Adds the edge to the minimum spanning tree
        spanningTreeNum++;                         //increment num visited
        copperTreeNum++;
        visitedIndices[destVertex] = true;				// Sets the vertex as visited

				node = cNetwork[destVertex].getListHead();

				while (node.hasData()) {					// Adds all neighbors of the newly visited vertex to the priority queue
					NetworkConnection edge = node.getData();
          if (mode && edge.isCopper() ) {
            minPQ.add(edge);
          }
          else if (!mode){
            minPQ.add(edge);
          }

          //System.out.print("adding " + edge.getOrigin() + " " + edge.getDestination() + " to pq\n");
					node = node.getNextNode();
				}
			}
		}

		return minSpanTree;

  }

  /**
   * Retrieves and prints the maximum amount of data transfer between vertices (max flow)
   */
  private static void getMaxFlow(){

    NetworkUI.displayMaxFlow();
    int to = -1, from = -1;
    boolean valid;

    //Starting Vertex
    do{
      valid = true;
      System.out.printf("Enter the starting vertex: ");
      try{
        from = Integer.parseInt(input.nextLine());
      }
      catch(NumberFormatException nfe){
        System.out.print("Error invalid vertex: please enter a valid integer.\n");
        valid=false;
      }

      if ( (from < 0 || from > (NumVertices-1) ) && valid ){
        System.out.printf("Error: entered value does not lie within range of vertices\n");
        valid=false;
      }

    }while(!valid);

    //Destination Vertex
    do{
      valid=true;
      System.out.printf("Enter the ending vertex: ");
      try{
        to = Integer.parseInt(input.nextLine());
      }
      catch(NumberFormatException nfe){
        System.out.print("Error invalid vertex: please enter a valid integer.\n");
        valid=false;
      }

      if ( (to < 0 || to > (NumVertices-1) ) && valid){
        System.out.printf("Error: entered value does not lie within range of vertices\n");
        valid=false;
      }

    }while(!valid);

    int flow = fordFulkerson(from, to);

    System.out.printf("\nThe maximum bandwidth from %d to %d is %d Mb/s\n\n", from, to, flow);


  }

  private static int fordFulkerson(int src, int sink){

    int parent[] = new int[NumVertices];
    int graph[][] = parseResidualGraph();
    int max_flow = 0;
    int u,v;

    while( augmentingPath(graph, src, sink, parent) ){ //While there is an augmenting path from source to sink

      int path_flow = Integer.MAX_VALUE;
      //Find max flow in path
      for(v=sink; v!=src; v=parent[v]){
        u=parent[v];
        path_flow = Math.min(path_flow, graph[u][v]);
      }

      //Update flow along path
      for (v=sink; v!=src; v=parent[v]){
        u = parent[v];
        graph[u][v]-=path_flow;
        graph[u][v]-=path_flow;
      }

      max_flow+=path_flow;
    }

    return max_flow;
  }

  /**
   * Parses the NetworkConnection edges into a 2-D Residual bandwidth array
   */
  private static int[][] parseResidualGraph(){
    int graph[][] = new int[NumVertices][NumVertices];
    NetworkConnection edge;
    for (int v=0; v<NumVertices; v++){
      LLNode<NetworkConnection> node = cNetwork[v].getListHead();
      while(node.hasData()){
        edge = node.getData();
        graph[edge.getOrigin()][edge.getDestination()] = edge.getBandwidth();
        node=node.getNextNode();
      }
    }

  return graph;
  }

  /**
  * Determines if there is an augmenting path from source to sink based on residual bandwidth
  * @param graph residual bandwidth graph (represented as 2-D array)
  * @param src source vertex
  * @param sink sink vertex
  * @param parent array with route of residual bw
  * @return status of augmenting path
  */
  private static boolean augmentingPath(int[][] graph, int src, int sink, int parent[])
  {
      // Create a visited array and mark all vertices as not
      // visited
      boolean visitedIndices[] = new boolean[NumVertices];
      for(int i=0; i<NumVertices; ++i){
          visitedIndices[i]=false;
      }

      LinkedList<Integer> q = new LinkedList<Integer>();
      q.add(src);
      visitedIndices[src]=true;
      parent[src]=-1;

      while(q.size()!=0){
        int temp = q.poll();
        for(int v=0; v<NumVertices; v++){

          if (!visitedIndices[v] && graph[temp][v]>0){
            q.add(v);
            parent[v]=temp;
            visitedIndices[v]=true;
          }
        }

      }

      return visitedIndices[sink];

    }


  /**
   * Displays the connection status of the graph if ANY 2 vertices were to fail
   */
  private static void getFailedConnection(){
    NetworkUI.displayVertexFailure();
    boolean flag=true;
    for (int i = 0; i<NumVertices-1; i++){
      for(int j = i+1; j<NumVertices; j++){
        if ( removeVertices(i,j) ){
          System.out.printf("Graph disconnected when vertices %d & %d fail\n\n", i, j);
          flag=false;
        }

      }
    }

      if(flag){
        System.out.printf("The graph will remain connected if ANY two vertices fail.\n\n");
      }



  }

  /**
   * Creates a spanning tree by removing indices i & j
   * @param i first index to remove
   * @param j second index to remove
   * @return graph connection status : true if connected vertices = total-2
   */
  private static boolean removeVertices(int i, int j){
    NetworkHeap minPQ = new NetworkHeap();
    boolean[] visitedIndices = new boolean[NumVertices];
    for (int v=0;v<NumVertices;v++){
      visitedIndices[v]=false;
    }

    int cutNum=0;
    int startVertex;
    if(i!=0 && j!=0){
      startVertex = 0;
    }
    else if(i!=1 && j!=1){
      startVertex = 1;
    }
    else{
      startVertex = 2;
    }

		LLNode<NetworkConnection> node = cNetwork[startVertex].getListHead();

		while (node.hasData()) {					// Adds all neighbors of the vertex to the priority queue
			NetworkConnection edge = node.getData();
      if (edge.getOrigin()!= i && edge.getOrigin()!= j && edge.getDestination()!=i && edge.getDestination()!=j ){
        minPQ.add(edge);
      }
      node = node.getNextNode();
		}

		while (!minPQ.isEmpty()) {				// Loops while there exists something in pq

			NetworkConnection minEdge = minPQ.popMin();					// Gets the minimum edge from the priority queue
			int destVertex = minEdge.getDestination();				// Gets the destination from the edge

			if (!visitedIndices[destVertex]) {			// Conditional if the destination is unvisited
        cutNum++;                         //increment num visited
        visitedIndices[destVertex] = true;				// Sets the vertex as visited
				node = cNetwork[destVertex].getListHead();

				while (node.hasData()) {					// Adds all neighbors of the newly visited vertex to the priority queue
					NetworkConnection edge = node.getData();
          if (edge.getOrigin()!= i && edge.getOrigin()!= j && edge.getDestination()!=i && edge.getDestination()!=j ){
            minPQ.add(edge);
          }
					node = node.getNextNode();
				}
			}

		}
		return ( cutNum<(NumVertices-2) );
  }

}
