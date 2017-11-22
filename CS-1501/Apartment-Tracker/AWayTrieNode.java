/**
 * @author Zachary M. Mattis
 * CS 1501
 * Assignment 3
 * November 6, 2017
 *
 * This class functions as a node for each level of the AWayTrie data
 * structure.  Nodes are simply arrays with an extra int variable to
 * store index values for the heap.  Methods of this class access
 * and modify the heap index as well as visit other nodes within the
 * data structure.
 */

public class AWayTrieNode {

	int heapIndex;					// Stores the heap index
	AWayTrieNode[] trieNode;			// An array where other nodes are stored


	/**
	 * Creates a new node by setting its heap index to -1 and initializing the array
	 */
	public AWayTrieNode() {

		trieNode = new AWayTrieNode[34];		// 34 is the alphabet size of the st addr
		setHeapIndex(-1);
	}


	/**
	 * Sets the heap index of a node in the trie
	 * @param index The index of the AptInfo object in its heap
	 */
	public void setHeapIndex(int index) {

		heapIndex = index;
	}


	/**
	 * Gets the heap index of a node in the trie
	 * @return The index of a AptInfo object in its heap
	 */
	public int getHeapIndex() {

		return heapIndex;
	}


	/**
	 * Visits the index of an array of a node; enables traversal through a trie
	 * @param index The index of the array within the node
	 * @return A node of the next level in the trie
	 */
	public AWayTrieNode visitTrieIndex(int index) {

		return trieNode[index];
	}


	/**
	 * Creates a new node in the trie
	 * @param index The index of the array within the node
	 */
	public void setTrieNode(int index) {

		trieNode[index] = new AWayTrieNode();
	}
}
