/**
 * @author Zack Mattis
 * CS 1501
 * Assignment 1
 * September 25, 2017
 * 
 * This class contains methods that supplement a DLBTrie object that specifically uses
 * the generic parameter <Character, String>.  These methods are used to add strings to
 * this type of DLBTrie and print end-of-path values as well.
 */

public class DLBMethods {
	
	public static void addWord(DLBTrie<Character, String> trie, String word) {
		
		DLBNode<Character, String> currentNode, rootNode;			// De la Briandais trie node objects
		
		rootNode = trie.getRootNode();				// Gets the first key of the trie
		currentNode = rootNode;						// Sets the current node to the root
		
		for (int i = 0; i < word.length(); i++) {			// Loops through the trie adding a single character at a time
			currentNode = addCharacter(word.charAt(i), currentNode);
		}
		
		addSentinelNode(word, currentNode);				// Adds the sentinel node and sets its value (end-of-path)
	}
	
	
	/**
	 * Attempts to add a character as a key to the de la Briandais trie.  If the key already exists, it will move
	 * on to the next linked list in the trie.  If not, it will add it to the end of the current linked list.
	 * @param ch The character to add
	 * @param node The node to compare keys
	 * @return A node signifying the next location in the trie
	 */
	private static DLBNode<Character, String> addCharacter(Character ch, DLBNode<Character, String> node) {
		
		if (node.hasNoKey()) {			// If the node does not have a key, it will be set to the character
			node.setKey(ch);
			node.setNextChild(new DLBNode<Character, String>());
			node = node.getNextChild();			// Will return the node of the next linked list
		}
		else if (ch.equals(node.getKey())) {		// Conditional if the node's key is the same as the character
			node = node.getNextChild();			// Will return the node of the next linked list
		}
		else if (node.hasNextSibling()) {			// Conditional if the keys do not match and the current node has an adjacent node
			node = node.getNextSibling();
			node = addCharacter(ch, node);		// Recursively calls the method to attempt adding the key to the next node
		}
		else {					// Otherwise, creates a new adjacent node in the list and calls the method to add it to the new node
			node.setNextSibling(new DLBNode<Character, String>());
			node = node.getNextSibling();
			node = addCharacter(ch, node);
		}
		
		return node;			// Returns the next node to search from
	}
	
	
	/**
	 * This method sets the sentinel value of the trie, i.e., a way to mark the end of a path in the trie.  It also
	 * sets the value of the end-of-path node to the string passed in the parameter.  It functions similarly to
	 * addCharacter except that the character is fixed and also adds a string.
	 * @param word The string to add to the end-of-path node
	 * @param node The end-of-path node
	 * @return A node signifying the next location in the trie
	 */
	private static void addSentinelNode(String word, DLBNode<Character, String> node) {
		
		Character sentinelChar = '*';			// For this project, the sentinel value is '*', an unused character
		
		node.setKey(sentinelChar);			// Sets the key to '*'
		node.setValue(word);				// Sets the value to the string parameter
		node.setCount(node.getCount()+1);
	}
	
	
	/**
	 * Determines if the a node is terminal node, i.e. that its key is '*'
	 * @param node The node to check
	 * @return Returns true if the key is '*'
	 */
	public static boolean isTerminalNode(DLBNode<Character, String> node) {
		
		return (node.getKey().equals('*'));
	}
	
	
	/**
	 * This method adds words to the prediction string array using prefixes by being passed a given node parameter.
	 * @param node The node that all values will start with prefix 
	 * @param num The number of strings to print
	 * @param predictions String array containing autocomplete predictions
	 */
	public static int findPredictions(DLBNode<Character, String> node, int num, String[] predictions) {
		
		if (!node.hasNoKey() && num > 0) {		// If the node has a key and there are still strings to print
			if (node.hasNextChild()) {					// Traverses to the next linked list node if it exists
				num = findPredictions(node.getNextChild(), num, predictions);		// Calls the print function of this node
				if (0 == num) {				// Returns if there are no strings left to print
					return num;
				}
			}
			if (isTerminalNode(node)) {				// Conditional if the node is a terminal node
				//System.out.printf("\t%s", node.getValue());			// Prints out the string of the node
				predictions[predictions.length-num] = node.getValue(); //Assign node string into string array
				num--;				// Decrements the number of strings to print
			}	
			if (node.hasNextSibling()) {					// Traverses to the next node if it exists
				num = findPredictions(node.getNextSibling(), num, predictions);		// Calls the print function of this node
				if (0 == num) {				// Returns if there are no strings left to print
					return num;
				}
			}
			
			return num;
		}
		
		return num;
	}
}
