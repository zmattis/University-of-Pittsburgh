/**
 * @author Zachary M. Mattis
 * CS 1501
 * Assignment 3
 * November 6, 2017
 *
 * This class functions as an R-Way Trie to store array indices for AptInfo
 * objects stored in a PriorityQueue.java heap.  This trie uses the st addr as
 * keys to reach the heap index.  Since Street Addr have a specific
 * character set, each character in the addr corresponds to an array
 * index in the R-Way trie.  The trie returns -1 when a specified addr
 * is not present in its corresponding heap (I.e., it has been
 * removed).
 */

public class AWayTrie {

	private AWayTrieNode trieHead;		// The first node in the trie


	/**
	 * Creates a new trie by setting its initial node
	 */
	public AWayTrie() {

		trieHead = new AWayTrieNode();
	}


	/**
	 * Stores an index in the trie using a apt's address as keys
	 * @param addr The addr of the apt
	 * @param heapIndex The index of the AptInfo object in its heap
	 */
	public void storeHeapIndex(String addr, int heapIndex) {

		AWayTrieNode trieNode = trieHead;					// The starting node of the trie

		for (int i = 0; i < addr.length(); i++) {					// For each character of the addr, navigates through each node of the trie
			int trieIndex = translateCharToInt(addr.charAt(i));
			if (trieNode.visitTrieIndex(trieIndex) == null) {		// If a node does not exist, a new one is created
				trieNode.setTrieNode(trieIndex);
			}
			trieNode = trieNode.visitTrieIndex(trieIndex);
		}
		trieNode.setHeapIndex(heapIndex);					// Stores the index in the last node
	}


	/**
	 * Retrieves a apt's index in the heap by using its addr as keys
	 * @param addr The addr of the apt
	 * @return The index of the AptInfo object in its heap
	 */
	public int getHeapIndex(String addr) {

		AWayTrieNode trieNode = trieHead;					// The starting node of the trie

		for (int i = 0; i < addr.length(); i++) {					// For each character of the addr, navigates through each node of the trie
			int trieIndex = translateCharToInt(addr.charAt(i));
			if (trieNode.visitTrieIndex(trieIndex) == null) {		// If a node does not exist, the method returns -1 (addr is not registered)
				return -1;
			}
			trieNode = trieNode.visitTrieIndex(trieIndex);
		}
		return trieNode.getHeapIndex();					// Returns the index of the apt in its heap
	}


	/**
	 * Removes an apt from the trie by setting its index to -1
	 * @param addr The addr of the apt
	 */
	public void removeHeapIndex(String addr) {

		AWayTrieNode trieNode = trieHead;					// The starting node of the trie

		for (int i = 0; i < addr.length(); i++) {					// For each character of the addr, navigates through each node of the trie
			int trieIndex = translateCharToInt(addr.charAt(i));
			if (trieNode.visitTrieIndex(trieIndex) == null) {		// If a node does not exist, the method prints a message and returns
				System.out.printf("The entered Apartment does not exist!\n");
				return;
			}
			trieNode = trieNode.visitTrieIndex(trieIndex);
		}
		trieNode.setHeapIndex(-1);						// Sets the heap index to -1
	}


	/**
	 * Transforms a character from the string into an index of the array
	 * @param ch A character of the addr
	 * @return An index for the array
	 */
	private int translateCharToInt(char ch) {

		switch (ch) {
			case '0':
				return 0;
			case '1':
				return 1;
			case '2':
				return 2;
			case '3':
				return 3;
			case '4':
				return 4;
			case '5':
				return 5;
			case '6':
				return 6;
			case '7':
				return 7;
			case '8':
				return 8;
			case '9':
				return 9;
			case 'A':
			case 'a':
				return 10;
			case 'B':
			case 'b':
				return 11;
			case 'C':
			case 'c':
				return 12;
			case 'D':
			case 'd':
				return 13;
			case 'E':
			case 'e':
				return 14;
			case 'F':
			case 'f':
				return 15;
			case 'G':
			case 'g':
				return 16;
			case 'H':
			case 'h':
				return 17;
			case 'J':
			case 'j':
				return 18;
			case 'K':
			case 'k':
				return 19;
			case 'L':
			case 'l':
				return 20;
			case 'M':
			case 'm':
				return 21;
			case 'N':
			case 'n':
				return 22;
			case 'P':
			case 'p':
				return 23;
			case 'R':
			case 'r':
				return 24;
			case 'S':
			case 's':
				return 25;
			case 'T':
			case 't':
				return 26;
			case 'U':
			case 'u':
				return 27;
			case 'V':
			case 'v':
				return 28;
			case 'W':
			case 'w':
				return 29;
			case 'X':
			case 'x':
				return 30;
			case 'Y':
			case 'y':
				return 31;
			case 'Z':
			case 'z':
			return 32;
			default:			// Character ' '
				return 33;
		}
	}
}
