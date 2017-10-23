/**
 * @author Zack Mattis
 * CS 1501
 * Assignment 1
 * September 25, 2017
 * 
 * This class functions as the nodes for a de la Briandais trie.  The "nextSibling"
 * field refers to adjacent nodes within the linked list whereas the "nextChild"
 * field refers to the head of a linked list that is attached to the node.
 * 
 * @param <S> The key the node will use
 * @param <T> The type of object the node will hold
 */

public class DLBNode<S, T> {
	
	private S key;				// The key of the node
	private T value;			// The value of completed node path
	private int cnt;			// Count variable
	private DLBNode<S, T> nextChild, nextSibling, lastChild, lastSibling;			// Holds references to adjacent nodes
	
	
	/**
	 * Creates a new de la Briandais trie node with all variables set to null.
	 */
	public DLBNode() {
		
		key = null;
		value = null;
		nextChild = null;
		nextSibling = null;
		lastChild = null;
		lastSibling = null;
		cnt = 0;
	}
	
	
	/**
	 * Creates a new de la Briandais trie node by copying an existing one
	 * @param node
	 */
	public DLBNode(DLBNode<S, T> node) {
		
		key = node.getKey();
		value = node.getValue();
		nextChild = node.getNextChild();
		nextSibling = node.getNextSibling();
		lastChild = node.getLastChild();
		lastSibling = node.getLastSibling();
		cnt = node.getCount();
	}
	
	
	/**
	 * Accesses the node's key
	 * @return The key of the node
	 */
	public S getKey() {
		
		return key;
	}
	
	
	/**
	 * Stores key into the node
	 * @param value The key to store in the node
	 */
	public void setKey(S keyValue) {
		
		key = keyValue;
	}
	
	
	/**
	 * Checks if the node currently has any key
	 * @return Returns true if the key is null
	 */
	public boolean hasNoKey() {
		
		return (key == null);
	}
	
	
	/**
	 * Checks to see if the node has a stored value
	 * @return Returns true if the node has a value
	 */
	public boolean hasValue() {
		
		return (value != null);
	}
	
	
	/**
	 * Accesses the node's value
	 * @return The value of the node's end path
	 */
	public T getValue() {
		
		return value;
	}
	
	
	/**
	 * Sets the node's value
	 * @param val The value to be stored
	 */
	public void setValue(T val) {
		
		value = val;
	}
	
	/**
	 * Gets the node's count
	 * @return Count
	 */
	public int getCount() {
		return cnt;
	}
	
	/**
	 * Sets the node's count
	 * @param count The count to be stored
	 */
	public void setCount(int count) {
		
		cnt = count;
	}
	
	
	
	/**
	 * Checks to see if the node has a neighboring node within the linked list
	 * @return Returns true if the next node reference is not null
	 */
	public boolean hasNextSibling() {
		
		return (nextSibling != null);
	}
	
	
	/**
	 * Checks to see if the node has a separate linked list attached to it
	 * @return Returns true if the next list reference is not null
	 */
	public boolean hasNextChild() {
		
		return (nextChild != null);
	}
	
	
	/**
	 * Checks to see if the node has a previous node attached to it
	 * @return Returns true if the previous node reference is not null
	 */
	public boolean hasLastSibling() {
		
		return (lastSibling != null);
	}
	
	
	/**
	 * Checks to see if the node has a previous linked list attached to it
	 * @return Returns true if the last list reference is not null
	 */
	public boolean hasLastChild() {
		
		return (lastChild != null);
	}
	
	/**
	 * Fetches the reference for an adjacent node
	 * @return The reference of the neighboring node
	 */
	public DLBNode<S, T> getNextSibling() {
		
		return nextSibling;
	}
	
	
	/**
	 * Fetches the reference for an adjacent linked list's root node
	 * @return The reference of the neighboring linked list head
	 */
	public DLBNode<S, T> getNextChild() {
		
		return nextChild;
	}
	
	
	/**
	 * Fetches the reference for the previous node
	 * @return The reference of the previous node
	 */
	public DLBNode<S, T> getLastSibling() {
		
		return lastSibling;
	}
	
	
	/**
	 * Fetches the reference for a previous linked list node
	 * @return The reference of a previous linked list node
	 */
	public DLBNode<S, T> getLastChild() {
		
		return lastChild;
	}
	
	
	/**
	 * Sets the reference for the adjacent node
	 * @param node The node to attach
	 */
	public void setNextSibling(DLBNode<S, T> node) {
		
		nextSibling = node;
		node.setLastSibling(this);
	}
	
	
	/**
	 * Sets the reference for the neighboring linked list
	 * @param list The head of the linked list to attach
	 */
	public void setNextChild(DLBNode<S, T> list) {
		
		nextChild = list;
		list.setLastChild(this);
	}
	
	
	/**
	 * Sets the reference for the previous node
	 * @param node The previous node
	 */
	public void setLastSibling(DLBNode<S, T> node) {
		
		lastSibling = node;
	}
	
	
	/**
	 * Sets the reference for the previous linked list
	 * @param list A node of the previous linked list
	 */
	public void setLastChild(DLBNode<S, T> list) {
		
		lastChild = list;
	}
}
