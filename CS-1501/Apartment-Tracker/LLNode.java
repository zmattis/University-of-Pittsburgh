/**
 * @author Zachary M. Matts
 * CS 1501
 * Assignment 3
 * November 6, 2017
 *
 * This class functions as the node for a basic linked list.  It
 * contains methods for accessing/modifying the node's data as
 * well as accessing/modifying neighboring nodes in the list.
 */

public class LLNode<T> {

	private T data;						// The data of the node
	private LLNode<T> nextNode;			// The next node in the linked list
	private LLNode<T> lastNode;			// The previous node in the linked list


	/**
	 * Creates a new, empty node with all its fields set to null
	 */
	public LLNode() {

		setData(null);
		setNextNode(null);
		setLastNode(null);
	}


	/**
	 * Creates a new node with its data field set to the object
	 * @param obj The object to set
	 */
	public LLNode(T obj) {

		setData(obj);				// Sets the data of the node as the object
		setNextNode(null);
		setLastNode(null);
	}


	/**
	 * Sets the data of the node
	 * @param obj The object to set
	 */
	public void setData(T obj) {

		data = obj;
	}


	/**
	 * Gets the data of the node
	 * @return The object stored in the node
	 */
	public T getData() {

		return data;
	}


	/**
	 * Checks if the node has data
	 * @return True if the data of the node is not null
	 */
	public boolean hasData() {

		return (getData() != null);
	}


	/**
	 * Sets the next node of the current node
	 * @param next The next node of the current node
	 */
	protected void setNextNode(LLNode<T> next) {

		nextNode = next;
	}


	/**
	 * Sets the previous node of the current node
	 * @param last The previous node of the current node
	 */
	protected void setLastNode(LLNode<T> last) {

		lastNode = last;
	}


	/**
	 * Checks if the current node has a next node
	 * @return True if the next node is not null
	 */
	public boolean hasNextNode() {

		return (getNextNode() != null);
	}


	/**
	 * Gets the next node attached to the current node
	 * @return The next node of the current node
	 */
	public LLNode<T> getNextNode() {

		return nextNode;
	}


	/**
	 * Gets the previous node attached to the current node
	 * @return The previous node of the current node
	 */
	public LLNode<T> getLastNode() {

		return lastNode;
	}
}
