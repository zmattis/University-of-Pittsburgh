/**
 * @author Zachary M. Mattis
 * CS 1501
 * Assignment 3
 * November 6, 2017
 *
 * This class extends the basic doubly-linked list by adding specific
 * methods regarding AptInfo objects.  These methods include removing
 * AptInfo objects from the linked list by using a apt's addr
 * and methods for updating an apt's price.
 */

public class AptLL extends LL<ApartmentInfo> {

	/**
	 * Creates a new, empty linked list
	 */
	public AptLL() {

		setListHead(new LLNode<ApartmentInfo>());
	}


	/**
	 * Creates a new linked list with an initial AptInfo object as its list head node
	 * @param apt
	 */
	public AptLL(ApartmentInfo apt) {

		setListHead(new LLNode<ApartmentInfo>(apt));
	}


	/**
	 * Removes a AptInfo object from the linked list based on its addr
	 * @param addr The addr of the AptInfo
	 * @return The ApartmentInfo object that was deleted
	 */
	public ApartmentInfo removeByAddr(String addr) {

		LLNode<ApartmentInfo> previousNode = null;			// Initializes the last node as null
		LLNode<ApartmentInfo> currentNode = getListHead();		// Gets the list head node

		while (currentNode.hasData()) {						// Loops while the node has a ApartmentInfo object stored
			ApartmentInfo listApt = currentNode.getData();
			String listAddr = listApt.getTrieAddr();				// Gets the addr of the node's ApartmentInfo object

			if (addr.compareTo(listAddr) == 0) {					// Conditional if the addr parameter matches the addr of the node
				LLNode<ApartmentInfo> nextNode = currentNode.getNextNode();		// Gets the next node of the current node
				if (previousNode == null) {
					setListHead(nextNode);			// Sets the next node to be the new list head if the previous node is null
				}
				else {
					previousNode.setNextNode(nextNode);			// Otherwise, sets the previous node's next reference to be the next node
				}
				nextNode.setLastNode(previousNode);			// Sets the next node's last reference to be the previous node
				return listApt;
			}
			else {
				previousNode = currentNode;					// If addrs do not match, traverses the linked list to the next node
				currentNode = currentNode.getNextNode();
			}
		}
		return null;				// Returns null if nothing was removed
	}


	/**
	 * Updates the price of an ApartmentInfo object by using its addr
	 * @param addr The apt's addr
	 * @param price The updated price
	 * @return The updated ApartmentInfo object
	 */
	public ApartmentInfo updatePrice(String addr, double price) {

		LLNode<ApartmentInfo> currentNode = getListHead();		// Gets the linked list head

		while (currentNode.hasData()) {				// Traverses the linked list while the node has data
			ApartmentInfo listApt = currentNode.getData();			// Gets the ApartmentInfo object of the node
			String listAddr = listApt.getTrieAddr();					// Gets the apt's addr

			if (addr.compareTo(listAddr) == 0) {				// Conditional if the addrs match
				listApt.setCost(price);					// Sets the price of the ApartmentInfo object
				currentNode.setData(listApt);				// Saves it back into the node
				return listApt;								// Returns the updated ApartmentInfo object
			}
			else {
				currentNode = currentNode.getNextNode();		// Otherwise, traverses the linked list to the next node
			}
		}
		return null;				// Returns null if nothing was updated
	}

}
