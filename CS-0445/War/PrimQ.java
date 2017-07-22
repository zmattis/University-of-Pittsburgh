// CS 0445 Spring 2017
// Assignment 1 PrimQ<T> interface
// Carefully read the specifications for each of the operations and
// implement them correctly in your MultiDS class.

// The overall logic of the PrimQ<T> is the following:
//		Data is logically "added" in the same order that it is "removed".
// However, there is no requirement for the physical storage of the actual
// data.  Your only requirement for the MultiDS<T> class is that all of the
// methods work as specified and that your MultiDS<T> class have an array as its
// its primary data.  You MAY NOT use ArrayList, Vector or any predefined
// collection class for your MultiDS<T> data.  However, you may want to use some
// some additional instance variables to keep track of the data effectively.

// Note: Later in the term we will discuss how to implement a queue in an
// efficient way.

public interface PrimQ<T>
{
	// Add a new Object to the PrimQ<T> in the next available location.  If
	// all goes well, return true.  If there is no room in the PrimQ for
	// the new item, return false (you should NOT resize it)
	public boolean addItem(T item);
	
	// Remove and return the "oldest" item in the PrimQ.  If the PrimQ
	// is empty, return null.
	public T removeItem();
	
	// Return true if the PrimQ is full, and false otherwise
	public boolean full();
	
	// Return true if the PrimQ is empty, and false otherwise
	public boolean empty();
	
	// Return the number of items currently in the PrimQ
	public int size();

	// Reset the PrimQ to empty status by reinitializing the variables
	// appropriately
	public void clear();
}