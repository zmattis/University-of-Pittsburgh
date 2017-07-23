// CS 0445 Spring 2017
// This implementation is mostly taken from the Carrano LList.  It is modified
// for Assignment 2.  This code MUST BE USED AS IS for Assignment 2.

/**
   A linked implementation of the ADT list.
   @author Frank M. Carrano
   @version 2.0
*/
public class A2LList<T>
{
	protected Node firstNode; // reference to first node
	protected int  numberOfEntries;

	public A2LList()
	{
		clear();
	} // end default constructor

	public final void clear() // note the final method
	{
		firstNode = null;
		numberOfEntries = 0;
	} // end clear

	public int getLength()
	{
		return numberOfEntries;
	}

	public void add(T newEntry)
	{
   		Node newNode = new Node(newEntry);

   		if (isEmpty())
      		firstNode = newNode;
   		else                              // add to end of non-empty list
   		{
      		Node lastNode = getNodeAt(numberOfEntries);
      		lastNode.setNextNode(newNode); // make last node reference new node
   		} // end if

   		numberOfEntries++;
	}  // end add

	public boolean add(int newPosition, T newEntry) // OutOfMemoryError possible
	{
   		boolean isSuccessful = true;

   		if ((newPosition >= 1) && (newPosition <= numberOfEntries + 1))
   		{
      		Node newNode = new Node(newEntry);

      		if (newPosition == 1)                // case 1
      		{
         		newNode.setNextNode(firstNode);
         		firstNode = newNode;
      		}
      		else								// case 2: list is not empty
      		{                                    // and newPosition > 1
         		Node nodeBefore = getNodeAt(newPosition - 1);
         		Node nodeAfter = nodeBefore.getNextNode();
         		newNode.setNextNode(nodeAfter);
         		nodeBefore.setNextNode(newNode);
      		} // end if

      		numberOfEntries++;
   		}
   		else
      		isSuccessful = false;

   		return isSuccessful;
	} // end add

	public boolean isEmpty()
	{
   		boolean result;

   		if (numberOfEntries == 0) // or getLength() == 0
   		{
      		assert firstNode == null;
      		result = true;
   		}
   		else
   		{
      		assert firstNode != null;
      		result = false;
   		} // end if

   		return result;
	} // end isEmpty

	public T remove(int givenPosition)
	{
   		T result = null;                           // return value

   		if ((givenPosition >= 1) && (givenPosition <= numberOfEntries))
   		{
      		assert !isEmpty();

      		if (givenPosition == 1)                 // case 1: remove first entry
      		{
         		result = firstNode.getData();        // save entry to be removed
         		firstNode = firstNode.getNextNode();
      		}
      		else                                    // case 2: not first entry
      		{
         		Node nodeBefore = getNodeAt(givenPosition - 1);
         		Node nodeToRemove = nodeBefore.getNextNode();
         		Node nodeAfter = nodeToRemove.getNextNode();
         		nodeBefore.setNextNode(nodeAfter);
         		result = nodeToRemove.getData();    // save entry to be removed
      		} // end if

      		numberOfEntries--;
   		} // end if

   		return result;                            // return removed entry, or
                                             // null if operation fails
	} // end remove

	public boolean replace(int givenPosition, T newEntry)
	{
   		boolean isSuccessful = true;

   		if ((givenPosition >= 1) && (givenPosition <= numberOfEntries))
   		{
      		assert !isEmpty();

      		Node desiredNode = getNodeAt(givenPosition);
      		desiredNode.setData(newEntry);
   		}
   		else
      		isSuccessful = false;

   		return isSuccessful;
	} // end replace

// This method is intentionally commented out and you may not use it in your
// LinkedListPlus or ReallyLongInt classes.  However, you can uncomment it
// during development / for testing as long as the comments are back in your
// final submitted version.
/*
	public T getEntry(int givenPosition)
	{
   		T result = null;  // result to return

   		if ((givenPosition >= 1) && (givenPosition <= numberOfEntries))
   		{
      		assert !isEmpty();
      		result = getNodeAt(givenPosition).getData();
   		} // end if

   		return result;
	} // end getEntry
*/
	public boolean contains(T anEntry)
	{
   		boolean found = false;
   		Node currentNode = firstNode;

   		while (!found && (currentNode != null))
   		{
      		if (anEntry.equals(currentNode.getData()))
         		found = true;
      		else
         		currentNode = currentNode.getNextNode();
   		} // end while

   		return found;
	} // end contains

   // Returns a reference to the node at a given position.
   // Precondition: List is not empty;
   //               1 <= givenPosition <= numberOfEntries
	private Node getNodeAt(int givenPosition)
	{
		assert !isEmpty() && (1 <= givenPosition) && (givenPosition <= numberOfEntries);
		Node currentNode = firstNode;

      // traverse the list to locate the desired node
		for (int counter = 1; counter < givenPosition; counter++)
			currentNode = currentNode.getNextNode();

		assert currentNode != null;

		return currentNode;
	} // end getNodeAt

	// Note that this class is protected so you can access it directly from
	// your LinkedListPlus and ReallyLongInt classes.  However, in case you
	// prefer using accessors and mutators, those are also provided here.
   protected class Node
   {
      protected T data; 	// entry in list
      protected Node next; 	// link to next node

      protected Node(T dataPortion)
      {
         this(dataPortion, null);
      } // end constructor

      protected Node(T dataPortion, Node nextNode)
      {
         data = dataPortion;
         next = nextNode;
      } // end constructor

      protected T getData()
      {
         return data;
      } // end getData

      protected void setData(T newData)
      {
         data = newData;
      } // end setData

      protected Node getNextNode()
      {
         return next;
      } // end getNextNode

      protected void setNextNode(Node nextNode)
      {
         next = nextNode;
      } // end setNextNode
   } // end Node
} // end A2LList
