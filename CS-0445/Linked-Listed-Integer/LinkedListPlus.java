// CS 0445 Spring 2017
// Zack Mattis -- 4020473
// LinkedListPlus<T> class  implementation

public class LinkedListPlus<T> extends A2LList<T>
{
	// Default constructor simply calls super()
	public LinkedListPlus()
	{
		super();
	}

	// Copy constructor.  This is a "deepish" copy so it will make new
	// Node objects for all of the nodes in the old list.  However, it
	// is not totally deep since it does NOT make copies of the objects
	// within the Nodes -- rather it just copies the references.
	public LinkedListPlus(LinkedListPlus<T> oldList)
	{
		super();
		if (oldList.getLength() > 0)
		{
			// Special case for first Node since we need to set the
			// firstNode instance variable.
			Node temp = oldList.firstNode;		// front of old list
			Node newNode = new Node(temp.data);	// copy the data
			firstNode = newNode;				// set front of new list

			// Now we traverse the old list, appending a new Node with
			// the correct data to the end of the new list for each Node
			// in the old list.  Note how the loop is done and how the
			// Nodes are linked.
			Node currNode = firstNode;
			temp = temp.next;
			while (temp != null)
			{
				currNode.next = new Node(temp.data);
				temp = temp.next;
				currNode = currNode.next;
			}
			numberOfEntries = oldList.numberOfEntries;
		}
	}

	// Make a StringBuilder then traverse the nodes of the list, appending the
	// toString() of the data for each node to the end of the StringBuilder.
	// Finally, return the StringBuilder as a String.
	public String toString()
	{
		StringBuilder b = new StringBuilder();
		for (Node curr = firstNode; curr != null; curr = curr.next)
		{
			b.append(curr.data.toString());
			b.append(" ");
		}
		return b.toString();
	}

	// Shift left num positions.  This will shift out the first num Nodes
	// of the list, with Node num+1 now being at the front.
	public void leftShift(int num)
	{
		if (num >= this.getLength())	// If we shift more than the number of
		{								// Nodes, just initialize to empty
			firstNode = null;
			numberOfEntries = 0;
		}
		else if (num > 0)
		{
			Node temp = firstNode;		// Start at front
			for (int i = 0; i < num-1; i++)	 // Get to node BEFORE the one that
			{								 // should be the new firstNode
				temp = temp.next;
			}
			firstNode = temp.next;		// Set firstNode to Node after temp
			temp.next = null;		    // Disconnect old prefix of list (just
										// to be extra cautious)
			numberOfEntries = numberOfEntries - num;	// Update logical size
		}
	}

	// Remove num items from the end of the list
	public void rightShift(int num)
	{
		if (num >= this.getLength())	// If we shift more than the number of
		{								// Nodes, just initialize to empty
			firstNode = null;
			numberOfEntries = 0;
		}
		else if (num > 0)
		{
			Node temp = firstNode;		// Start at front
			for (int i = 0; i < this.getLength()-num-1; i++)	 // Get to node BEFORE the one that
			{								 // should be the new LastNode
				temp = temp.next;
			}
			temp.next = null;		    // Remove the rest of the entires

			numberOfEntries = numberOfEntries - num;	// Update logical size
		}

	}

	// Remove from the front and add at the end.  Note that this method should
	// not create any new Node objects -- it is simply moving them.  If num
	// is negative it should still work, actually doing a right rotation.
	public void leftRotate(int num)
	{
		num=num%this.getLength(); //Use modulus to simplify the shift

		// Call a right shift if num is negative
		if (num<0){
			rightRotate(-num);
		}

		else if(num>0){

		Node temp = firstNode;

		for (int i =0; i<num-1; i++){
			temp=temp.next;
		}

		Node lastNode=temp;

		//Find Final node
		while(temp.next!=null){
			temp=temp.next;
		}

		//Set original last node to firstNode
		temp.next=firstNode;

		//Set firstNode pointer
		firstNode=lastNode.next;

		//Remove pointer for the last node (null)
		lastNode.next=null;

		}

	}

	// Remove from the end and add at the front.  Note that this method should
	// not create any new Node objects -- it is simply moving them.  If num
	// is negative it should still work, actually doing a left rotation.
	public void rightRotate(int num)
	{
		if (num>0){
			leftRotate( (this.getLength()-num) );
		}
		else if (num<0){
			leftRotate(-num);
		}


	}

	// Reverse the nodes in the list.  Note that this method should not create
	// any new Node objects -- it is simply moving them.
	public void reverse()
	{

		Node currNode=firstNode;
		Node nextNode=null;
		Node prevNode=null;


		 while(currNode!=null){
			 nextNode = currNode.next;
			 currNode.next = prevNode;
			 prevNode = currNode;
			 currNode = nextNode;
		}

		firstNode=prevNode;

	}
}
