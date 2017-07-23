// CS 0445 Spring 2017
// Zack Mattis -- 4020473

public class ReallyLongInt 	extends LinkedListPlus<Integer>
							implements Comparable<ReallyLongInt>
{
	// Instance variables are inherited.  You may not add any new instance variables

	// Default constructor
	private ReallyLongInt()
	{
		super();
	}

	// Note that we are adding the digits here in the FRONT. This is more efficient
	// (no traversal is necessary) and results in the LEAST significant digit first
	// in the list.  It is assumed that String s is a valid representation of an
	// unsigned integer with no leading zeros.
	public ReallyLongInt(String s)
	{
		super();
		char c;
		int digit;
		// Iterate through the String, getting each character and converting it into
		// an int.  Then make an Integer and add at the front of the list.  Note that
		// the add() method (from A2LList) does not need to traverse the list since
		// it is adding in position 1.  Note also the the author's linked list
		// uses index 1 for the front of the list.
		for (int i = 0; i < s.length(); i++)
		{
			c = s.charAt(i);
			if (('0' <= c) && (c <= '9'))
			{
				digit = c - '0';
				this.add(1, new Integer(digit));
			}
			else throw new NumberFormatException("Illegal digit " + c);
		}
	}

	// Simple call to super to copy the nodes from the argument ReallyLongInt
	// into a new one.
	public ReallyLongInt(ReallyLongInt rightOp)
	{
		super(rightOp);
	}

	// Method to put digits of number into a String.  Since the numbers are
	// stored "backward" (least significant digit first) we first reverse the
	// number, then traverse it to add the digits to a StringBuilder, then
	// reverse it again.  This seems like a lot of work, but given the
	// limitations of the super classes it is what we must do.
	public String toString()
	{
		StringBuilder sb = new StringBuilder();
		if (numberOfEntries > 0)
		{
			this.reverse();
			for (Node curr = firstNode; curr != null; curr = curr.next)
			{
				sb.append(curr.data);
			}
			this.reverse();
		}
		return sb.toString();
	}

	// Adding two ReallyLongInt's together
	public ReallyLongInt add(ReallyLongInt rightOp)
	{
		int carry=0;
		int result=0;
		Node temp1=rightOp.firstNode;
		Node temp2=this.firstNode;
		ReallyLongInt answer = new ReallyLongInt(rightOp);


		int flag=0;
		Node newNode = new Node(0);

		while(temp1.next!=null || temp2.next!=null){


			result=temp1.data+temp2.data+carry;

			carry=0;

			if (result>=10){
			carry=1;
			result=result%10;
			}

			if (flag==0){
				newNode.data=result;
				answer.firstNode=newNode;
				flag=-1;
			}
			else{
			newNode.next = new Node(result);
			newNode=newNode.next;
			}

			if (temp1.next!=null){
				temp1=temp1.next;
			}
			else{
				temp1=new Node(0);
			}

			if (temp2.next!=null){
				temp2=temp2.next;
			}
			else{
				temp2=new Node(0);
			}

		}

		result=temp1.data+temp2.data+carry;

			carry=0;

			if (result>=10){
			carry=1;
			result=result%10;
			}

			if (flag==0){
				newNode.data=result;
				answer.firstNode=newNode;
				flag=-1;
			}
			else{
			newNode.next = new Node(result);
			newNode=newNode.next;
			}



		if (carry==1){
			Node myNode = new Node(1);
			newNode.next=myNode;
		}
		return answer;

	}

	// Subtract a ReallyLongInt
	public ReallyLongInt subtract(ReallyLongInt rightOp)
	{
		int borrow=0;
		int result=0;
		Node temp1=rightOp.firstNode;
		Node temp2=this.firstNode;
		ReallyLongInt answer = new ReallyLongInt(rightOp);


		int flag=0;
		Node newNode = new Node(0);

		while(temp1.next!=null || temp2.next!=null){



			result=temp2.data-temp1.data-borrow;
			borrow=0;

			if (result<0){
				result+=10;
				borrow=1;
			}

			if (flag==0){
				newNode.data=result;
				answer.firstNode=newNode;
				flag=-1;
			}
			else{
			newNode.next = new Node(result);
			newNode=newNode.next;
			}

			if (temp1.next!=null){
				temp1=temp1.next;
			}
			else{
				temp1=new Node(0);
			}

			if (temp2.next!=null){
				temp2=temp2.next;
			}
			else{
				temp2=new Node(0);
			}

		}

			result=temp2.data-temp1.data-borrow;
			borrow=0;

			if (result<0){
				result+=10;
				borrow=1;
			}

			if (flag==0){
				newNode.data=result;
				answer.firstNode=newNode;
				flag=-1;
			}
			else{
			newNode.next = new Node(result);
			newNode=newNode.next;
			}

		//Remove Leading 0's
		answer.reverse();
		Node temp=answer.firstNode;
		while (temp.data==0){
			temp=temp.next;
		}
		answer.firstNode=temp;
		answer.reverse();


		return answer;

	}

	// Compare two ReallyLongInt's
	public int compareTo(ReallyLongInt rOp)
	{
		if(this.numberOfEntries>rOp.numberOfEntries)
			return 1;

		else if(this.numberOfEntries<rOp.numberOfEntries)
			return -1;

		this.reverse();
		rOp.reverse();

		Node left = firstNode;
		Node right = rOp.firstNode;
		int result = 0;

		//if they are of equal number of digits
				while(right!=null)
					{
						if(left.getData()>right.getData())
							{
								this.reverse();
								rOp.reverse();
								return 1;
							}
						else if(left.getData()<right.getData())
							{
								this.reverse();
								rOp.reverse();
								return -1;
							}
					right=right.next;
					left=left.next;
					} //end while


			this.reverse();
			rOp.reverse();

			return 0;
	}

	// Check to see if two ReallyLongInt's are equal
	public boolean equals(Object rightOp)
	{
		ReallyLongInt rOp = (ReallyLongInt)rightOp;
		int result = compareTo(rOp);

		if (result==0){
			return true;
		}

		return false;

	}

	// ReallyLongInt x 10^x
	public void multTenToThe(int num)
	{
		for (int i=0;i<num; i++){
			Node newNode = new Node(0);
			newNode.next=firstNode;
			firstNode=newNode;
		}
	}

	// ReallyLongInt / 10^x
	public void divTenToThe(int num)
	{
		Node temp=firstNode;
		for (int i=0; i<num-1; i++){
			temp=temp.next;
		}

		firstNode=temp.next; //Change firstNode pointer
		temp.next=null; //Remove old pointer

	}
}
