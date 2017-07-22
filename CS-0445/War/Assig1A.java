// CS 0445 Spring 2017
// Assig1A driver program.  This program must work as is with your
// MultiDS<T> class.  Look carefully at all of the method calls so that
// you create your MultiDS<T> methods correctly.  For example, note the
// constructor calls and the toString() method call.  The output should
// be identical to my sample output, with the exception of the result of
// the shuffle() methods -- since this should be random yours should not
// match mine.
public class Assig1A
{
	public static void main(String [] args)
	{
		// Testing constructor and PrimQ<T> interface
		PrimQ<Integer> theQ = new MultiDS<Integer>(5);

		// Testing addItem
		for (int i = 0; i < 6; i++)
		{
			Integer newItem = new Integer(2 * i);
			if (!(theQ.full()))
			{
				theQ.addItem(newItem);
				System.out.println(newItem + " added to Q");
			}
			else
			{
				System.out.println("No room for " + newItem);
			}
		}

		// Testing removeItem
		while (!(theQ.empty()))
		{
			Integer oldItem = theQ.removeItem();
			System.out.println(oldItem + " retrieved from Q");
		}
		Integer noItem = theQ.removeItem();
		if (noItem == null)
			System.out.println("Nothing in the Q");

		// Testing array management
		int count = 1;
		PrimQ<String> theQ2 = new MultiDS<String>(5);
		String theItem = new String("Item " + count);
		System.out.println("Adding " + theItem);
		theQ2.addItem(theItem);
		for (int i = 0; i < 8; i++)
		{
			count++;
			theItem = new String("Item " + count);
			System.out.println("Adding " + theItem);
			theQ2.addItem(theItem);
			theItem = theQ2.removeItem();
			System.out.println("Removing " + theItem);
		}
		int sz = theQ2.size();
		System.out.println("There are " + sz + " items in the buffer");

		// This code will test the toString() method and the Reorder
		// interface.
		System.out.println("\nAbout to test Reorder methods");
		MultiDS<Integer> newDS = new MultiDS<Integer>(15);
		for (int i = 0; i < 8; i++)
		{
			newDS.addItem(new Integer(i));
		}
		System.out.println(newDS.toString());
		System.out.println("Reversing");
		newDS.reverse();
		System.out.println(newDS.toString());
		System.out.println("Removing 3 items then adding 1");
		Integer bogus = newDS.removeItem();
		bogus = newDS.removeItem();
		bogus = newDS.removeItem();
		newDS.addItem(new Integer(8));
		System.out.println(newDS.toString());
		System.out.println("Reversing");
		newDS.reverse();
		System.out.println(newDS.toString());
		System.out.println("Shifting right");
		newDS.shiftRight();
		System.out.println(newDS.toString());
		System.out.println("Shifting left twice");
		newDS.shiftLeft();
		newDS.shiftLeft();
		System.out.println(newDS.toString());
		
		System.out.println("\nAbout to test shuffle...");
		newDS.clear();
		for (int i = 0; i < 10; i++)
		{
			newDS.addItem(new Integer(i));
		}
		System.out.println(newDS.toString());
		System.out.println("Shuffling...");
		newDS.shuffle();
		System.out.println(newDS.toString());
		System.out.println("Removing 2 and adding 1");
		bogus = newDS.removeItem();
		bogus = newDS.removeItem();
		newDS.addItem(new Integer(22));
		System.out.println(newDS.toString());
		System.out.println("Shuffling again");
		newDS.shuffle();
		System.out.println(newDS.toString());
	}
}