import java.util.ArrayList;

public class Sieve {

    private static int _max = 0;

    /**
     * "Debug" version of PrintSieve.  This will show all numbers
     * along with their T/F mark of primality.
     * @param results - list of ints from 1 to n
     * @param prime - boolean array indicating primality
     */

    public static void printsieve(int[] results, boolean[] prime) {
	    System.out.print("> ");
	    char primeChar = ' ';

	    // Just loop through the array and print the values.
	    // Put a (T) after each one if it has been marked
	    // prime; put a (F) after one if it has  been marked
	    // composite (not prime).

	    for (int j = 0; j < results.length;) {
		primeChar = prime[j] ? 'T' : 'F';
		System.out.print(results[j] + "(" + primeChar + ") ");
		j++;
	    }
	    System.out.println();
    }

    /**
     * Prints out an array which should contain all prime values.
     * Really, will print out any array.
     * @param results array of ints to print out
     */

    public static void printSieve(int[] results) {

	// As long as there are elements in the array,
	// print it.  Otherwise, print "BLANK".

	if (results.length == 0 || results != null) {
	    System.out.print("> ");
	    for (int j = 1; j < results.length;) {
		System.out.print(results[j] + " ");
		j++;
	    }
	} else {
	    System.out.print("BLANK");
	}

	System.out.println("");
	
    }

    /**
	 * Returns array representing prime status of integers.
     * @param num the size of the array to return
	 * @return boolean array representing prime
     */

    public static boolean[] getTrueArray(int num) {
	boolean[] toReturn = new boolean[num];
	for (int j = 0; j < num; j++) {
	    toReturn[j] = true;
	}
	// Return an all-true array.
	return toReturn;

    }

    /**
     * Convert our "behind the scenes" arrays - the list of ints from
     * 1 to n and the same-size array which indicates their primality -
     * into a simple array of all prime values.
     * @param results an array with all ints from 1 to n
     * @return int[] the prime numbers from 1 to n
     */

    public static int[] convertResults(int[] results, boolean[] prime) {

	// Create an ArrayList.	 If a value is true for primality,
	// add it to the array list.

	ArrayList<Integer> actual = new ArrayList<Integer>();
	for (int j = 0; j < results.length; j++) {
	    if (prime[j]) {
		actual.add(new Integer(j + 1));
	    }
	}

	// Since we want to turn this back into a plain old array
	// of ints, create an array of the same size as the ArrayList.
	// Then add each element from the ArrayList into the proper
	// (sorted ascending) location in the returned array.

	// Note that the elements in the ArrayList are Integers, and
	// the toReturn variable is an int array, but Java will
	// automatically convert thanks to autoboxing as long as
	// you are using Java 1.5 or higher.

	int[] toReturn = new int[actual.size()];

	for (int j = 0; j < actual.size(); j++) {
	    toReturn[j] = actual.get(j);
	}
	return toReturn;
    }

    /**
     * Given an array of ints, starting at 1, calculate which ones
     * are prime using the Sieve of Eratosthenes.
     * Uncomment println statements to watch it work.
     * @param results array of ints starting at 1
     * @return int[] array of all prime ints
     */

    public static int[] calculateSieve(int[] results) {
	int ptr = 1; // means value 2
	int size = results.length;
	// At this point, assume all numbers are prime.
	boolean[] prime = getTrueArray(size);
	while (ptr < Math.sqrt(results.length)) {
	    // if this number is marked false, ignore it - all other
	    // numbers which are multiples of it will also already
	    // be marked false
	    // Otherwise, loop through and look for any multiples of
	    // it, which should now be marked false
	    if (prime[ptr]) {
		int val = results[ptr]; // value pointed at
		int localPtr = ptr; // secondary pointer
		int counter = 2; // multiple counter
		int comp = 0; // computed value
		// System.out.println("Ptr = " + ptr + ",  val = " + val);

		// Loop through the rest of the loop (starting past ptr,
		// which is what localPtr is equal to now) and look for
		// any multiples of that number.

		// These numbers are composite, so mark their prime[localPtr]
		// value as false.

		while (localPtr <= size ) {
		    comp = val * counter;
		    // System.out.println("\t" + val + " * " + counter + " = " + comp);
		    // Convert our one-based array ptr to Java zero-based array ptr
		    localPtr = comp - 1;
		    if (localPtr < size) {
			// System.out.println("\tSetting " + results[localPtr] + " to F");
			prime[localPtr] = false;
		    }
		    counter++;
		}
	    }
	    ptr++;
	}
	// Debug-print the behind the scenes sieve values
	// printsieve(results, prime);
	results = convertResults(results, prime);
	return results;
    }

    /**
     * Generates our "off-by-one" array which will be all numbers
     * that we calculate for the sieve.	 That is, the zeroth element
     * will contain the value 1, element 1 will contain the value 2,
     * etc.
     * @param maxSize - the size of the array to return
     * @return int[] array of correct format, as indicated above
     */

    public static int[] generateSieve(int maxSize) {
	int size = maxSize;
	int[] toReturn = new int[maxSize];
	for (int j = 0; j <= maxSize; j++) {
	    if (j == 0) {
		j++;
	    }
	    toReturn[j - 1] = j;
	}
	return toReturn;

    }

    /**
     * Based on args provided, calculate the maximum value to calculate.
     * @param args first element
     * @return maximum size of array
     */
    public static int calculateMax(String[] args) throws IllegalArgumentException {
		int toReturn = -1; // default (invalid) value
		if (args.length > 0) {
			toReturn = (int) Integer.parseInt(args[0]);
			if (toReturn < 1) {
			// User did not enter a valid integer
			throw new IllegalArgumentException();
			}
		} else {
			// User forgot to enter an argument!
			throw new IllegalArgumentException();
		}
		return toReturn;
    }


    /**
     * Main method.  Accepts one argument, which should be parseable
     * as an integer, ignores any other arguments.  This argument is
     * the maximum value to calculate for the sieve.
     * If no argument is provided, or it cannot be parsed, assume
     * the user meant 100.
     * @param args maximum value for sieve as first arg
     */

    public static void main(String[] args) {
	System.out.println("Sieve of Eratosthenes");

	// Get the passed-in argument of the maximum value for
	// the sieve to calculate the primality of.

	// If the user did not enter any arguments, or the argument
	// is not a positive integer (1 or greater), then the
	// program should assume that the user meant 100.

	// Other arguments past the first will be ignored.

	try {
	    _max = calculateMax(args);
	} catch (IllegalArgumentException ex) {
	    System.out.println("You forgot to enter a valid integer (> 0)!");
	    System.out.println("Assuming you meant to type 100...");
	    _max = 100;
	}

	// Calculate sieve and print it out
	int[] sieve = generateSieve(_max);
	int[] results = calculateSieve(sieve);
	printSieve(results);
    }

}
