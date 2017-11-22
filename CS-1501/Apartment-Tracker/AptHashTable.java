/**
 * @author Zachary M. Mattis
 * CS 1501
 * Assignment 3
 * November 6, 2017
 *
 * This class functions as a hash table for storing linked lists of
 * ApartmentInfo objects grouped by the apartment's city.  Hash indices are
 * generated by using the first 7 characters of an apartment's city,
 * concatenated w/ " " if it is not long enough.Horner's method of
 * hashing is used, modulo 311 (prime number).
 *
 */

public class AptHashTable {

	private AptLL[] aptArray;				// The hash table containing linked lists of apt types


	/**
	 * Creates a new hash table of size 311, a large prime number.
	 */
	public AptHashTable() {

		aptArray = (AptLL[])new AptLL[311];
	}


	/**
	 * Inserts an ApartmentInfo object into the hash table
	 * @param apt The ApartmentInfo object to add
	 */
	public void insertApt(ApartmentInfo apt) {

		int hashIndex = generateHashIndex(apt);				// Generates a hash index of the apt
		addApt(apt, hashIndex);						// Places the apt in the array
	}


	/**
	 * Removes an ApartmentInfo object from the hash table
	 * @param apt The ApartmentInfo object to remove
	 */
	public void removeApt(ApartmentInfo apt) {

		int hashIndex = generateHashIndex(apt);				// Generates a hash index of the apt
		deleteApt(apt, hashIndex);					// Removes the apt from the array
	}


	/**
	 * Updates the price of a ApartmentInfo object in the array
	 * @param apt The ApartmentInfo object to update
	 * @param price The updated price
	 * @return The updated ApartmentInfo object
	 */
	public ApartmentInfo updatePrice(ApartmentInfo apt, double price) {

		int hashIndex = generateHashIndex(apt);				// Generates a hash index of the apt
		String addr = apt.getTrieAddr();							// Gets the addr of the apt
		AptLL list = aptArray[hashIndex];			// Gets the specific apt linked list from the hash table
		return list.updatePrice(addr, price);				// Updates and returns the ApartmentInfo object's price specified by the VIN
	}


	/**
	 * Gets the linked list of a specific apt's addr
	 * @param addr The apartment's addr
	 * @return The linked list of the specific apt
	 */
	public AptLL getList(String city) {

		int hashIndex = generateHashIndex(city);		// Generates a hash index of the apt
		return aptArray[hashIndex];						// Returns a linked list
	}


	/**
	 * Generates a hash index by using information from an ApartmentInfo object
	 * @param apt The ApartmentInfo object to generate a hash index for
	 * @return A hash index
	 */
	private int generateHashIndex(ApartmentInfo apt) {

		return generateHashIndex( apt.getCity() );				// Returns hash index using the apt's city

	}


	/**
	 * Generates a hash index by using an apartment's city directly
	 * @param city The apartment's city
	 * @return A hash index
	 */
	private int generateHashIndex(String city) {

		if (city.length() < 7) {
			while (city.length() < 7) {
				city = city.concat(" ");
			}
		}
		else {
			city = city.substring(0, 7);
		}

		long hashSum = 0;

		for (int i = 0; i < city.length(); i++) {
			int ascii = (int)city.charAt(i);
			hashSum *= 256;
			hashSum += ascii;
		}
		return (int)(hashSum % 311);
	}


	/**
	 * Places an ApartmentInfo object into a linked list in the hash table
	 * @param apt The ApartmentInfo object
	 * @param index An index of the hash table
	 */
	private void addApt(ApartmentInfo apt, int index) {

		if (aptArray[index] == null) {					// If the object at the hash table index is null, creates a new linked list
			aptArray[index] = new AptLL();
		}

		AptLL list = aptArray[index];			// Gets the linked list at the index in the hash table
		list.add(apt);									// Adds the ApartmentInfo object to the linked list
		aptArray[index] = list;						// Saves the linked list back into the hash table
	}


	/**
	 * Removes an ApartmentInfo object from a linked list in the hash table
	 * @param apt
	 * @param index
	 */
	private void deleteApt(ApartmentInfo apt, int index) {

		String addr = apt.getTrieAddr();						// Gets the addr of the apt
		AptLL list = aptArray[index];			// Gets the linked list at the index in the hash table
		list.removeByAddr(addr);							// Removes the ApartmentInfo object in the linked list
		aptArray[index] = list;						// Saves the linked list back into the hash table
	}

}