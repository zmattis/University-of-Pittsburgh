/**
 * @author Zachary M. Mattis
 * CS 1501
 * Assignment 3
 * October 28, 2017
 *
 * This class functions as an object that stores the relevant information
 * about an apartment when buying or selling one. It holds the apartment
 * street address, number, city, zip, price (rent), and square footage.
 * It also contains the methods to access or modify this information.
 *
 */
public class ApartmentInfo {

	private String streetAddr;
  private int number;
  private String city;
	private String zipCode;
	private double price;
	private int squareFootage;

	/**
	 * Creates a new ApartmentInfo object with the assigned attributes
	 * @param stAddr The street address
   * @param num The apartment number
	 * @param loc The apartment's city location
	 * @param zip The apartment's zpi code
	 * @param cost The cost of rent
	 * @param sqft The amount of square footage
	 */
	public ApartmentInfo(String stAddr, int num, String loc, String zip, double cost, int sqft) {

		setAddr(stAddr);		// The following methods set each of the apartment's attributes
		setNum(num);
		setCity(loc);
		setZip(zip);
		setCost(cost);
		setSqFt(sqft);
	}


	/**
	 * Sets the street address of the apartment
	 * @param stAddr The apartment's address
	 */
	public void setAddr(String stAddr) {

		streetAddr = stAddr;
	}


	/**
	 * Sets the apartment number
	 * @param num The apartment's number
	 */
	public void setNum(int num) {

		number = num;
	}


	/**
	 * Sets the location of the apartment
	 * @param loc The apartment's city
	 */
	public void setCity(String loc) {

		city = loc;
	}


	/**
	 * Sets the zip code of the apartment
	 * @param zip The apartment's zip code
	 */
	public void setZip(String zip) {

		zipCode = zip;
	}


	/**
	 * Sets the cost of the apartment
	 * @param cost The apartment's cost
	 */
	public void setCost(double cost) {

		price = cost;
	}


	/**
	 * Sets the square footage of the apartment
	 * @param sqft The apartment's square footage
	 */
	public void setSqFt(int sqft) {

		squareFootage = sqft;
	}


  /**
	 * Gets the street address of the apartment
	 * @return  The apartment's address
	 */
	public String getAddr() {

		return streetAddr;
	}


  /**
   * Gets the apartment number
   * @return The apartment's number
   */
  public int getNum() {

    return number;
  }


  /**
	 * Gets the location of the apartment
	 * @return The apartment's city
	 */
	public String getCity() {

		return city;
	}


  /**
	 * Gets the zip code of the apartment
	 * @return The apartment's zip code
	 */
	public String getZip() {

		return zipCode;
	}


  /**
	 * Gets the cost of the apartment
	 * @return The apartment's cost
	 */
	public double getCost() {

		return price;
	}


  /**
   * Gets the square footage of the apartment
   * @return The apartment's square footage
   */
  public int getSqFt() {

    return squareFootage;
  }

	/**
   * Gets the trie storage param of the apartment
   * @return The apartment's trie storage
   */
  public String getTrieAddr() {

    return streetAddr + Integer.toString(number) + zipCode;
  }


	/**
	 * Returns a string representation of the apartment
	 */
	public String toString() {

		String apt = getAddr() + " Apt " + getNum() + "\n" + getCity() + ", " + getZip();

		return apt;
	}

}
