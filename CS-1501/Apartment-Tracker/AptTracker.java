/**
 * @author Zachary M. Mattis
 * CS 1501
 * Assignment 3
 * November 1, 2017
 *
 * This class is the main executable that sorts apartments by their price
 * or square footage using a priority queue.  Methods include adding new
 * apartments, updating existing apartments, removing apartments from consideration,
 * and retrieving lowest priced / highest square footage apartments from the entire
 * database or by specific city.
 */

 import java.util.*;
 import java.util.regex.*;
 import java.lang.*;


 public class AptTracker{

   private static Heap aptPrices = new Heap(true);			// Creates a new min heap based on apt prices
   private static Heap aptSqFt = new Heap(false);      // Creates a new max heap based on sq ft
   private static AWayTrie pricesByAddr = new AWayTrie();     // Redirection structure for pq (prices)
   private static AWayTrie sqFtByAddr = new AWayTrie();      // Redirection structure for pq (sq ft)
   private static AptHashTable aptByCity = new AptHashTable();			// Creates a hash table for similar apt cities
   private static Scanner input = new Scanner(System.in);				// Scanner object for input
   private static String menuOption;								// Holds the user's input


   public static void main(String[] args) {

 		AptUI.displayWelcome();					// Prints the welcome message
 		AptUI.displayMenu();					// Prints the menu

 		do {					// Loops until the user enters the exit option
 			System.out.printf("Enter \"M\" to show the menu.\nSelect an option: ");
 			menuOption = input.nextLine();							// Gets input from the user

 			switch (menuOption) {
 				case "1":						// If the input is "1", adds a new apartment to the database
 					addNewApt();
 					break;
 				case "2":						// If the input is "2", updates an apartment in the database
 					updateApt();
 					break;
 				case "3":						// If the input is "3", removes an apartment from the database
 					removeApt();
 					break;
 				case "4":						// If the input is "4", gets the lowest priced apartment from the database
 					getLowestPriceApt();
 					break;
 				case "5":						// If the input is "5", gets the highest square footage apartment from the database
 					getHighestSqFtApt();
 					break;
 				case "6":          // If the input is "6", gets the lowest priceced apartment from a specific city
          getLowestSpecificApt();
          break;
 				case "7":						// If the input is "7", gets the highest square footage apartment from a specific city
 					getLowestSpecificApt();
 					break;
 				case "8":						// If the input is "8", exits the program
 					AptUI.displayExit();		// Prints the exit message
 					System.exit(0);
 				case "m":
 				case "M":						// If the input is "m" or "M", prints the menu
 					AptUI.displayMenu();
 					break;
 				default:								// Otherwise, an invalid command was entered
 					System.out.printf("The command entered could not be recognized.  Please enter a valid integer.\n");
 					break;
 			}
 		} while (true);

  }

  /**
	 * Adds a new apartment to the database
	 */
	private static void addNewApt() {

		AptUI.displayAddApt();				// Prints the add apt heading
		String aptSt, city, zip;
		double price;
		int aptNum, sqFt;
		boolean valid;				// A boolean representing if the input was valid or not

    // Street Address
		do {
			valid = true;

			System.out.printf("Enter the apartment street address: ");
			aptSt = input.nextLine();			// Gets the st addr from the user
      Pattern p = Pattern.compile("[ a-zA-z0-9]+"); // Valid user input
      Matcher m = p.matcher(aptSt);

			if ( !m.matches() ) {		// Checks for apt street addr characters
				System.out.printf("The entered apartment street address was invalid.  Please try again.\n");
				valid = false;					// If the input was invalid, the user will be prompted to re-enter the VIN
			}
		} while (!valid);					// Loops while the st addr is invalid

    // Apt number
		do {
			valid = true;

			System.out.printf("Enter the apartment number: ");
			String aptNumString = input.nextLine();					// Gets the make from the user

      try {
        aptNum = Integer.parseInt(aptNumString);
      }
      catch(NumberFormatException nfe) {
        System.out.printf("The entered apartment number was invalid.  Please try again.\n");
        aptNum=1;   //default
        valid = false;  // If the input was not a number, the user will be prompted to re-enter the apt num
      }

      if (aptNum < 0) {							// If the apt Num entered was negative
        System.out.printf("The entered apartment number cannot be negative. Please try again.\n");
        valid = false;    //try again
      }

		} while (!valid);					// Loops while the make is invalid

    // Apartment City
		do {
			valid = true;

			System.out.printf("Enter the city: ");
			city = input.nextLine();					// Gets the model from the user

			if (city.length() == 0) {					// Checks if the input is empty
				System.out.printf("The apartment's city cannot be left blank!  Please try again.\n");
				valid = false;				// If the input was empty, the user will be prompted to re-enter the city
			}
		} while (!valid);					// Loops while the city is invalid

    // Zip
    do {
  		System.out.printf("Enter the zip code of the apartment: ");
  		zip = input.nextLine();				// Gets the mileage from the user

  		if (zip.length() == 0) {							// Usr error
  			System.out.printf("The entered zip code is invalid. Please try again.\n");
  			valid = false;
  		}

    } while (!valid);

    // Price
    do{
      valid = true;

      System.out.printf("Enter the price: $");
  		String priceString = input.nextLine();				// Gets the price from the user

      try {
        price = Double.parseDouble(priceString);
      }
      catch (NumberFormatException nfe) {				// If the price was not entered as a double
        System.out.printf("The entered price was invalid. Please try again.\n");
        price = 0.0;
        valid = false;    //try again
      }

      if (price < 0.0) {							// If the price entered was negative
        System.out.printf("The entered price cannot be negative. Please try again.\n");
        valid = false;    //try again
      }

    } while(!valid);    //Loops while price is invalid

    // Sq Ft
    do {
		System.out.printf("Enter the square footage of the apartment as an integer: ");
		String sqFtString = input.nextLine();				// Gets the mileage from the user

		try {
			sqFt = Integer.parseInt(sqFtString);
		}
		catch (NumberFormatException nfe) {				// If the sqft was not entered as an integer
			System.out.printf("The entered apartment square footage was invalid. Please try again.\n");
      sqFt = 0;
      valid = false;
		}

		if (sqFt <= 0) {							// If the sq ft entered was negative
			System.out.printf("The entered apartment square footage cannot be negative. Please try again.\n");
			valid = false;
		}

  } while (!valid);

		ApartmentInfo newApt = new ApartmentInfo(aptSt, aptNum, city, zip, price, sqFt);		// Creates a new AptInfo object from the inputs
		aptPrices.addApt(newApt, pricesByAddr);						// Adds the apt to the prices heap and updates the prices trie
		aptSqFt.addApt(newApt, sqFtByAddr);					// Adds the apt to the sqft heap and updates the sqft trie
		aptByCity.insertApt(newApt);						// Adds the apt to a linked list in the hash table
		System.out.printf("The apartment \n\t%s, #%d\n\t%s, %s\n\tPrice: $%.2f\n\tSquare Footage: %d ft.\n was successfully added!\n\n", aptSt, aptNum, city, zip, price, sqFt);		// Prints a success message
	}

  /**
	 * Updates an existing apartment in the database
	 */
  public static void updateApt() {

    String aptStUpdate, aptNumUpdate, zipUpdate;
    int aptNumUp;

		AptUI.displayUpdateApt();				// Prints the update apt submenu

    //Street address
    System.out.printf("Enter the street address: ");
    aptStUpdate = input.nextLine();			// Gets the price from the user

    // Apartment Number
    System.out.printf("Enter the apartment number: ");
    aptNumUpdate = input.nextLine();			// Gets the apt num from the user
    try {
      aptNumUp= Integer.parseInt(aptNumUpdate);
    }
    catch(NumberFormatException nfe) {
      System.out.printf("The entered apartment number was invalid. Defaulting to 1.\n");
      aptNumUp=1;   //default
    }

    System.out.printf("Enter the zip code: ");
    zipUpdate = input.nextLine();			// Gets the zip from the user

		int pricesHeap = pricesByAddr.getHeapIndex(aptStUpdate + aptNumUpdate + zipUpdate);				// Gets the index of the apt in the prices heap
		int sqFtHeap = sqFtByAddr.getHeapIndex(aptStUpdate + aptNumUpdate + zipUpdate);				// Gets the index of the apt in the sq ft heap

		if (pricesHeap < 0 || sqFtHeap < 0) {				// If either of the indices are less than 0, returns to the main menu
			System.out.printf("The Apartment with the Address %s %d, %s does not exist in the database!  Returning to the main menu.\n\n", aptStUpdate, aptNumUp, zipUpdate);
			return;
		}

    boolean flag;
    double price;

    do{
      flag = true;

  		System.out.printf("Enter the apartment's updated price: $");
  		String priceString = input.nextLine();			// Gets the price from the user

  		try {
  			price = Double.parseDouble(priceString);
  		}
  		catch (NumberFormatException nfe) {				// Defaults to 0.0 if the price entered is invalid
  			System.out.printf("The entered price was invalid. Please enter a valid price.\n");
  			price = 0.0;
        flag=false;
      }

      if (price <= 0) {							// If the price entered was negative
        System.out.printf("The apartment price cannot be negative. Please try again.\n");
        flag = false;
      }

    }while (!flag);

		ApartmentInfo apt = aptPrices.getApt(pricesHeap);			// Gets the AptInfo object from the prices heap
		apt.setCost(price);							// Updates the price of the apt
		aptPrices.setApt(apt, pricesHeap);				// Stores the apt in the prices heap
		aptPrices.checkAptUpdates(pricesHeap, pricesByAddr);			// Repositions the updated apt in the prices heap, if necessary
		aptSqFt.setApt(apt, sqFtHeap);				// Stores the apt in the sqft heap (no repositioning necessary)
		aptByCity.updatePrice(apt, price);				// Updates the apt in the hash table

    System.out.printf("The price of apartment %s #%d, %s was successfully updated to %f.\n\n", aptStUpdate, aptNumUp, zipUpdate, price);

	}



  /**
	 * Removes an apt from the database
	 */
	private static void removeApt() {

    String aptStUpdate, aptNumUpdate, zipUpdate;
    int aptNumUp;

		AptUI.displayRemoveApt();				// Prints the remove apt heading

    //Street address
    System.out.printf("Enter the street address: ");
    aptStUpdate = input.nextLine();			// Gets the price from the user

    // Apartment Number
    System.out.printf("Enter the apartment number: ");
    aptNumUpdate = input.nextLine();			// Gets the apt num from the user
    try {
      aptNumUp= Integer.parseInt(aptNumUpdate);
    }
    catch(NumberFormatException nfe) {
      System.out.printf("The entered apartment number was invalid. Defaulting to 1.\n");
      aptNumUp=1;   //default
    }

    System.out.printf("Enter the zip code: ");
    zipUpdate = input.nextLine();			// Gets the zip from the user

    int pricesHeap = pricesByAddr.getHeapIndex(aptStUpdate + aptNumUpdate + zipUpdate);				// Gets the index of the apt in the prices heap
    int sqFtHeap = sqFtByAddr.getHeapIndex(aptStUpdate + aptNumUpdate + zipUpdate);				// Gets the index of the apt in the sq ft heap

    if (pricesHeap < 0 || sqFtHeap < 0) {				// If either of the indices are less than 0, returns to the main menu
      System.out.printf("The Apartment with the Address %s %d, %s does not exist in the database!  Returning to the main menu.\n\n", aptStUpdate, aptNumUp, zipUpdate);
      return;
    }

		ApartmentInfo apt = aptPrices.getApt(pricesHeap);				// Gets the AptInfo object from the prices heap
		pricesByAddr.removeHeapIndex(apt.getTrieAddr());						// Removes the AptInfo object from the prices heap
		aptPrices.removeApt(pricesHeap, pricesByAddr);			// Removes the apt's heap index from the prices trie

		sqFtByAddr.removeHeapIndex(apt.getTrieAddr());						// Removes the AptInfo object from the mileage heap
		aptSqFt.removeApt(sqFtHeap, sqFtByAddr);			// Removes the apt's heap index from the mileage trie
    aptByCity.removeApt(apt);								// Removes the apt from its linked list in the hash table

    System.out.printf("The apartment %s #%d, %s was removed from consideration.\n\n", aptStUpdate, aptNumUp, zipUpdate);
	}

  /**
    * Gets the lowest priced Apartment from the database
    */
  private static void getLowestPriceApt() {

    AptUI.displayLowestPrice();						// Prints the lowest price heading
    ApartmentInfo apt = aptPrices.getMinPrice();			// Gets the highest priority AptInfo object from the prices heap

    if (apt == null) {				// If the apt is null, notifies the user of an empty database
      System.out.printf("There are no apartments in the database!  Returning to the main menu.\n\n");
    }
    else {
      System.out.printf("Lowest priced apartment listing:\n");
      System.out.printf("\t%s, #%d\n\t%s, %s\n\tPrice: $%.2f\n\tSquare Footage: %d ft.\n\n", apt.getAddr(), apt.getNum(), apt.getCity(), apt.getZip(), apt.getCost(), apt.getSqFt() );		// Prints a success message

    }

  }

  /**
    * Gets the highest square foot Apartment from the database
    */
  private static void getHighestSqFtApt() {

    AptUI.displayHighestSqFt();						// Prints the highest sq ft heading
    ApartmentInfo apt = aptSqFt.getMaxSqFt();			// Gets the highest priority AptInfo object from the sq ft heap

    if (apt == null) {				// If the apt is null, notifies the user of an empty database
      System.out.printf("There are no apartments in the database!  Returning to the main menu.\n\n");
    }
    else {
      System.out.printf("Highest square footage apartment listing:\n");
      System.out.printf("\t%s, #%d\n\t%s, %s\n\tPrice: $%.2f\n\tSquare Footage: %d ft.\n\n", apt.getAddr(), apt.getNum(), apt.getCity(), apt.getZip(), apt.getCost(), apt.getSqFt() );		// Prints a success message

    }

  }

  /**
 * Gets the lowest priced or highest sq ft apt of a specific city
 */
private static void getLowestSpecificApt() {

  if ( menuOption.equals("6") ){
    AptUI.displayLowestPriceByCity();
  }
  else{
    AptUI.displayHighestSqFtByCity();
  }


  System.out.printf("Enter the city of the apartments: ");
  String city = input.nextLine();					// Gets the make from the user


  if (city.length() == 0) {
    System.out.printf("Invalid city!  Returning to the main menu.\n\n");
    return;
  }

  AptLL list = aptByCity.getList(city);			// Gets the linked list of the specific apt's city

  if (list == null) {						// If the linked list is null, returns to the main menu
    System.out.printf("The city you entered does not have any apartments in the database.  Returning to the main menu.\n\n");
    return;
  }
  else {
    LLNode<ApartmentInfo> node = list.getListHead();			// Gets the linked list head
    CityHeap tempHeap;					// Declares a temporary, simplified heap

    if (menuOption.compareTo("6") == 0) {			// If the menu option is "6", creates a simplified heap based on price (min)
      tempHeap = new CityHeap(true);
    }
    else {											// Otherwise, creates a simplified heap based on sq ft (max)
      tempHeap = new CityHeap(false);
    }

    if (node == null || !node.hasData()) {				// Returns to the main menu if the node is null or does not have data
      System.out.printf("The apt you entered was not found in the database.  Returning to the main menu.\n\n");
      return;
    }

    while (node.hasData()) {						// Traverses through the entire linked list and adds apts to the heap
      tempHeap.addApt(node.getData());
      node = node.getNextNode();
    }

    ApartmentInfo apt;
    if ( menuOption.equals("6") ){
      System.out.printf("Lowest Price Apartment in %s:\n", city);
      apt =tempHeap.getMinPrice();
      System.out.printf("\t%s, #%d\n\t%s, %s\n\tPrice: $%.2f\n\tSquare Footage: %d ft.\n\n", apt.getAddr(), apt.getNum(), apt.getCity(), apt.getZip(), apt.getCost(), apt.getSqFt() );		// Prints a success message
    }
    else{
      System.out.printf("Highest Square Footage Apartment in %s:\n", city);
      apt = tempHeap.getMaxSqFt();
      System.out.printf("\t%s, #%d\n\t%s, %s\n\tPrice: $%.2f\n\tSquare Footage: %d ft.\n\n", apt.getAddr(), apt.getNum(), apt.getCity(), apt.getZip(), apt.getCost(), apt.getSqFt() );		// Prints a success message
    }

  }

}




}
