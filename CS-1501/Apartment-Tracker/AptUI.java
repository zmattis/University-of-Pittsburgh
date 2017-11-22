/**
 * @author Zachary M. Mattis
 * CS 1501
 * Assignment 3
 * October 28, 2017
 *
 * This class functions as the UI for the AptTracker.java executable.
 * Methods of this class simply print out different messages to the
 * console screen.
 */

public class AptUI {

	/**
	 * Prints the welcome message
	 */
	public static void displayWelcome() {

		System.out.printf("\n"
				+ "================================================\n"
				+ "    Welcome to the Zeta Real Estate App!\n"
				+ "================================================\n\n");
	}


	/**
	 * Prints the main menu
	 */
	public static void displayMenu() {

		System.out.printf("\t\t-------- MAIN MENU --------\n"
				+ "Please select an option by entering the number next to the option name.\n"
				+ "\t1. Add a new apartment for consideration\n"
				+ "\t2. Update an existing apartment\n"
				+ "\t3. Remove an apartment from consideration\n"
				+ "\t4. Retrieve the lowest price apartment\n"
				+ "\t5. Retrieve the highest square mileage apartment\n"
				+ "\t6. Retrieve the lowest price apartment by city\n"
				+ "\t7. Retrieve the highest square mileage apartment by city\n"
				+ "\t8. Exit\n");
	}


	/**
	 * Prints the new apartment heading
	 */
	public static void displayAddApt() {

		System.out.printf("-------- ADD NEW APARTMENT --------\n");
	}


	/**
	 * Prints the update apt heading and submenu
	 */
	public static void displayUpdateApt() {

		System.out.printf("-------- UPDATE APARTMENT --------\n");
		System.out.printf("Please provide the street address, apartment number, and zip code.\n\n");
	}


	/**
	 * Prints the remove apartment heading
	 */
	public static void displayRemoveApt() {

		System.out.printf("-------- REMOVE APARTMENT --------\n");
		System.out.printf("Please provide the street address, apartment number, and zip code.\n\n");
	}


	/**
	 * Prints the the lowest price heading
	 */
	public static void displayLowestPrice() {

		System.out.printf("-------- LOWEST PRICED APARTMENT --------\n\n");
	}


	/**
	 * Prints the highest square footage heading
	 */
	public static void displayHighestSqFt() {

		System.out.printf("-------- HIGHEST SQUARE FOOTAGE APARTMENT --------\n\n");
	}


	/**
	 * Prints the find apartment heading
	 */
	public static void displayHighestSqFtByCity() {

		System.out.printf("-------- HIGHEST SQUARE FOOTAGE APARTMENT BY CITY --------\n");
	}

	/**
	 * Prints the find apartment heading
	 */
	public static void displayLowestPriceByCity() {

		System.out.printf("-------- LOWEST PRICED APARTMENT BY CITY --------\n");
	}


	/**
	 * Prints the exit message
	 */
	public static void displayExit() {

		System.out.printf("\nThank you for using the Zeta Real Estate App!\n"
				+ "Good-bye!\n");
	}
}
