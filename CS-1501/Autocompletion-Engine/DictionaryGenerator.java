/**
 * @author Zack Mattis
 * CS 1501
 * Assignment 1
 * September 25, 2016
 * 
 * This class will create a de la Briandais trie and
 * populate it with words found in dictionary.txt.
 * It depends on dictionary.txt being in the same directory
 * and will display an exception if it is not found.
 */

import java.util.Scanner;
import java.io.*;

public class DictionaryGenerator {
	
	private static Scanner scan;			// Parses through dictionary.txt
	private static String word;				// A word from the text file
	private static DLBTrie<Character, String> dictionaryTrie;		// A trie containing dicitonary words
	
	
	public static DLBTrie<Character, String> newTrie() throws IOException {
		
		try {					// Attempts to open dictionary.txt for parsing
			File dictionary = new File("dictionary.txt");
			scan = new Scanner(dictionary);
		}
		
		catch (FileNotFoundException e) {			// Throws a file not found exception if it is missing
			System.out.printf("Error: Could not find \"dictionary.txt\".  Please check your directory for the text file.\n");
			System.exit(1);
		}
		
		dictionaryTrie = new DLBTrie<Character, String>();				// Initializes the dictionary trie
				
		while (scan.hasNextLine()) {				// Loops while dictionary.txt has a line to parse
			word = scan.nextLine();			// Pulls a word from dictionary.txt
			
			DLBMethods.addWord(dictionaryTrie, word);		// Adds the word to the dictionary trie
			
		}
		
		return dictionaryTrie;
		
	}
	

}
