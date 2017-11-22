/**
 * @author Zack Mattis
 * CS 1501
 * Assignment 1
 * September 6, 2017
 * 
 * This is the main class that is called to run through the programs
 * that generates autocompletion upon each successive character enter.
 * It utilizes a de la Briandais trie to implement both the dictionary
 * of words as well as the user history, which is stored in user_history.txt.
 * Users can select one of five autocompete options to select a word, or enter
 * a '$' to symbolize the end of a word. To end the program, the user enters '!'.
 */

import java.io.*;
import java.util.*;

public class AutoComplete {
	
	private static final int NUM_PREDICTIONS = 5;			//final integer for number of predictions
	
	private static Scanner scan, parse;						// Scan is for getting user input, Parse is for scanning text files
	private static File userFile;						// The user_history.txt File object
	private static FileWriter userHistory;					// Prints words to user_history.txt
	private static DLBTrie<Character, String> dictionaryTrie;			// Stores dictionary in a de la Briandais trie
	private static DLBTrie<Character, String> historyTrie;			// Stores user History in a de la Briandais trie
	private static String[] predictions;					// 5 Predictions for user
	private static StringBuilder workingWord;						//StringBuidler for working Word
	
	
	public static void main(String[] args) throws IOException {
		
		if (args.length == 0) {	 //Proper num of args
				
				
				String userInput, hWord, completeWord="";					//User input string
				scan = new Scanner(System.in);						//Scanner for user input
				int count=0;										//Num of iterations (Timing)
				double totalTime=0.0, sTime=0.0, fTime=0.0, dTime=0.0;			//Timing varaibles (Total, Start, Finish, difference)
				boolean fFlag=true, sFlag=false, wFlag=false;		//Flags for User io 
				predictions = new String[5];						//5 Predictions for user
				workingWord = new StringBuilder();

				
				dictionaryTrie = DictionaryGenerator.newTrie();
					
				
				File userFile = new File("user_history.txt"); //User History File object
				userFile.createNewFile();		//Create new file if it doesn't exist, else: do nothing if it does exist
				parse = new Scanner(userFile);	//Scan file
				userHistory = new FileWriter(userFile, true);
				
				historyTrie = new DLBTrie<Character, String>(); //Initialize history trie
				
				while (parse.hasNextLine()){	//Populate history trie
					hWord = parse.nextLine();
					DLBMethods.addWord(historyTrie, hWord);
				}
				
				
				
				do {
					if (fFlag){		//First time through loop
						System.out.printf("\nEnter your first character: ");
						fFlag = false;
					}
					else if(sFlag){		//2+ Words && first char
						System.out.printf("\nEnter first character of your next word: ");
					}
					else{		//Default case
						System.out.printf("\nEnter your next character: ");
					}
					
					userInput = scan.nextLine();			// Gets user input
					System.out.printf("\n");				// Newline for display formatting only
					
					workingWord.append(userInput);				//Append character to working string
					
					
					switch (userInput){			//Switch case for special user input values 
						case "1":
							completeWord = predictions[0];
							wFlag = true;
							sFlag = true;
							break;
						case "2":
							completeWord = predictions[1];
							wFlag = true;
							sFlag = true;
							break;
						case "3":
							completeWord = predictions[2];
							wFlag = true;
							sFlag = true;
							break;
						case "4":
							completeWord = predictions[3];
							wFlag = true;
							sFlag=true;
							break;
						case "5":
							completeWord = predictions[4];
							wFlag = true;
							sFlag=true;
							break;
						case "$":
							workingWord.setLength(workingWord.length()-1);
							completeWord = workingWord.toString();
							wFlag = true;
							sFlag=true;
							break;
						case "!":												
							wFlag = true;
							break;
						default:
							break;
		
					}
					
					if (!wFlag){ 		//Generate Predictions if word not completed / not finished
						
						sTime = System.nanoTime();
						predictionGen(workingWord);
						fTime = System.nanoTime();
						dTime = (fTime-sTime)/1000000000.0;
						
						System.out.println("(" + dTime + " s)\nPredictions:");
						
						//Print predicitons to std out
						for (int i=0; i<predictions.length; i++){
							if (predictions[i]!=null){
							System.out.print("(" + (i+1) + ") " + predictions[i] +"     " );
							}
						}
						
						
					}
					else if(wFlag && !userInput.equals("!") ){		//Display output on finished word
						System.out.printf("\n\tWORD COMPLETED:  " + completeWord + "\n\n");
						
						DLBMethods.addWord(historyTrie, completeWord);
						userHistory.write(completeWord + "\n");
						
						wFlag = false;				//Reset complete word Flag
						workingWord.setLength(0);	//Reset working word
					}
										
					totalTime += dTime;				// Keep total time elapsed
					count++;						// Increment count
				} while ( !userInput.equals("!") );
				
				userHistory.close();
				
				System.out.printf("Average time:  %f s\nBye!", (double)(totalTime/count) ); 
								
		}
		
		else{				// Case where there are too many args
				throw new IllegalArgumentException("Too many arguments were passed into the command line!  Please try again.");
		}
		
		System.exit(0);
	}

	
	/**
	 * Takes the user inputted character sequence using a StringBuilder and iterates through the nodes using those 
	 * characters to use as a prefix map. Calls method to search through node to find possible autocompletions,
	 * populated the predictions string array. 
	 * @param working Word
	 */
	private static void predictionGen(StringBuilder workingWord) {
		
		int curr = NUM_PREDICTIONS;
		boolean flag=false;
		DLBNode<Character, String> uNode = historyTrie.getRootNode();
		DLBNode<Character, String> dNode = dictionaryTrie.getRootNode();		
		
		
		for (int i=0; i<workingWord.length(); i++){
			while( !uNode.hasNoKey() && workingWord.charAt(i)!=uNode.getKey() && uNode.hasNextSibling() ){
				uNode=uNode.getNextSibling();
				//DeBug
				//System.out.println(uNode.getKey() + " " + workingWord.charAt(i) );
			}
			if (!uNode.hasNoKey() && uNode.getKey()==workingWord.charAt(i) && uNode.hasNextChild()){	
				uNode=uNode.getNextChild();
				if(i==workingWord.length()-1){
					flag=true;
				}
			}
		}
		
		if(!uNode.hasNoKey() && flag){
		curr = DLBMethods.findPredictions(uNode, curr, predictions);
		}
		
		
		
		flag=false;
		for (int i=0; i<workingWord.length(); i++){
			while( !dNode.hasNoKey() && workingWord.charAt(i)!=dNode.getKey() && dNode.hasNextSibling() ){
				dNode=dNode.getNextSibling();
				//DeBug
				//System.out.println(dNode.getKey() + " " + workingWord.charAt(i) );
			}
			if (!dNode.hasNoKey() && dNode.getKey()==workingWord.charAt(i) && dNode.hasNextChild()){	
				dNode=dNode.getNextChild();
				if(i==workingWord.length()-1){
					flag=true;
				}
			}
		}
		
		if(!dNode.hasNoKey() && flag){
		curr = DLBMethods.findPredictions(dNode, curr, predictions );
		}
			// If no predictions are found
		for (int i=0; i<curr; i++){
			predictions[5-curr+i]=null;
		}
		
		
				
	}
}
