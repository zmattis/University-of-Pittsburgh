//COE 0445
//Assignment #3
//Recursive Backtracking
//Zack Mattis
//March 3, 2017

import java.io.*;
import java.util.*;

public class WordSearch
{
	public static void main(String [] args)
	{
		new WordSearch();
	}

	// Constructor to set things up and make the initial search call.
	public WordSearch()
	{
  	Scanner inScan = new Scanner(System.in);
		Scanner fReader;
		File fName;
        String fString = "", phrase = "";

       	// Make sure the file name is valid
        while (true)
        {
           try
           {
               System.out.println("Please enter grid filename:");
               fString = inScan.nextLine();
               fName = new File(fString);
               fReader = new Scanner(fName);

               break;
           }
           catch (IOException e)
           {
               System.out.println("Problem " + e);
           }
        }

		// Parse input file to create 2-d grid of characters
		String [] dims = (fReader.nextLine()).split(" ");
		int rows = Integer.parseInt(dims[0]);
		int cols = Integer.parseInt(dims[1]);


		char [][] theBoard = new char[rows][cols];

		for (int i = 0; i < rows; i++)
		{
			String rowString = fReader.nextLine();
			for (int j = 0; j < rowString.length(); j++)
			{
				theBoard[i][j] = Character.toLowerCase(rowString.charAt(j));
			}
		}

		System.out.println();
		// Show user the grid
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				System.out.print(theBoard[i][j] + " ");
			}
			System.out.println();
		}
		System.out.println();
		System.out.println("Please enter phrase (sep. by single spaces):");

		phrase = (inScan.nextLine()).toLowerCase();


		while (!(phrase.equals("")))
		{
			String[] words = phrase.split(" ");
			int wordNum = words.length;
			String[] index = new String[(words.length)*2];


			System.out.println("Looking for: " + phrase);
			System.out.println("containing " + wordNum + " words");
			// Search for the word.  Note the nested for loops here.  This allows us to
			// start the search at any of the locations in the board.  The search itself
			// is recursive (see findWord method for details).  Note also the boolean
			// which allows us to exit the loop before all of the positions have been
			// tried -- as soon as one solution has been found we can stop looking.
			boolean found = false;
			for (int r = 0; (r < rows && !found); r++)
			{
				for (int c = 0; (c < cols && !found); c++)
				{
				// Start search for each position at index 0 of the word
				found = findPhrase(r, c, words, 0, index, theBoard);

				}
			}

			if (found)
			{
				System.out.println("The phrase: " + phrase);
				System.out.println("was found:");
				for (int i=0; i<wordNum; i++){
					System.out.println(words[i] + ": " + index[i*2] + " to " + index[(i*2)+1]);
				}


				for (int i = 0; i < rows; i++)
				{
					for (int j = 0; j < cols; j++)
					{
						System.out.print(theBoard[i][j] + " ");
						theBoard[i][j] = Character.toLowerCase(theBoard[i][j]);
					}
					System.out.println();
				}
			}
			else
			{
				System.out.println("The phrase: " + phrase);
				System.out.println("was not found");

			}

			System.out.println("Please enter the phrase to search for:");
        	phrase = (inScan.nextLine()).toLowerCase();
		}

	}

	private boolean findPhrase(int r, int c, String[] words, int wordNum, String[] index, char [][] bo )
	{
		boolean myfound=false;
		for (int i=0; i<4; i++){
			myfound = findWord(r, c, words, 0, i, 0, index, bo );

			if (myfound){
				break;
			}
		}

		return myfound;

	}


	// Recursive method to search for the word.  Return true if found and false
	// otherwise.
	private boolean findWord(int r, int c, String[] words, int loc, int dir, int wordNum, String[] index,  char [][] bo)
	{

		// Check boundary conditions
		if(loc==0){
					index[(wordNum*2)]="(" + r + "," + c + ")";
				}
		if (r >= bo.length || r < 0 || c >= bo[0].length || c < 0){
			return false;
		}

		else if (bo[r][c] != words[wordNum].charAt(loc)){  // char does not match
			return false;
		}
		else { 											// current character matches
			bo[r][c] = Character.toUpperCase(bo[r][c]);  // Change it to
				// upper case.  This serves two purposes:
				// 1) It will no longer match a lower case char, so it will
				//    prevent the same letter from being used twice
				// 2) It will show the word on the board when displayed

			boolean answer=false;
			if ( (wordNum==(words.length-1)) && (loc == (words[words.length-1]).length()-1) ){		// base case - word found and we
				answer = true;																		// are done!
				index[(wordNum*2)+1]="(" + r + "," + c + ")";
			}

			else if (loc==words[wordNum].length()-1){									//End of word
				index[(wordNum*2)+1]="(" + r + "," + c + ")";
				if (!answer)
					answer = findWord(r, c+1, words, 0, 0, wordNum+1, index, bo);  // Right
				if (!answer)
					answer = findWord(r+1, c, words, 0, 1, wordNum+1, index, bo);  // Down
				if (!answer)
					answer = findWord(r, c-1, words, 0, 2, wordNum+1, index, bo);  // Left
				if (!answer)
					answer = findWord(r-1, c, words, 0, 3, wordNum+1, index, bo);  // Up

				if(!answer){
					bo[r][c]= Character.toLowerCase(bo[r][c]);
				}


			}


			else	// Still have more letters to match, so recurse.
			{		// Try all four directions if necessary (but only if necessary)
				if ( (!answer && dir==0) || (!answer && loc==0 && wordNum==0) )
					answer = findWord(r, c+1, words, loc+1, 0, wordNum, index, bo);  // Right
				if ( (!answer && dir==1) || (!answer && loc==0 && wordNum==0) )
					answer = findWord(r+1, c, words, loc+1, 1, wordNum, index, bo);  // Down
				if ( (!answer && dir==2) || (!answer && loc==0 && wordNum==0) )
					answer = findWord(r, c-1, words, loc+1, 2, wordNum, index, bo);  // Left
				if ( (!answer && dir==3) || (!answer && loc==0 && wordNum==0) )
					answer = findWord(r-1, c, words, loc+1, 3, wordNum, index, bo);  // Up

				// If answer was not found, backtrack.  Note that in order to
				// backtrack for this algorithm, we need to move back in the
				// board (r and c) and in the word index (loc) -- these are both
				// handled via the activation records, since after the current AR
				// is popped, we revert to the previous values of these variables.
				// However, we also need to explicitly change the character back
				// to lower case before backtracking.
				if (!answer){
					bo[r][c] = Character.toLowerCase(bo[r][c]);
				}
			}
			return answer;
		}
	}


}
