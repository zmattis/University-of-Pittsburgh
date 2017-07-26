# Word Search

CS 0445 Assignment 3

## Description

Word Search allows you to search for custom phrases within a user-inputted board of letters, utilizing
a recursive, backtracking algorithm. Phrases can be separated by spaces, as the search will utilize
branching to find sequential words.

## Usage

To compile the source (.java) files, execute the following command:

* javac WordSearch.java

To run the program, execute the following command:

* java WordSearch

Within the program, enter the name of the file that contains both the parameters of the board size
and the board itself. See sample.txt as an example. After loading the board, simply enter the phrase
you wish to search for, or enter nothing to exit the program.

## File Details

* WordSearch.java -- Word Search parent program, capable of searching for entire phrases
* FindWord.java -- Backtracking child program, only capable of searching for words
* sample.txt -- example board for program, with 2 integers ( row col ) listed above the board
* sample2.txt -- additional example
* sample3.txt -- additional example
* a3description.pdf -- Project description from professor

## Project Hierarchy

Driver:

* WordSearch.java -- sample.txt

## License

Everything in this repo is distributed under the MIT License.

See LICENSE for more information.
