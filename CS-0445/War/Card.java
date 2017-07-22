// CS 0445 Spring 2017
// Assignment 1
// Card class for Assigment 1. You must use this class as given in the implementation
// of your War card game. 
public class Card implements Comparable<Card>
{
	// These are enum types that make the suits and ranks of the cards more readable.  They
	// are also used for comparison of cards, as shown in equals and compareTo below.  You
	// can access the type values directly using the class, which you will need to do to
	// initialize your deck of cards.  See A1Help.java for a demo of this.
	public static enum Suits {Spades, Clubs, Diamonds, Hearts}
	public static enum Ranks {Two, Three, Four, Five, Six, Seven, Eight, Nine, Ten, Jack, Queen, King, Ace}
	
	private Suits suit;
	private Ranks rank;

	public Card(Suits s, Ranks r)
	{
		suit = s;
		rank = r;
	}

	// Equals is only true if both the suit and the rank of the cards match
	public boolean equals(Object rhs)
	{
		Card rside = (Card) rhs;
		if (suit == rside.suit && rank == rside.rank)
			return true;
		else
			return false;
	}

	public String toString()
	{
		return (rank + "-of-" + suit);
	}

	// Compare this object to another Card.  Cards of the same rank are
	// considered equal, regardless of the suit.  Note that this differs
	// from the equals method.  Java enum types are automatically Comparable,
	// based on the order in which the values are defined (smallest to largest).
	public int compareTo(Card rhs)
	{
		return rank.compareTo(rhs.rank);
	}
}