// CS 0445 Spring 2017
// Zack Mattis 4020473
// Assignment 1 War class

public class War
{
	public static void main(String[] args)
	{
		int numberOfTurns=Integer.parseInt(args[0]);
		int roundCount = 0;
		int cardCount = 0;
		System.out.print("Welcome to the Game of War!\n\n");

		//Create the Players hands and discard piles
		MultiDS<Card> Player0_hand = new MultiDS<Card>(52);
		MultiDS<Card> Player1_hand = new MultiDS<Card>(52);
		MultiDS<Card> Player0_discard = new MultiDS<Card>(52);
		MultiDS<Card> Player1_discard = new MultiDS<Card>(52);

		MultiDS<Card> initialDeck = new MultiDS<Card>(52);
		MultiDS<Card> warDeck = new MultiDS<Card>(52);

		//Create initial Deck with each type of card
		for (Card.Suits s : Card.Suits.values() )
		{
			for (Card.Ranks r : Card.Ranks.values() )
			{
				Card c = new Card(s,r);
				initialDeck.addItem(c);
			}
		}
		initialDeck.shuffle();


		//Equally divide shuffled hand between 2 players
		System.out.print("Now dealing the cards to the players...\n\n");
		for(int i=0;i<52;i++){
			if(i%2==1){
				Player0_hand.addItem(initialDeck.removeItem());
			}
			else{
				Player1_hand.addItem(initialDeck.removeItem());
			}
		}
		System.out.print("Here is Player 0's Hand:\n" + Player0_hand.toString() + "\n\n");
		System.out.print("Here is Player 1's Hand:\n" + Player1_hand.toString() + "\n\n");
		boolean gameContinue0=true, gameContinue1=true;

		//Move cards from discard to hand
		gameContinue0=discard2Hand(Player0_hand,Player0_discard,"0");
		//Move cards from discard to hand
		gameContinue1=discard2Hand(Player1_hand,Player1_discard,"1");

		//For Loop to run through the specified number of turns
		while (roundCount!=numberOfTurns && gameContinue0==true && gameContinue1==true)
		{
			cardCount=0;

			Card temp0=Player0_hand.removeItem();
			Card temp1=Player1_hand.removeItem();
			int result = temp0.compareTo(temp1);
			cardCount++;
			cardCount++;

			if (result==0){
				System.out.print("\tWAR: " + temp0 + " ties " + temp1 + "\n");
				warDeck.addItem(temp0);
				warDeck.addItem(temp1);

				while (result == 0)
				{
					//Add if statements to check for empty hands
					gameContinue0=discard2Hand(Player0_hand,Player0_discard,"0");
					gameContinue1=discard2Hand(Player1_hand,Player1_discard,"1");
					if (gameContinue0==false) {break;}
					if (gameContinue1==false) {break;}
					Card risk0 = Player0_hand.removeItem();
					Card risk1 = Player1_hand.removeItem();
					warDeck.addItem(risk0);
					warDeck.addItem(risk1);
					cardCount++;
					cardCount++;
					System.out.print("\tPlayer 0:" + risk0 + " and Player 1:" + risk1 + " are at risk!\n");
					gameContinue0=discard2Hand(Player0_hand,Player0_discard,"0");
					gameContinue1=discard2Hand(Player1_hand,Player1_discard,"1");
					if (gameContinue0==false) {break;}
					if (gameContinue1==false) {break;}

					// If statement for secondary war
					Card war0=Player0_hand.removeItem();
					Card war1=Player1_hand.removeItem();
					warDeck.addItem(war0);
					warDeck.addItem(war1);
					cardCount++;
					cardCount++;
					//temp0=war0;
					//temp1=war1;
					result = war0.compareTo(war1);

					if (result==0){
						System.out.print("\tWAR: " + war0 + " ties " + war1 + "\n");
					}

				}

			}

			if(result>0){
				System.out.print("Player 0 Wins Round " + roundCount + ": " + temp0 + " beats " + temp1 + " : " + cardCount + " cards\n");
				if (warDeck.empty()){
					Player0_discard.addItem(temp0);
					Player0_discard.addItem(temp1);
				}
				else {
					int wTemp=warDeck.size();
					for (int k=0; k<wTemp;k++){
						Player0_discard.addItem(warDeck.removeItem());
					}
				}
			}

			else if(result<0){
				System.out.print("Player 1 Wins Round " + roundCount + ": " + temp0 + " loses to " + temp1 + " : " + cardCount + " cards\n");
				if (warDeck.empty()){
					Player1_discard.addItem(temp0);
					Player1_discard.addItem(temp1);
				}
				else {
					int wTemp=warDeck.size();
					for (int k=0; k<wTemp;k++){
						Player1_discard.addItem(warDeck.removeItem());
					}
				}
			}

			warDeck.clear();
			roundCount++;

			//Move cards from discard to hand
			gameContinue0=discard2Hand(Player0_hand,Player0_discard,"0");
			//Move cards from discard to hand
			gameContinue1=discard2Hand(Player1_hand,Player1_discard,"1");
		}

		discard2Hand(Player0_hand,Player0_discard,"end");
		discard2Hand(Player1_hand,Player1_discard,"end");

		int final0 = Player0_hand.size()+Player0_discard.size();
		int final1 = Player1_hand.size()+Player1_discard.size();


		if (Player0_hand.empty()){
			System.out.print("\nPlayer 0 is out of cards!\n Player 1 is the WINNER!");
		}
		else if (Player1_hand.empty()){
			System.out.print("\nPlayer 1 is out of cards!\nPlayer 0 is the WINNER!");
		}
		else{
			System.out.print("\nAfter " + numberOfTurns + " rounds here is the status:\n\tPlayer 0 has " + final0 + " cards\n\tPlayer 1 has " + final1 + " cards\n");
			if (final0>final1){
				System.out.print("Player 0 is the WINNER!");
			}
			else if(final0<final1){
				System.out.print("Player 1 is the WINNER!");
			}
			else{
				System.out.print("It is a STALEMATE");
			}
		}

	}

	//Method that checks to see if players hand is empty and needs to be exchanged w/ discard
	//Returns false if there are no cards in hand or discard to symbolize end of game
	public static boolean discard2Hand(MultiDS<Card> hand, MultiDS<Card> discard, String t){
		if (hand.empty())
			{
				if (discard.size()==0){
					return false;
				}
				if(!discard.empty())
				{
					int temp = discard.size();
					for (int j=0;j<temp;j++)
					{
						hand.addItem(discard.removeItem());
					}
					if (!t.equals("end")) System.out.print("\tGetting and shuffling the pile for player " + t + "\n");
					hand.shuffle();
				}

			}
		return true;


	}

}
