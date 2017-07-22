// CS 0445 Spring 2017 
// Zack Mattis 4020473
// Assignment 1 MultiDS<T> interface

import java.util.*;

public class MultiDS<T> implements PrimQ<T>, Reorder
{
	private final T[] array;
	private int count;
	private static final int DEFAULT_SIZE = 25;
	
	
	//Default Constructor -- Creates an array of size 25.
	public MultiDS()
	{
		this(DEFAULT_SIZE);
		count=0;
	}
	
	///Constructor for user desired size of array
	public MultiDS(int desiredCapacity)
	{
		@SuppressWarnings("unchecked")
		T[] tempArray = (T[])new Object[desiredCapacity]; //Unchecked cast
		array = tempArray;
		count=0;		
	}
	
	//PrimQ
	
	//Adds a new item to the PrimQ in the next available location
	public boolean addItem(T item)
	{
		if (!full()){
			array[count]=item;
			count++;
			return true;
		}
		
		else{
			return false;
		}
			
	}
	
	//Removes and returns the "oldest" item in the PrimQ (null if empty)
	public T removeItem()
	{
		T temp;
		if(count!=0){
			shiftLeft();
			temp = array[count-1];
			count--;
		}
		
		else
			return null;	

		return temp;
		
	}
	
	//Returns true if the array is full
	public boolean full()
	{
		return count >= array.length;
	}
	
	//Returns true if the array is empty
	public boolean empty()
	{
		return count<=0;
	}
	
	//Returns the size of the array
	public int size()
	{
		return count;
	}
	
	//Clears the array to default state
	public void clear()
	{
		for (int i=0;i<count;i++)
		{
			array[i]=null;
		}
		
		count=0;
		
	}
	
	//Reverses only the logical values of the array
	public void reverse()
	{
		for(int i = 0; i < (count / 2); i++)
		{
			T temp = array[i];
			array[i] = array[count-1-i];
			array[count-1-i] = temp;
		}
				
	}
	
	//Removes logical last item and puts it at the front
	public void shiftRight()
	{
		T temp = array[count-1];
		
		for (int i=count-1; i>0; i--)
		{
			array[i]=array[i-1];			
		}
		
		array[0]=temp;
		
	}
	
	//Removes the logical first item and puts it at the end
	public void shiftLeft()
	{
		T temp = array[0];
		
		for (int i=0; i<count-1; i++)
		{
			array[i]=array[i+1];			
		}
		
		array[count-1]=temp;
		
	}
	
	//Fisher-Yates Shuffling Algorithm
	public void shuffle()
	{
		Random r = new Random();
		
		for (int i=count-1;i>0;i--){
			int rnd = r.nextInt(i+1);
			T temp = array[rnd];
			array[rnd] = array[i];
			array[i] = temp;
		}
	}
	
	//Method to convert array contents to string
	//Preceding "contents" from Assig1A driver class
	public String toString()
	{
		String temp = "Contents:\n";
		
		for (int i=0;i<count;i++)
		{
			temp=temp + array[i].toString() + " ";
		}
		
		return temp;
		
	}
		
	
}