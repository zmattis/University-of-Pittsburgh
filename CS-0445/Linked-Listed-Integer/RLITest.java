// CS 0445 Spring 2017
// Test program for your ReallyLongInt class -- for full credit you CANNOT MODIFY
// this code in ANY WAY.

// This program should execute without error and produce output identical to
// the output shown on the Web site.  If your output does not match mine, think
// carefully about what your operations are doing and trace them to find the
// problem.

// If your output does not match mine, or if you must change this file to get
// your code to work, you will lose credit, but you can still get PARTIAL
// credit for your work, so be sure to turn something in no matter what!

import java.util.*;

public class RLITest
{
	public static void main (String [] args)
	{
		ReallyLongInt R1 = new ReallyLongInt("123456789");
		ReallyLongInt R2 = new ReallyLongInt("987654321");
		System.out.println("R1 = " + R1.toString());
		System.out.println("R2 = " + R2.toString());
		System.out.println();
		
		// Testing add method
		ReallyLongInt R3 = R1.add(R2);
		System.out.println(R1 + " + " + R2 + " = " + R3);
		R1 = new ReallyLongInt("1");
		R2 = new ReallyLongInt("999999999999999");
		R3 = R1.add(R2);
		ReallyLongInt R4 = R2.add(R1);
		System.out.println(R1 + " + " + R2 + " = " + R3);
		System.out.println(R2 + " + " + R1 + " = " + R4);
		System.out.println();
		
		// Testing subtract method
		R1 = new ReallyLongInt("23456");
		R2 = new ReallyLongInt("4567");
		R3 = R1.subtract(R2);
		System.out.println(R1 + " - " + R2 + " = " + R3);
		R1 = new ReallyLongInt("1000000");
		R2 = new ReallyLongInt("1");
		R3 = R1.subtract(R2);
		System.out.println(R1 + " - " + R2 + " = " + R3);
		R1 = new ReallyLongInt("123456");
		R2 = new ReallyLongInt("123455");
		R3 = R1.subtract(R2);
		System.out.println(R1 + " - " + R2 + " = " + R3);
		R1 = new ReallyLongInt("1000");
		R2 = new ReallyLongInt("1001");
		try
		{
			R3 = R1.subtract(R2);
		}
		catch (ArithmeticException e)
		{
			System.out.println(e);
		}
		System.out.println();

		// Testing copy constructor
		ReallyLongInt R5 = new ReallyLongInt(R4);
		System.out.println("Copy of " + R4.toString() + " = " + R5.toString());
		System.out.println();
		
		// Testing compareTo
		ReallyLongInt [] C = new ReallyLongInt[4];
		C[0] = new ReallyLongInt("844444444444444");
		C[1] = new ReallyLongInt("744444444444444");
		C[2] = new ReallyLongInt("844444445444444");
		C[3] = new ReallyLongInt("4444");
		for (int i = 0; i < C.length; i++)
		{
			for (int j = 0; j < C.length; j++)
			{
				int ans = C[i].compareTo(C[j]);
				if (ans < 0)
					System.out.println(C[i] + " < " + C[j]);
				else if (ans > 0)
					System.out.println(C[i] + " > " + C[j]);
				else
					System.out.println(C[i] + " == " + C[j]);
			}
		}
		System.out.println();
		Arrays.sort(C);
		System.out.println("Here is the sorted array: ");
		for (ReallyLongInt R: C)
			System.out.println(R);
		System.out.println();

		// Testing equals
		R1 = new ReallyLongInt("12345678987654321");
		R2 = new ReallyLongInt("12345678987654321");
		R3 = new ReallyLongInt("12345678907654321");
		if (R1.equals(R2))
			System.out.println(R1 + " equals " + R2);
		if (!R1.equals(R3))
			System.out.println(R1 + " does not equal " + R3);
		System.out.println();

		// Testing shift operations
		R1 = new ReallyLongInt("1234567");
		System.out.println(R1);
		R1.multTenToThe(6);
		System.out.println(R1);
		R1.divTenToThe(8);
		System.out.println(R1);
		System.out.println();
		
	}
}