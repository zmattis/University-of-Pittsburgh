/**
 * @author Zachary M. Mattis
 * CS 0449
 * Pi Linux Device Driver
 * July 26, 2017
 *
 * This C file is program that reads the digits of pi
 * from command line arguments via the /dev/pi device
 * defined in pi_driver.c.
 */

/* Libraries */
#include <stdio.h>
#include <stdlib.h>

const char *USAGE_INFO[] = {
  "Usage Info:",
  "pi_digits\n\tDisplays this usage info",
  "pi_digits START END\n\tDisplays the digits of pi from START position to END position",
  "\t\tEX: pi_digits 0 3 produces 3141"
};

const int USAGE_INFO_SIZE = sizeof(USAGE_INFO)/sizeof(char *);

int usage() {
  int i;
  for (i = 0; i < USAGE_INFO_SIZE; i++)
    printf("%s\n", USAGE_INFO[i]);
  return -1;
}

/*
 * ------------------------------------------------------------------------
 * Read n digits from Pi using START and END values and display to stdout
 * ------------------------------------------------------------------------
 */
int main (int argc, char *argv[]) {
  int start;
  int end;
  int length;
  FILE *piData;

  // Command Line Usage
  if ((argc <= 2) || (argc > 3)) {
    return usage();
  }

  else {
    start = atoi(argv[1]);    // Parse first argument to start
    end = atoi(argv[2]);      // Parse second argument to end

    // User input error check
    if (start < 0) { // Make sure start is greater than 0
      printf("You entered an incorrect start value! The start value must NOT be negative!\n");
      return 1;
    }
    else if (end < 0) { // Make sure end is greater than 0
      printf("You entered an incorrect end value! The end value must NOT be negative!\n");
      return 1;
    }
    else if (start > end) { // Make sure start is less than or equal to end
      printf("You entered an incorrect start or end value. End must be GREATER THAN or EQUAL TO start!\n");
      return 1;
    }

    length = (end - start) + 1; // Set length accordingly
    char pi_value[length + 1]; // Allocate a character array ( +1 for /0 )

    piData = fopen("/dev/pi", "r"); // Open the pi char device

    // Return 1 if the pi devices was empty or somehow bad
    if (piData == NULL) {
      return 1;
    }

    // Seek to the correct position
    // Note, this is just ONE WAY of changing *ppos
    fseek(piData, start, SEEK_SET);
    fread(pi_value, 1, length, piData); // Read the data. Uses the pi_read function
    fclose(piData); // Close the file

    pi_value[length] = '\0'; // Add terminating null character

    printf("%s\n", pi_value);     // Print to standard output

  }

  return 0;        // Return success
}
