/**
 * @author Zack Mattis
 * CoE 1150
 * Lab 1
 * September 27, 2017
 * 
 * This program is a user client that networks to "The Weather Underground" through a socket
 * to a host, "api.wunderground.com". It gathers user input to select the desired location.
 * It then parses the retrieved XML data to return the current weather conditions. This
 * program was successfully compiled using gcc on the machine: unxs.cis.pitt.edu 
 * using the following command: gcc -o weather_client weather_client.c -lsocket -lnsl
 * The flags -lsocket and -lnsl are used to properly link the corresponding libraries.
 */ 
 
/* Header Files */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* Macros */
#define PORT "80" // Port client connects to (HTTP)
#define INTERNET_HOST "api.wunderground.com"	// Internet host
#define KEY 715de8d316f47c2d		// W Underground Key
 
/* Function to Print Weather Conditions from Buffer */
void printWeather(char *buff){
	int len;
	char *str;
	
	/* Location */
	str = strstr(buff, "<full>");
	len = strcspn(str+6, "<");
	printf("\nLocation: %.*s\n", len, str+6);
	
	printf("\nCurrent Weather Conditions");
	
	/* Observation Time */
	str = strstr(buff, "<observation_time>");
	len = strcspn(str+18, "<");
	printf(" (%.*s):\n\n", len, str+18);
	
	
	/* Temp */
	str = strstr(buff, "<temperature_string>");
	len = strcspn(str+20, "<");
	printf("\tTemperature: %.*s", len, str+20);
	
	/* Weather */
	str = strstr(buff, "<weather>");
	len = strcspn(str+9, "<");
	printf("\n\tWeather: %.*s", len, str+9);

	/* Wind String */
	str = strstr(buff, "<wind_string>");
	len = strcspn(str+13, "<");
	printf("\n\tWind: %.*s", len, str+13);
	
	/* Humidity */
	str = strstr(buff, "<relative_humidity>");
	len = strcspn(str+19, "<");
	printf("\n\tHumidity: %.*s", len, str+19);
	
	
	printf("\n\n");
	
} 
 
 
 
int main(void){
	 
	 /* Initialize variables */
	 struct addrinfo hints;			//Define struct properties
	 struct addrinfo *wu_addr, *p;  // Address of Weather Underground
	 int socket_fd;    			// Socket File Descriptor
	 int size, i=0;				// size of data received
	 char *s, ch;				//char ptr for str
	 char host[64];				// Host IP address
	 char buff[4096];			//Buffer for xml (Requests receive ~3700 bytes)
	 char city[32], state[32];	//city and state buffers for user input
	 char request[128] = "GET /api/715de8d316f47c2d/conditions/q/";		//HTTP request string
	 
	 
	 printf("\n\nWelcome to The Weather Underground API Client!\n\t\tWritten by Zack Mattis\n\n");
	 
	 
	 /*Set Hint Properties */
	 memset(&hints, 0, sizeof(hints));
	 hints.ai_family = AF_INET;  //IPv4
	 hints.ai_socktype = SOCK_STREAM;	//Constant Stream
	 hints.ai_protocol = IPPROTO_TCP;	//TCP
	 
	 /* Domain Properties */
	 
	 if ( getaddrinfo(INTERNET_HOST, PORT, &hints, &wu_addr) != 0 ){
		 perror("Error: Failure connecting to API\n");
		 exit(EXIT_FAILURE);
	 }
	 else {
		 printf("Connecting to Weather Underground API...\n\n");
	 }
	 
	 /* Loop through all the results and connect to the first possible instance */
     for(p = wu_addr; p != NULL; p = p->ai_next) {
         if ((socket_fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
             perror("Error: Failure opening socket... Trying again\n");
             continue;
         }

         if (connect(socket_fd, p->ai_addr, p->ai_addrlen) == -1) {
            close(socket_fd);
            perror("Error: Failure connecting to server... Trying again\n");
            continue;
         }

         break;
     }

	 /* Connection cannot be established */
	 if (p==NULL){
		 perror("Error: Client failed to connect\n");
		 exit(EXIT_FAILURE);
	 }
	 
	 /* Convert to numbered IP address */
	 inet_ntop(p->ai_family, p->ai_addr, host, sizeof(host) );
     printf("Connection: Client connected to IP %s\n\n", host);
	 
	 freeaddrinfo(wu_addr);		//Free structure
	 
	 /* Generate HTTP Request String from user input */
	 printf("What city would you like to look up?\n\n");
	 printf("City: ");
	 fgets(city, 32, stdin);
	 city[strcspn(city, "\n")] = '\0';
	 printf("\nState(US, Abbreviated) or Country(International): ");
	 fgets(state, 32, stdin);
	 state[strcspn(state, "\n")] = '\0';
	 strcat(request, state);
	 strcat(request, "/");
	 strcat(request, city);
	 strcat(request, ".xml HTTP/1.1\r\nHost: api.wunderground.com\r\n\r\n" );
	 
	 
	 /* HTTP Request */
	 printf("\n\nSending HTTP Request String to Server...");
	 send(socket_fd, request, strlen(request), 0 );
	
	 /* Receive Data */
	 printf("\n\nReceiving Data from Server...");
	 recv(socket_fd, buff, 4095, 0 );
	 
	 close(socket_fd);		//close socket
	 
	 //printf("%s", buff);	//print xml from buff
	 
	 /* Parse Content Length */
	 s = strstr(buff, "Content-Length");
	 s = s+16;
	 while((ch = s[i] )!= '\r')
	{
		size = size*10 +(ch - '0');
		i++;
	}
	 
	 if (size<1000){
		 printf("\n\nERROR: Invalid user input");
		 printf("\nUSAGE: Pittsburgh, PA");
		 printf("\nUSAGE: Paris, France\n\n");
		 exit(EXIT_FAILURE);
	 }
 	 printf("\nReceived: %d bytes\n\n", size);

	 
	 printWeather(&buff[0]);
	 
	 return EXIT_SUCCESS;
	 
	 
}
