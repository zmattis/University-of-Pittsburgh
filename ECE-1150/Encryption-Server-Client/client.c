/**
 * @author Zack Mattis
 * CoE 1150
 * Lab 2
 * November 22, 2017
 * 
 * This program is a client that sends input file data to a server 
 * that encryptes / decryptes the data using an XOR key. The client 
 * receives the data back and overwrites the input file
 * w/ the new server data.
 */ 

 /* Header Files */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

/* Macros */
#define PORT "1337" // the port client will be connecting to 
#define MAXDATASIZE 1000 // max number of bytes we can get at once 

/* Function Declaration */
char *parseFile(char *);		//parse input file data as byte stream
int establishConnection(char *);		//establish connection to server (command line arg 2)
void sendData(int, char *, char *);		// send data from input file to server
void *get_in_addr(struct sockaddr *);	// get IP addr from struct (IP v4 || IPv6)
void writeOutput(char *, char *);		// write data from server back to file


int main(int argc, char *argv[])
{
	int socket_fd, len;
	char *buff;
	
	
	// Usage
	if (argc != 3) {
	    fprintf(stderr,"usage: client hostname file.*\n");
	    exit(1);
	}
	
	buff = parseFile(argv[2]);
	
	socket_fd = establishConnection(argv[1]);
	
	sendData(socket_fd, buff, argv[2]);

	return EXIT_SUCCESS;
}

char *parseFile(char *file){
	FILE *fp;
	long len;
	char *buff;

	printf("file: \"%s\"\n", file);
	fp = fopen(file, "rb");  	// Open the file in binary mode
	if(!fp){
		perror("File does not exist");
		exit(EXIT_FAILURE);
	}
	fseek(fp, 0, SEEK_END);          // Jump to the end of the file
	len = ftell(fp);             // Get the current byte offset in the file
	rewind(fp);                      // Jump back to the beginning of the file

	buff = (char *)malloc((len+1)*sizeof(char)); // Enough memory for file + \0
	fread(buff, len, 1, fp); 			// Read in the entire file
	fclose(fp); 					// Close the file
	
	return buff;
}

// Establish conenction with server
int establishConnection(char *serv){
	int socket_fd;  
	struct addrinfo hints, *servinfo, *p;
	char s[INET_ADDRSTRLEN];
	int rv;
		
	// hint properties
	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	
	if ((rv = getaddrinfo(serv, PORT, &hints, &servinfo)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
		exit(EXIT_FAILURE);
	}

	// loop through all the results and connect to the first we can
	for(p = servinfo; p != NULL; p = p->ai_next) {
		if ((socket_fd = socket(p->ai_family, p->ai_socktype,
				p->ai_protocol)) == -1) {
			perror("client: socket");
			continue;
		}

		if (connect(socket_fd, p->ai_addr, p->ai_addrlen) == -1) {
			perror("client: connect");
			close(socket_fd);
			continue;
		}

		break;
	}

	// No connection established
	if (p == NULL) {
		fprintf(stderr, "client: failed to connect\n");
		exit(EXIT_FAILURE);
	}

	inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr), s, sizeof s);
	printf("client: connecting to %s\n", s);

	freeaddrinfo(servinfo); // all done with this structure
	
	return socket_fd;		//ret socket fd
	
	
}

// send data to server 
void sendData(int socket_fd, char* buff, char *file){
	int numbytes;
	int len = strlen(buff);
	
	printf("client: sending data to the server...\n");
	printf("\tSending: \"%.*s\"\n",len+1, buff);

	if ( send(socket_fd, buff, len+1, 0) == -1){
		perror("send");
		exit(EXIT_FAILURE);
	}

	printf("client: receiving data from the server\n");
	if ((numbytes = recv(socket_fd, buff, MAXDATASIZE-1, 0)) == -1) {
	    perror("recv");
	    exit(1);
	}
	printf("\tReceived: \"%.*s\"\n",len+1, buff);

	buff[numbytes] = '\0';

	writeOutput(buff, file);
	
	close(socket_fd);
	
}


// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
	if (sa->sa_family == AF_INET) {
		return &(((struct sockaddr_in*)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6*)sa)->sin6_addr);
}


// Write server data back into input file 
void writeOutput(char *buff, char *f){	
	FILE *fp;
	fp = fopen(f, "w");
	
	printf("client: writing server data back to file\n\n");
	if (fputs(buff, fp) == EOF ){
		printf("Failed to write server data to ouptut");
	}
	fclose(fp);
	
}
