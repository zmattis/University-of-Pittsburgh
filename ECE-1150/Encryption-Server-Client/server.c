/**
 * @author Zack Mattis
 * CoE 1150
 * Lab 2
 * November 22, 2017
 * 
 * This program is a server that accepts files as a byte stream and runs an XOR
 * encryption/decryption function on the data, before sending it back to the client.
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
#include <sys/wait.h>
#include <signal.h>

/* Macros */
#define PORT "1337"
#define BACKLOG 4
#define MAX_BUFFER_SIZE 10000


/* Function Declaration */
int establishConnnection(void);			//establish socket connection on port
void acceptConnection(int);				// accept new connections
void sigchld_handler(int);				// handle sigs
void *get_in_addr(struct sockaddr *);	// get IP addr from struct (IP v4 || IPv6)
void encryptDecrypt(char[]);			// XOR cipher algorithm


int main(void)
{
    int socket_fd;
	
	socket_fd = establishConnnection();
	acceptConnection(socket_fd);


    return 0;
}

// Establish connection through port
int establishConnnection(){
	
	struct addrinfo hints, *servinfo, *p;
	struct sigaction sa;
	int socket_fd;
	int rv, yes=1;
	
	memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE; // use my IP

    if ((rv = getaddrinfo(NULL, PORT, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    // loop through all the results and bind to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((socket_fd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) {
            perror("server: socket");
            continue;
        }

        if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &yes,
                sizeof(int)) == -1) {
            perror("setsockopt");
            exit(1);
        }

        if (bind(socket_fd, p->ai_addr, p->ai_addrlen) == -1) {
            close(socket_fd);
            perror("server: bind");
            continue;
        }

        break;
    }

    freeaddrinfo(servinfo); // all done with this structure

    if (p == NULL)  {
        fprintf(stderr, "server: failed to bind\n");
        exit(1);
    }

    if (listen(socket_fd, BACKLOG) == -1) {
        perror("listen");
        exit(1);
    }

    sa.sa_handler = sigchld_handler; // reap all dead processes
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    if (sigaction(SIGCHLD, &sa, NULL) == -1) {
        perror("sigaction");
        exit(1);
    }

    printf("server: waiting for connections...\n");
	return socket_fd;
	
}

//Accept new connections to server
void acceptConnection(int socket_fd){
	int new_fd;  // listen on socket_fd, new connection on new_fd
    struct sockaddr_storage their_addr; // connector's address information
    socklen_t sin_size;
	int numbytes, i;
    char s[INET6_ADDRSTRLEN];
	char *buff;
	
    while(1) {  // main accept() loop
        sin_size = sizeof their_addr;
        new_fd = accept(socket_fd, (struct sockaddr *)&their_addr, &sin_size);
        if (new_fd == -1) {
            continue;
        }
		
        inet_ntop(their_addr.ss_family,
            get_in_addr((struct sockaddr *)&their_addr),
            s, sizeof s);

        printf("server: got connection from\t%s\n", s);
		

        if (!fork()) { // this is the child process
            close(socket_fd); // child doesn't need the listener
			
			buff = (char *)malloc(MAX_BUFFER_SIZE * sizeof(char) );			//allocate maximum size of buffer
			if ( numbytes = recv(new_fd, buff, MAX_BUFFER_SIZE, 0 ) == -1){		//receive data from new connection from fd
				perror("receive");
			}
			
			// receive and modify data
			printf("\tReceived: \"%.*s\"\n", MAX_BUFFER_SIZE, buff);
			printf("server: running XOR cipher...\n");
			encryptDecrypt(buff);			//Modify data
			
			//send data back to client
			printf("\tSending: \"%.*s\"\n\n", MAX_BUFFER_SIZE, buff);
            if (send(new_fd, buff, strlen(buff), 0) == -1)			//send data back to client
                perror("send");
            close(new_fd);
            exit(0);
        }
        close(new_fd);  // parent doesn't need this
    }
	
	
}

//Handle Sigs
void sigchld_handler(int s)
{
    // waitpid() might overwrite errno, so we save and restore it:
    int saved_errno = errno;

    while(waitpid(-1, NULL, WNOHANG) > 0);

    errno = saved_errno;
}


// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
	if (sa->sa_family == AF_INET) {
		return &(((struct sockaddr_in*)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6*)sa)->sin6_addr);
}



// Simple XOR Cipher Function
void encryptDecrypt(char cip[])
{
    int i;
    char xorKey = 'Z';		//XOR key
 
    // calculate length of input string
    int len = strlen(cip);
 
    
    for ( i = 0; i < len; i++)			// perform XOR operation of key w/ every caracter in string
    {
        cip[i] = cip[i] ^ xorKey;  
    }
	
}


