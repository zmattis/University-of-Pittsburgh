# Encryption Server / Client

ECE 1150 Lab 2

## Description

The encryption/decryption server/client parses an input file into a buffer on the client side, which
is then transmitted to the server through an established TCP connection. The server takes the raw
byte stream and runs it through an XOR cipher algorithm which XORs each byte against a designated
character, which is 'Z' in this case. The server then sends the data back to the client, where
it is written back into the input file.

## Usage

To compile the source (.c) files, execute the following command:
Note: -lsocket -lnsl flags are used to link the proper networking libraries

* gcc -o server server.c -lsocket -lnsl
* gcc -o client client.c -lsocket -lnsl

To run the program, execute the following command:

* ./server
* ./client hostname file.*

While the server is running, execute the client by passing the server host name as the second argument and the input 
file (located in your working directory) as the third argument.

## File Details

* server.c -- server source code, accepts incoming connections
* client.c -- client source code, connects to server
* lab2_analysis -- analysis document of server/client
* lab2_description -- Project description from professor
* secret.txt -- example file for client
* out -- folder containing piped-output files from running server/client

## Project Hierarchy

Driver:

* server.c 
* client.c -- secret.txt

## License

Everything in this repo is distributed under the MIT License.

See LICENSE for more information.
