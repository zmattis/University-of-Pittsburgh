# Encryption Server / Client

ECE 1150 Lab 2

## Description

The encryption/decryption server/client parses an input file into a buffer on the client side, which is then transmitted to the server through an established TCP connection. The server takes the raw byte stream and runs it through an XOR cipher algorithm which XORs each byte against a designated character, which is 'Z' in this case. The server then sends the data back to the client, where it is written back into the input file.

## Usage

To compile the source (.c) files, execute the following commands, using the flags -lsocket -lnsl to link the proper networking libraries:

```bash
$ gcc -o encryption_server encryption_server.c -lsocket -lnsl
$ gcc -o encryption_client encryption_client.c -lsocket -lnsl
```

To create the server, run the compiled executable:

```bash
$ ./encryption_server
```

While the server is running, execute the client by passing the server host name as the 2nd argument and the input file as the 3rd argument.

```bash
$ ./client <hostname> <file.*>
```

## File Details

<dl>
  <dt>encryption_server.c</dt>
  <dd>Server source code, accepts incoming connections</dd>
  <dt>encryption_client.c</dt>
  <dd>Client source code, connects to server</dd>
  <dt>encryption_server_client_analysis.docx</dt>
  <dd>Analysis document of server/client</dd>
  <dt>encryption_server_client_description.pdf</dt>
  <dd>Project description from professor</dd>
  <dt>secret.txt</dt>
  <dd>example file for client</dd>
  <dt>/out</dt>
  <dd>Folder containing piped-output files from running server/client pair</dd>
</dl>

## Project Hierarchy

Drivers
  - server.c
  - client.c

Data
  - secret.txt

## License

Everything in this repo is distributed under the MIT License.

See LICENSE for more information.
