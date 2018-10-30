# Weather Client

ECE 1150 Lab 1

## Description

The weather client establishes a TCP network connection to "The Weather Underground" API through 
berkley sockets to a host, "api.wunderground.com", using my custom key. It gathers user input to 
select the desired location. The client then receives and parses the XML data to return the current 
weather conditions to the user.

## Usage

To compile the source (.c) files, execute the following command:<br/>
Note: -lsocket -lnsl flags are used to link the proper networking libraries

```bash
gcc -o weather_client weather_client.c -lsocket -lnsl
```

To run the program, execute the following command:

```bash
./weather_client
```

## File Details

* weather_client.c -- client source code, connects to API and retrieves weather conditions
* lab1_analysis.docx -- analysis document of client running
* lab1_description.pdf -- Project description from professor


## Project Hierarchy

Driver:
 
* weather_client.c

## License

Everything in this repo is distributed under the MIT License.

See LICENSE for more information.
