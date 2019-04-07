# Weather Client

ECE 1150 Lab 1

## Description

The weather client establishes a TCP network connection to "The Weather Underground" API through berkley sockets to a host, "api.wunderground.com", using my custom key. It gathers user input to select the desired location. The client then receives and parses the XML data to return the current weather conditions to the user.

## API

This application utilizes my personal WUNDERGROUND API key.

| API                  | Key              |
| -------------------- | ---------------- |
| api.wunderground.com | 715de8d316f47c2d |

## Usage

To compile the source (.c) files, execute the following command, using the flags -lsocket -lnsl to link the proper networking libraries:

```bash
$ gcc -o weather_client weather_client.c -lsocket -lnsl
```

To run the program, execute the compiled application:

```bash
$ ./weather_client
```

## File Details

<dl>
  <dt>weather_client.c</dt>
  <dd>Client application, connects to API and retrieves weather conditions</dd>
  <dt>weather_client_analysis.docx</dt>
  <dd>Analysis document of client running</dd>
  <dt>weather_client_description.pdf</dt>
  <dd>Project description from professor</dd>
</dl>

## Project Hierarchy

Driver
  - weather_client.c

## License

Everything in this repo is distributed under the MIT License.

See LICENSE for more information.
