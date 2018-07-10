# Monte Carlo Pi

COE 1530 Threading

## Description

Pi.java is a simulation of a Monte Carlo algorithm that randomly throws a dart at a dartboard, creating a ratio 
of Pi depending on which quadrant the dart landed in. This program utilizes concurrency to allow each of these
dart throws to happen in psuedo-parallel, with the calculation of Pi occuring in the main thread once each of the worker
threads terminates its execution.

## Usage

To compile the source (.java) files, execute the following command:

```shell
$ javac Pi.java
```

To run the program, execute the following command:

```shell
$ java Pi <number_of_threads> <iterations>
```

On the command line, replace \<number_of_threads\> and \<iterations\> with integer values for the number of threads to
to be used in the calculation and the number of times to calculate Pi, respectivelly. <br />
  
NOTE: the calculated value of Pi approaches its actual value as the number of iterations --> ∞ 
 
To compare program execution time based on various thread counts or iterations, preface the command as follows:

#### Bash
```bash
$ time java Pi <number_of_threads> <iterations>
```
#### PowerShell
```PowerShell
> Measure-Command {java Pi <number_of_threads> <iterations>}
```

## File Details


* Pi.java -- Monte Carlo Pi Simulation, allows user to input number of threads and calculations
* threads.md -- Assignment description from professor

## Project Hierarchy

Driver:

* Pi.java

## License

Everything in this repo is distributed under the MIT License.

See LICENSE for more information.
