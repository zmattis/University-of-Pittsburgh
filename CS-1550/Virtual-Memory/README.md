# Virtual Memory

CS 1550 Assignment 3

## Description

The Virtual Memory simulator is a Java application that simulates 4 different paging algorithms with different frame numbers and trace files. The application runs through the memory references of the trace file and displays the action taken for each address.

## Algorithms

| Algorithm | Description                                                |
| --------- | ---------------------------------------------------------- |
| Opt       | optimal page replacement algorithm given perfect knowledge |
| Clock     | better implementation of the second-chance algorithm       |
| FIFO      | first-in, first-out                                        |
| NRU       | not recently used page using the R and D bits              |

## Page Table Actions

  1. hit
  2. page fault – no eviction
  3. page fault – evict clean
  4. page fault – evict dirty

## Usage

To compile the source (.java) file (and dependencies), execute the following command:

```PowerShell
> javac vmsim.java -Xlint
```

The -Xlint option is used to suppress unchecked cast warnings.

To run the simulator, execute the following command:

```PowerShell
> java vmsim –n <numframes> -a <opt|clock|fifo|nru> [-r <refresh>] <tracefile>
```

 - \<numframes\> - number of frames
 - \<opt|clock|fifo|nru\> - paging algorithm
 - \<refresh\> - nru refresh parameter
 - \<tracefile\> - file for memory references

## File Details

<dl>
  <dt>vmsim.java</dt>
  <dd>Virtual Memory simulator application</dd>
  <dt>PageTable.java</dt>
  <dd>Page Table used to index page table entries</dt>
  <dt>PTE.java</dt>
  <dd>Page Table Entry, with index, frame number, and additional bits</dd>
  <dt>VirtualMemory.java</dt>
  <dd>Page Replacement Algorithm implementations</dd>
  <dt>bzip.trace, gcc.trace, swim.trace</dt>
  <dd>sample memory address trace files</dd>
  <dt>virtual_memory_analysis.xlsx</dt>
  <dd>Page replacement algorithm data and graphs</dd>
  <dt>virtual_memory_analysis.tex</dt>
  <dd>Page replacement algorithm analysis LaTex file</dd>
  <dt>virtual_memory_analysis.pdf</dt>
  <dd>Analysis PDF generated from *.tex</dd>
  <dt>*.png</dt>
  <dd>Images for analysis document</dd>
  <dt>virtual_memory_description.pdf</dt>
  <dd>PDF description from professor</dd>
</dl>

## Project Hierarchy

Drivers
  - vmsim.java

Classes
  - PageTable.java
  - PTE.java
  - VirtualMemory.java

Data
  - bzip.trace
  - gcc.trace
  - swim.trace

## License

Everything in this repo is distributed under the MIT License.

See LICENSE for more information.
