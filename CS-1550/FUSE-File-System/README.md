# FUSE File System

CS 1550 Assignment 4

## Description

[FUSE](https://fuse.sourceforge.net/) (Filesystem in Userspace)  is a Linux kernel extension that allows for a user space program to provide the implementations for the various file-related syscalls. This application is a simple 2-tiered filesystem where the root (/) directory only contains subdirectories, and each subdirectory only contains files. This implementation uses a 16-bit File Allocation Table (FAT).

## Linux Syscalls

| Syscall  | Description                                                  |
| -------- | ------------------------------------------------------------ |
| stat     | Checks existence of file                                     |
| mkdir    | Create a new directory                                       |
| readdir  | Return directory entries                                     |
| rmdir    | Remove the given directory                                   |
| mknod    | Create a new file                                            |
| write    | Write bytes into given file                                  |
| read     | Read bytes from given file                                   |
| unlink   | Deletes a file                                               |
| truncate | Remove bytes from end of file                                |
| open     | Open a file                                                  |
| flush    | Called on close to allow filesystem to report delayed errors |

## Usage

#### FUSE Installation

FUSE consists of two major components: A kernel module that has already been installed, and a set of libraries and example programs that you need to install. Download the FUSE *.tar.gz file.

To unpack the compressed file:

```bash
$ tar xvfz fuse-2.7.0.tar.gz
$ cd fuse-2.7.0
```

To install:

```bash
$ ./configure
$ make
$ make install
```

#### Disk Management

In order to manage the free or empty space, you will need to create bookkeeping block(s) in .disk that records what blocks have been previously allocated or not.

To create a 5MB disk image, execute the following:

```bash
$ dd bs=1K count=5K if=/dev/zero of=.disk
```

#### FUSE Filesystem Compilation

To compile the source (.c) file, execute the following command:

```bash
$ gcc -o fuse_file_system fuse_file_system.c
```

#### FUSE Application

To create your filesystem mount, create a new directory:

```bash
$ mkdir fuse_mount
```

Run the application on the new mount directory:

```bash
$ ./fuse_file_system fuse_mount
```

You can now create new directories and files within the given FUSE mounted filesystem via Linux systemcalls. These syscalls' implementation is now given by the user-space fuse_file_system.c application.

```bash
$ cd fuse_mount
$ mkdir edu
$ cd edu
$ mknod application.c
$ ls -l
```

To unmount the filesystem:

```bash
$ fusermount -u fuse_mount
```

## File Details

<dl>
  <dt>fuse_file_system.c</dt>
  <dd>User-space application implemented filesystem syscalls</dd>
  <dt>fuse-2.7.0.tar.gz</dt>
  <dd>FUSE application source</dd>
  <dt>fuse_file_system_description.pdf</dt>
  <dd>PDF description from professor</dd>
</dl>

## Project Hierarchy

Drivers
  - fuse_file_system.c

FUSE
  - fuse-2.7.0.tar.gz

## License

Everything in this repo is distributed under the MIT License.

See LICENSE for more information.
