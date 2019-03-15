/**
 * @author Zachary M. Mattis
 * COE 1550
 * FUSE File System
 * July 28, 2018
 *
 * This C file is a FUSE application that provides
 * support for a Linux filesystem implemenation
 * in User Space using a File Allocation Table.
 *
 * Provides implementations of following Unix calls:
 *
 *     stat (getattr)
 *     mkdir
 *     readdir
 *     rmdir
 *     mknod
 * 	   write
 *     read
 * 	   unlink
 * 	   truncate
 * 	   open
 * 	   flush
 *
 *	FUSE: Filesystem in Userspace
 *	Copyright (C) 2001-2007  Miklos Szeredi <miklos@szeredi.hu>
 *
 *	This program can be distributed under the terms of the GNU GPL.
 *	See the file COPYING.
 */

#define	FUSE_USE_VERSION 26

#include <fuse.h>
#include <stdio.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

//size of a disk block
#define	BLOCK_SIZE 512

//we'll use 8.3 filenames
#define	MAX_FILENAME 	8
#define	MAX_EXTENSION 	3

//Disk block sections
#define DISK_ROOT 	0			//Root directory
#define DISK_FAT 	  1			//File Allocation Table
#define DISK_START	2			//Start for FAT table

//How many files can there be in one directory?
#define MAX_FILES_IN_DIR (BLOCK_SIZE - sizeof(int)) / ((MAX_FILENAME + 1) + (MAX_EXTENSION + 1) + sizeof(size_t) + sizeof(long))

//The attribute packed means to not align these things
struct fuse_directory_entry
{
  int nFiles;	//How many files are in this directory.
              //Needs to be less than MAX_FILES_IN_DIR

  struct fuse_file_directory
  {
    char fname[MAX_FILENAME + 1];	//filename (plus space for nul)
    char fext[MAX_EXTENSION + 1];	//extension (plus space for nul)
    size_t fsize;					  //file size
    long nStartBlock;				//where the first block is on disk
  } __attribute__((packed)) files[MAX_FILES_IN_DIR];	//There is an array of these

  //This is some space to get this to be exactly the size of the disk block.
  //Don't use it for anything.
  char padding[BLOCK_SIZE - MAX_FILES_IN_DIR * sizeof(struct fuse_file_directory) - sizeof(int)];
} ;

typedef struct fuse_root_directory fuse_root_directory;

#define MAX_DIRS_IN_ROOT (BLOCK_SIZE - sizeof(int)) / ((MAX_FILENAME + 1) + sizeof(long))

struct fuse_root_directory
{
  int nDirectories;	//How many subdirectories are in the root
                    //Needs to be less than MAX_DIRS_IN_ROOT
  struct fuse_directory
  {
    char dname[MAX_FILENAME + 1];	//directory name (plus space for nul)
    long nStartBlock;				//where the directory block is on disk
  } __attribute__((packed)) directories[MAX_DIRS_IN_ROOT];	//There is an array of these

  //This is some space to get this to be exactly the size of the disk block.
  //Don't use it for anything.
  char padding[BLOCK_SIZE - MAX_DIRS_IN_ROOT * sizeof(struct fuse_directory) - sizeof(int)];
} ;

typedef struct fuse_directory_entry fuse_directory_entry;

//How much data can one block hold?
#define	MAX_DATA_IN_BLOCK (BLOCK_SIZE)

struct fuse_disk_block
{
  //All of the space in the block can be used for actual data
  //storage.
  char data[MAX_DATA_IN_BLOCK];
};

typedef struct fuse_disk_block fuse_disk_block;

#define MAX_FAT_ENTRIES (BLOCK_SIZE/sizeof(short))

struct fuse_file_allocation_table {
  short tbl[MAX_FAT_ENTRIES];		//OxFFFF = EOF
};

//Function prototypes
fuse_root_directory get_root(void);
struct fuse_file_allocation_table get_fat(void);
void set_root(fuse_root_directory*);
void set_fat(struct fuse_file_allocation_table*);

/**
 * Gets the root directory of the filesystem
 * @return root		Struct contating root directory
 */
fuse_root_directory get_root(void) {
  static struct fuse_root_directory root;
  FILE *disk = fopen(".disk", "r+b");
  fseek(disk, 0, SEEK_SET);
  fread(&root, BLOCK_SIZE, 1, disk);
  fclose(disk);
  return root;
}

/**
 * Gets the file allocation table of the filesystem
 * @return fat		Struct contating file allocation table
 */
struct fuse_file_allocation_table get_fat(void) {
  static struct fuse_file_allocation_table fat;
  FILE *disk = fopen(".disk", "r+b");
  fseek(disk, BLOCK_SIZE, SEEK_SET);
  fread(&fat, BLOCK_SIZE, 1, disk);
  fclose(disk);
  return fat;
}

/**
 * Sets the root directory of the filesystem
 * @param root		Struct containing new root directory
 */
void set_root(fuse_root_directory *root) {
  FILE* disk = fopen(".disk", "r+b");
  fseek(disk, 0, SEEK_SET);
  fwrite(root, BLOCK_SIZE, 1, disk);
  fclose(disk);
}

/**
 * Sets the file allocation table of the filesystem
 * @param fat		Struct containing file allocation table
 */
void set_fat(struct fuse_file_allocation_table *fat) {
  FILE* disk = fopen(".disk", "r+b");
  fseek(disk, BLOCK_SIZE, SEEK_SET);
  fwrite(fat, BLOCK_SIZE, 1, disk);
  fclose(disk);
}


/*
 * Called whenever the system wants to know the file attributes, including
 * simply whether the file exists or not.
 *
 * man -s 2 stat will show the fields of a stat structure
 *
 * @param path		Relative path to file or directory
 * @param stbuf		Modified structure containing information about the file/dir
 * @return			Integer representing success of operation
 * 						0 				Success
 * 						-ENOENT			File not found
 * 						-ENAMETOOLONG 	Not following 8.3 naming convention
 *
 */
static int fuse_getattr(const char *path, struct stat *stbuf)
{
  int res = 0;

  //Store the path data for easy navigation later on
  char directory[MAX_FILENAME+1];
  char filename[MAX_FILENAME+1];
  char extension[MAX_EXTENSION+1];
  strcpy(directory, "");
  strcpy(filename, "");
  strcpy(extension, "");

  if(strlen(path) != 1){ //If the path is not just "/", parse the necessary directory, filename, and extension if possible
    sscanf(path, "/%[^/]/%[^.].%s", directory, filename, extension);
  }

  //If any of the variables are longer than the 8.3 file naming convention, return the ENAMETOOLONG error
  if(strlen(directory) > MAX_FILENAME || strlen(filename) > MAX_FILENAME || strlen(extension) > MAX_EXTENSION){
    return -ENAMETOOLONG;
  }

  //Clear the data in stbuf JUST IN CASE
  memset(stbuf, 0, sizeof(struct stat));

  //The path is just the root directory
  if(strcmp(path, "/") == 0){
    stbuf->st_mode = S_IFDIR | 0755;
    stbuf->st_nlink = 2;
    res = 0;
    return res;
  } else{ //Navigate through the file system to find the correct directory and/or file
    if(strcmp(directory, "") == 0){ //If the directory is empty, return that the file cannot be found
      res = -ENOENT;
      return res;
    } else{
      int i = 0;
      struct fuse_directory dir;
      strcpy(dir.dname, "");
      dir.nStartBlock = -1;

      fuse_root_directory root = get_root();

      for(i = 0; i < root.nDirectories; i++){ //Find the subdirectory in the root's array of directories
        struct fuse_directory curr_dir = root.directories[i];
        if(strcmp(curr_dir.dname, directory) == 0){ //This current directory and our search directory names match
          dir = curr_dir;
          break;
        }
      }

      if(strcmp(dir.dname, "") == 0){ //No directory was found, so return an ENOENT error
        res = -ENOENT;
        return res;
      }

      if(strcmp(filename, "") == 0){ //No more left in the path to traverse; the user was only looking to get the attributes of a directory
        res = 0;
        stbuf->st_mode = S_IFDIR | 0755;
        stbuf->st_nlink = 2;
        return res; //Return a success
      }

      FILE* disk = fopen(".disk", "r+b");
      int location_on_disk = BLOCK_SIZE*dir.nStartBlock;
      fseek(disk, location_on_disk, SEEK_SET);

      fuse_directory_entry dir_entry;
      dir_entry.nFiles = 0;
      memset(dir_entry.files, 0, MAX_FILES_IN_DIR*sizeof(struct fuse_file_directory));

      int num_items_successfully_read = fread(&dir_entry, BLOCK_SIZE, 1, disk); //Read in the directory's data, such as its files contained within
      fclose(disk);

      if(num_items_successfully_read == 1){ //One block was successfully read, so proceed
        struct fuse_file_directory file;
        strcpy(file.fname, "");
        strcpy(file.fext, "");
        file.fsize = 0;
        file.nStartBlock = -1;

        int i = 0;
        for(i = 0; i < MAX_FILES_IN_DIR; i++){ //Iterate over the files in the directory
          struct fuse_file_directory curr_file = dir_entry.files[i];
          if(strcmp(curr_file.fname, filename) == 0 && strcmp(curr_file.fext, extension) == 0){ //Both the current filename and file extension match
                                //the filename and file extension we're looking for.
            file = curr_file;
            break;
          }
        }

        if((file.nStartBlock) == -1){ //No file was found, so return a file not found error
          res = -ENOENT;
          return res;
        } else{ //The file we were looking for was found!
          res = 0;
          stbuf->st_mode = S_IFREG | 0666;
          stbuf->st_nlink = 1;
          stbuf->st_size = file.fsize;
          return res; //Return success
        }
      }
    }
  }

  return res;
}


/*
 * Called whenever the contents of a directory are desired. Could be from an 'ls'
 * or could even be when a user hits TAB to do autocompletion
 *
 * @param path		Relative path to directory
 * @param buf		  Buffer of files in directory
 * @param filler	Function allowing addition of files to list buffer
 * @param offset	n/a
 * @param fi		  n/a
 * @return			  Integer representing success of operation
 * 						0 				      Success
 * 						-ENOENT			    File not found
 * 						-ENAMETOOLONG 	Not following 8.3 naming convention
 *
 */
static int fuse_readdir(const char *path, void *buf, fuse_fill_dir_t filler, off_t offset, struct fuse_file_info *fi) {

  //Since we're building with -Wall (all warnings reported) we need
  //to "use" every parameter, so let's just cast them to void to
  //satisfy the compiler
  (void) offset;
  (void) fi;

  int ret = 0;
  int i;
  char dir[MAX_FILENAME+1];
  char fn[MAX_FILENAME+1];
  char ext[MAX_EXTENSION+1];

  // init memory
  memset(dir, 0, (MAX_FILENAME+1));
  memset(fn, 0, (MAX_FILENAME+1));
  memset(ext, 0, (MAX_EXTENSION+1));

  //the filler function allows us to add entries to the listing
  //read the fuse.h file for a description (in the ../include dir)
  filler(buf, ".", NULL, 0);
  filler(buf, "..", NULL, 0);

  // Parse path into dir, fn, ext; if path not "/" (root)
  if(strlen(path) != 1) {
    sscanf(path, "/%[^/]/%[^.].%s", dir, fn, ext);
  }

  // 8.3 file naming convention
  if(strlen(dir) > MAX_FILENAME || strlen(fn) > MAX_FILENAME || strlen(ext) > MAX_EXTENSION) {
    return -ENAMETOOLONG;
  }

  // Fail if filename or ext given (read dir)
  if (strcmp(fn, "") != 0 || strcmp(ext, "") != 0 ) {
    return -ENONET;
  }

  //Parse path
  if(strcmp(path, "/") == 0) {		//root dir
    fuse_root_directory root = get_root();

    for(i = 0; i < MAX_DIRS_IN_ROOT; i++){ 	//iterate through root dir
      if(strcmp(root.directories[i].dname, "") != 0) {
        filler(buf, root.directories[i].dname, NULL, 0);	//add to out
      }
    }
    ret = 0;
  } else {							//sub dir
    if (strcmp(dir, "") == 0) {
      ret = -ENONET;
    } else {
      struct fuse_directory sub_dir;
      struct fuse_root_directory root;

      memset(&sub_dir, 0, sizeof(struct fuse_directory));
      sub_dir.nStartBlock = -1;
      root=get_root();

      //Loop through array of sub dirs
      for (i=0; i<root.nDirectories; i++) {
        if ( strcmp(root.directories[i].dname, dir) == 0 ) {
          sub_dir = root.directories[i];
          break;
        }
      }

      if (strcmp(sub_dir.dname, "") == 0) {
        ret = -ENOENT;
      } else {
        FILE* disk;
        int loc;
        struct fuse_directory_entry directory;
        struct fuse_file_directory file_dir;
        char filename[MAX_FILENAME+1];

        loc = sub_dir.nStartBlock*BLOCK_SIZE;
        disk = fopen(".disk", "r+b");
        fseek(disk, loc, SEEK_SET);
        memset(&directory, 0, sizeof(struct fuse_directory_entry));
        memset(&directory, 0, sizeof(struct fuse_file_directory));

        fread(&directory, BLOCK_SIZE, 1, disk); //Read the directory data from memory to iterate over its files
        fclose(disk);

        for(i = 0; i < MAX_FILES_IN_DIR; i++) { //Iterate over files in dir
          file_dir = directory.files[i];
          if(strcmp(file_dir.fname, "") != 0){ //If the file is not empty, add it to the filler buffer
            strcpy(filename, file_dir.fname);
            if(strcmp(file_dir.fext, "") != 0){ //Append the file extension
              strcat(filename, ".");
              strcat(filename, file_dir.fext);
            }
            filler(buf, filename, NULL, 0);
          }
        }
        ret = 0;
      }
    }
  }
  return ret;
}

/*
 * Creates a directory. We can ignore mode since we're not dealing with
 * permissions, as long as getattr returns appropriate ones for us.
 *
 * @param path		Relative path to directory
 * @param mode		n/a
 * @return			Integer representing success of operation
 * 						0 				      Success
 * 						-EPERM 			    Directory not under root dir
 * 						-EEXIST 		    Directory already exists
 * 						-ENAMETOOLONG 	Not following 8.3 naming convention
 *
 */
static int fuse_mkdir(const char *path, mode_t mode)
{
  (void) path;
  (void) mode;

  //path will be in the format of /directory/sub_directory
  char* dir; //The first directory in the 2-level file system
  char* sub_dir; //The directory within the root's directory

  struct fuse_file_allocation_table fat;
  fuse_root_directory root;
  int i, j;

  //Parse the two strings
  int path_len = strlen(path);
  char path_copy[path_len];
  strcpy(path_copy, path);

  dir = strtok(path_copy, "/");
  sub_dir = strtok(NULL, "/"); //NULL indicates to continue where strtok left off at

  if(strlen(dir) > MAX_FILENAME) { //The main directory
    return -ENAMETOOLONG;
  } else if(sub_dir && sub_dir[0]){ //The user passed in a sub directory; illegal
    return -EPERM;
  }

  root = get_root();
  fat = get_fat();

  if (root.nDirectories >= MAX_DIRS_IN_ROOT) {
    return -EPERM; //Can't add anymore directories
  }

  //dir already exists?
  for(i = 0; i < MAX_DIRS_IN_ROOT; i++){
    if(strcmp(root.directories[i].dname, dir) == 0){
      return -EEXIST;
    }
  }

  for(i = 0; i < MAX_DIRS_IN_ROOT; i++){ //Iterate through the root's directories/folders
    if(strcmp(root.directories[i].dname, "") == 0){ //If this folder is nameless (it doesn't exist yet), use it to create a new directory
      struct fuse_directory new_dir;
      strcpy(new_dir.dname, dir); //Copy the user's new directory name into this struct

      for(j = DISK_START; j < MAX_FAT_ENTRIES; j++){ //Iterate over the FAT
        if(fat.tbl[j] == 0){ //Currently nothing allocated at this index
          fat.tbl[j] = EOF; //Directory only requires 1 block
          new_dir.nStartBlock = j;
          break;
        }
      }

      FILE* disk = fopen(".disk", "r+b");
      int loc = BLOCK_SIZE*new_dir.nStartBlock; //The location on the disk for this directory is the starting block * 512
      fseek(disk, loc, SEEK_SET);
      fuse_directory_entry de;
      de.nFiles = 0; //Directory begins with 0 files in it

      if( fread(&de, BLOCK_SIZE, 1, disk) == 1 ){ //fread returned successfully with 1 item
        memset(&de, 0, sizeof(struct fuse_directory_entry)); //Clear the directory data we just read in JUST IN CASE
        fwrite(&de, BLOCK_SIZE, 1, disk); //Write the new directory data
        fclose(disk);

        //Update the root with its new data and write it to disk, as well as the FAT
        root.nDirectories++;
        root.directories[i] = new_dir;

        set_root(&root);
        set_fat(&fat);
      } else{ //There was an error reading in the data from disk, so just close the file
        fclose(disk);
      }

      return 0;
    }
  }
  return 0;
}

/*
 * Removes a directory.
 */
static int fuse_rmdir(const char *path)
{
  (void) path;
  return 0;
}

/*
 * Does the actual creation of a file. Mode and dev can be ignored.
 *
 * man -s 2 mknod
 *
 * @param path		Relative path to file or directory
 * @param mode 		n/a
 * @param dev		n/a
 * @return			Integer representing success of operation
 * 						0 				Success
 * 						-EPERM			Illegal permissions
 * 						-EEXISTS		File already exists
 * 						-ENAMETOOLONG 	Not following 8.3 naming convention
 *
 *
 */
static int fuse_mknod(const char *path, mode_t mode, dev_t dev)
{
  (void) mode;
  (void) dev;

  //path will be in the format of /directory/sub_directory
  char* directory; //The first directory in the 2-level file system
  char* file_name; //The directory within the root's directory
  char* file_ext;

  //Parse the two strings
  int path_length = strlen(path);
  char path_copy[path_length];
  strcpy(path_copy, path);

  directory = strtok(path_copy, "/");
  file_name = strtok(NULL, "."); //NULL indicates to continue where strtok left off at
  file_ext = strtok(NULL, ".");

  if((directory && directory[0]) && strcmp(directory, "") != 0){ //Directory and filename are not empty, so search for it
    if(file_name && file_name[0]){ //filename NULL check
      if(strcmp(file_name, "") == 0){ //filename is empty
        return -EPERM; //Can't create a file in the root directory
      }

      if(file_ext && file_ext[0]){ //file extension NULL check
        if(strlen(file_name) > MAX_FILENAME || strlen(file_ext) > MAX_EXTENSION){ //filename or extension is longer than the 8.3 format
          return -ENAMETOOLONG;
        }
      } else{ //filename is not null, but file extensin is
        if(strlen(file_name) > MAX_FILENAME){ //filename is longer than 8 characters
          return -ENAMETOOLONG;
        }
      }
    } else{ //filename is null, so only directory was given
      return -EPERM; //Can't create in the root directory
    }

    //Read in the root and FAT so we can grab their data
    fuse_root_directory root = get_root();
    struct fuse_file_allocation_table fat = get_fat();

    struct fuse_directory dir;

    int i = 0;
    for(i = 0; i < MAX_DIRS_IN_ROOT; i++){ //Iterate over the directories in the root to find the correct directory
      struct fuse_directory curr_dir = root.directories[i];
      if(strcmp(directory, curr_dir.dname) == 0){ //Found a matching directory!
        dir = curr_dir;
        break;
      }
    }

    if(strcmp(dir.dname, "") != 0){ //Valid directory was found
      //Read in the directory from disk
      long dir_location_on_disk = BLOCK_SIZE*dir.nStartBlock;
      FILE* disk = fopen(".disk", "r+b");
      fseek(disk, dir_location_on_disk, SEEK_SET);

      fuse_directory_entry dir_entry;
      int success = fread(&dir_entry, BLOCK_SIZE, 1, disk);

      if(dir_entry.nFiles >= MAX_FILES_IN_DIR){
        return -EPERM; //Can't create more files than allowed in a directory
      }

      if(success){ //We can add another file to this directory
        int file_already_exists = 0;
        int first_free_file_dir_index = -1;

        int j = 0;
        for(j = 0; j < MAX_FILES_IN_DIR; j++){ //Iterate over the files in this directory to make sure the file doesn't already exist
          struct fuse_file_directory curr_file_dir = dir_entry.files[j];
          if(strcmp(curr_file_dir.fname, "") == 0 && strcmp(curr_file_dir.fext, "") == 0 && first_free_file_dir_index == -1){ //Keep track of this later so we can easily add the new file to this array index
            first_free_file_dir_index = j;
          }

          if(strcmp(curr_file_dir.fname, file_name) == 0 && strcmp(curr_file_dir.fext, file_ext) == 0){ //Found a file with the same filename and extension; abort!
            file_already_exists = 1;
            break;
          }
        }

        if(!file_already_exists){ //File doesn't exist already
          short file_fat_start_index = -1;

          int k = 0;
          for(k = 2; k < MAX_FAT_ENTRIES; k++){ //Allocate new block in the FAT for this file
            if(fat.tbl[k] == 0){
              file_fat_start_index = k;
              fat.tbl[k] = EOF;
              break;
            }
          }

          struct fuse_file_directory new_file_dir;
          strcpy(new_file_dir.fname, file_name);
          if(file_ext && file_ext[0]) strcpy(new_file_dir.fext, file_ext); //Add the file extension to the file entry
          else strcpy(new_file_dir.fext, ""); //Make the extension blank if none was given
          new_file_dir.fsize = 0; //Initialize file size to 0
          new_file_dir.nStartBlock = file_fat_start_index;

          //Remember that index we kept track of later?  We can easily insert the new file at that index
          dir_entry.files[first_free_file_dir_index] = new_file_dir;
          dir_entry.nFiles++; //This directory has 1 more file in it

          //Write the directory data back to disk
          fseek(disk, dir_location_on_disk, SEEK_SET);
          fwrite(&dir_entry, BLOCK_SIZE, 1, disk);

          fclose(disk);

          //Write the root and FAT back to disk
          set_root(&root);
          set_fat(&fat);
        } else{ //File already exists, so no permissions are given to add another one
          fclose(disk);
          return -EEXIST;
        }
      } else{ //Directory name is empty, so can't add a new file
        fclose(disk);
        return -EPERM;
      }
    } else{ //Directory string was null or empty
      if(strcmp(directory, "") == 0){
        return 0;
      } else if(strcmp(file_name, "") == 0){
        return -EPERM;
      }
    }
  }

  return 0;
}

/*
 * Deletes a file
 */
static int fuse_unlink(const char *path)
{
  (void) path;
  return 0;
}

/*
 * Read size bytes from file into buf starting from offset
 *
 */
static int fuse_read(const char *path, char *buf, size_t size, off_t offset,
        struct fuse_file_info *fi)
{
  (void) buf;
  (void) offset;
  (void) fi;
  (void) path;

  char* directory;
  char* file_name;
  char* file_ext;

  //Parse the two strings
  int path_length = strlen(path);
  char path_copy[path_length];
  strcpy(path_copy, path);

  directory = strtok(path_copy, "/");
  file_name = strtok(NULL, "."); //NULL indicates to continue where strtok left off at
  file_ext = strtok(NULL, ".");

  if((directory && directory[0]) && strcmp(directory, "") != 0){ //Directory and filename are not empty, so search for it
    if(file_name && file_name[0]){ //filename NULL check
      if(strcmp(file_name, "") == 0){ //filename is empty
        return -EEXIST; //Can't read a file in the root directory
      }

      if(file_ext && file_ext[0]){ //file extension NULL check
        //Check if filename or extension are too long
        if(strlen(file_name) > MAX_FILENAME || strlen(file_ext) > MAX_EXTENSION){
          return -ENAMETOOLONG;
        }
      } else{
        //Check if filename is too long
        if(strlen(file_name) > MAX_FILENAME){
          return -ENAMETOOLONG;
        }
      }
    } else{ //filename is NULL
      return -EEXIST; //Can't read a file in the root directory
    }

    //Read in the root and FAT
    fuse_root_directory root = get_root();
    struct fuse_file_allocation_table fat = get_fat();

    struct fuse_directory dir;

    int i = 0;
    for(i = 0; i < MAX_DIRS_IN_ROOT; i++){ //Iterate over the directories in the root to find the respective directory
      struct fuse_directory curr_dir = root.directories[i];
      if(strcmp(directory, curr_dir.dname) == 0){ //Found a matching directory!
        dir = curr_dir;
        break;
      }
    }

    if(strcmp(dir.dname, "") != 0){ //Valid directory was found
      //Read in the directory entry from disk
      long dir_location_on_disk = BLOCK_SIZE*dir.nStartBlock;
      FILE* disk = fopen(".disk", "r+b");
      fseek(disk, dir_location_on_disk, SEEK_SET);

      fuse_directory_entry dir_entry;
      int success = fread(&dir_entry, BLOCK_SIZE, 1, disk);

      fclose(disk);

      if(success){ //One directory entry was read in
        struct fuse_file_directory file_dir;

        int j = 0;
        for(j = 0; j < MAX_FILES_IN_DIR; j++){ //Search through this directory to find the file we're looking for
          struct fuse_file_directory curr_file_dir = dir_entry.files[j];

          if(strcmp(curr_file_dir.fname, file_name) == 0){ //Matching filename
            if(file_ext && file_ext[0]){ //File extension null check
              if(strcmp(curr_file_dir.fext, file_ext) == 0){ //Matching file extension, so we found the file!
                file_dir = curr_file_dir;
                break;
              }
            } else{ //File extension null check
              if(strcmp(curr_file_dir.fext, "") == 0){ //Matching file extension (empty), so we found the file!
                file_dir = curr_file_dir;
                break;
              }
            }
          }
        }

        if(strcmp(file_dir.fname, "") != 0){ //Filename is empty, so don't continue
          if(offset > file_dir.fsize){ //Offset is bigger than the file size
            return -EFBIG;
          }

          //Find the starting block number and offset to read from
          int block_number_of_file = 0;
          if(offset != 0) block_number_of_file = offset/BLOCK_SIZE;
          int offset_of_block = 0;
          if(offset != 0) offset_of_block = offset - block_number_of_file*BLOCK_SIZE;

          //Iterate through the FAT to find the starting block
          int curr_block = file_dir.nStartBlock;
          if(block_number_of_file != 0){
            while(block_number_of_file > 0){
              curr_block = fat.tbl[curr_block];
              block_number_of_file--;
            }
          }

          //Open the disk to read the data from the respective blocks of the file
          FILE* disk = fopen(".disk", "r+b");
          fseek(disk, BLOCK_SIZE*curr_block+offset_of_block, SEEK_SET);
          fuse_disk_block new_data;
          fread(&new_data.data, BLOCK_SIZE-offset_of_block, 1, disk);
          int curr_buffer_size = 0;

          //Append the read in data to the buffer
          if(file_dir.fsize >= BLOCK_SIZE){
            memcpy(buf, &new_data.data, BLOCK_SIZE-offset_of_block);
          } else{
            memcpy(buf, &new_data.data, file_dir.fsize);
          }

          curr_buffer_size = BLOCK_SIZE - offset_of_block; //Increase the size of the buffer

          //While this file hasn't ended yet and there are still more block sto read in, repeat the above procedure and keep iterating through the blocks of the file until EOF is reached
          while(fat.tbl[curr_block] != EOF){
            curr_block = fat.tbl[curr_block];

            fuse_disk_block data;
            fseek(disk, BLOCK_SIZE*curr_block, SEEK_SET);
            fread(&data.data, BLOCK_SIZE, 1, disk);
            memcpy(buf+curr_buffer_size, &data, strlen(data.data));
            curr_buffer_size += strlen(data.data);
          }

          fclose(disk);

          //Write the root and FAT back to disk
          set_root(&root);
          set_fat(&fat);

          size = curr_buffer_size;
        } else{ //Filename is empty, so can't read from a directory
          return -EISDIR;
        }
      } else{ //Directory is empty, so can't read in anything
        return -EPERM;
      }
    } else{ //Directory is empty or null, so either don't read in anything or return that permission is denied
      if(strcmp(directory, "") == 0){
        return 0;
      } else if(strcmp(file_name, "") == 0){
        return -EPERM;
      }
    }
  }

  return size; //Return the size of the buffer that is returned
}

/*
 * Write size bytes from buf into file starting from offset
 *
 */
static int fuse_write(const char *path, const char *buf, size_t size,
        off_t offset, struct fuse_file_info *fi)
{
  (void) buf;
  (void) offset;
  (void) fi;
  (void) path;

  char* directory;
  char* file_name;
  char* file_ext;

  //Parse the two strings
  int path_length = strlen(path);
  char path_copy[path_length];
  strcpy(path_copy, path);

  directory = strtok(path_copy, "/");
  file_name = strtok(NULL, "."); //NULL indicates to continue where strtok left off at
  file_ext = strtok(NULL, ".");

  if((directory && directory[0]) && strcmp(directory, "") != 0){ //Directory and filename are not empty, so search for it
    if(file_name && file_name[0]){ //Null check for filename
      if(strcmp(file_name, "") == 0){ //Check if filename is empty
        return -EEXIST; //Can't read a file in the root directory
      }

      if(file_ext && file_ext[0]){ //Null check for file extension
        //CHeck if the filename or extension exceed their maximum sizes of the 8.3 format
        if(strlen(file_name) > MAX_FILENAME || strlen(file_ext) > MAX_EXTENSION){
          return -ENAMETOOLONG;
        }
      } else{
        //Check if the file extension exceeds 3 characters
        if(strlen(file_name) > MAX_FILENAME){
          return -ENAMETOOLONG;
        }
      }
    } else{ //The filename is null, so return that it doesn't exist
      return -EEXIST; //Can't read a file in the root directory
    }

    //Read in the root and FAT to get data later
    fuse_root_directory root = get_root();
    struct fuse_file_allocation_table fat = get_fat();

    struct fuse_directory dir;

    int i = 0;
    for(i = 0; i < MAX_DIRS_IN_ROOT; i++){ //Iterate over the directories in the root
      struct fuse_directory curr_dir = root.directories[i];
      if(strcmp(directory, curr_dir.dname) == 0){ //Found a matching directory!
        dir = curr_dir;
        break;
      }
    }

    if(strcmp(dir.dname, "") != 0){ //Valid directory was found
      long dir_location_on_disk = BLOCK_SIZE*dir.nStartBlock;

      //Open the disk, read in the directory entry we're looking for, and close the disk
      FILE* disk = fopen(".disk", "r+b");
      fseek(disk, dir_location_on_disk, SEEK_SET);
      fuse_directory_entry dir_entry;
      int success = fread(&dir_entry, BLOCK_SIZE, 1, disk);
      fclose(disk);

      if(success){ //The directory entry was successfully read in
        struct fuse_file_directory file_dir;
        int file_directory_index = -1;

        int j;
        for(j = 0; j < MAX_FILES_IN_DIR; j++){ //Search through the files in the directory to find the file we're trying to write to
          struct fuse_file_directory curr_file_dir = dir_entry.files[j];

          if(strcmp(curr_file_dir.fname, file_name) == 0){ //Found a matching filename
            if(file_ext && file_ext[0]){ //Check whether the file we're looking for has a file extension or not; if true, it does
              if(strcmp(curr_file_dir.fext, file_ext) == 0){ //There is a matching file extension, so we found the file!
                file_dir = curr_file_dir;
                file_directory_index = j;
                break;
              }
            } else{ //File we're looking for doesn't have an extension
              if(strcmp(curr_file_dir.fext, "") == 0){ //Found the file!
                file_dir = curr_file_dir;
                file_directory_index = j;
                break;
              }
            }
          }
        }

        if(strcmp(file_dir.fname, "") != 0){ //Check if the filename is empty
          if(offset > file_dir.fsize){ //Offset is greater than the file size, so don't write
            return -EFBIG;
          }
          int buffer_size = strlen(buf); //Buffer size is the length of the buf
          int write_bytes_until_append = file_dir.fsize - offset; //The number of bytes we can write until we start appending

          //Calculate the number of blocks we must skip to get to a normal offset where we can just starting writing normally
          int block_number_of_file = 0; //Number of blocks that the offset must skip
          if(offset != 0) {
            block_number_of_file = offset/BLOCK_SIZE;
          }
          int offset_of_block = 0; //Once we reach the starting block, how much offset is left over?
          if(offset != 0) {
            offset_of_block = offset - block_number_of_file*BLOCK_SIZE;
          }

          //Iterate through the FAT until we get to the correct starting block that the offset belongs in
          int curr_block = file_dir.nStartBlock;
          if(block_number_of_file != 0){
            while(block_number_of_file > 0){
              curr_block = fat.tbl[curr_block];
              block_number_of_file--;
            }
          }

          int buffer_bytes_remaining = buffer_size; //The number of bytes remaining that we have to write to disk

          //Open the disk and write the correct about of buffer data in the first block; this basically gets rid of the offset so we can then start writing entire Blocks at a time later
          FILE* disk = fopen(".disk", "r+b");
          fseek(disk, BLOCK_SIZE*curr_block+offset_of_block, SEEK_SET);
          if(buffer_size >= BLOCK_SIZE){ //This means there will be left over stuff in the buffer that we have to write after we finish writing to this block
            fwrite(buf, BLOCK_SIZE-offset_of_block, 1, disk);
            buffer_bytes_remaining -= (BLOCK_SIZE-offset_of_block);

            if(offset == size){ //Expand the file size
              file_dir.fsize = offset+1;
            }
          } else{ //We can fit all of the data we want to write into this one block
            fwrite(buf, buffer_size, 1, disk);

            char null_array[BLOCK_SIZE-buffer_size];
            int m = 0;
            for(m = 0; m < BLOCK_SIZE-buffer_size; m++){
              null_array[m] = '\0';
            }
            fwrite(null_array, BLOCK_SIZE-buffer_size, 1, disk);
            buffer_bytes_remaining -= buffer_size;

            if(file_dir.fsize > size){ //File directory is greater than size, so we must re-adjust the file size to the length of size (e.g. "this is text" is replaced with "word");
              if(offset == size){
                file_dir.fsize = offset+1;
              } else{
                file_dir.fsize = size;
              }
            }
          }


          int bytes_to_clear = size - buffer_size;

          while(buffer_bytes_remaining > 0){ //There's still more data to write from buf
            if(fat.tbl[curr_block] == EOF){ //Basically append data to a file; allocate more blocks in the FAT.
              int free_block_found = 0;
              int k;
              for(k = DISK_START; k < MAX_FAT_ENTRIES; k++){ //We need to allocate another block for this file to write more bytes; find a new block in the FAT
                if(fat.tbl[k] == 0){
                  fat.tbl[curr_block] = k;
                  fat.tbl[k] = EOF;
                  curr_block = k;
                  free_block_found = 1;
                  break;
                }
              }

              if(!free_block_found){ //No more free blocks in the FAT.
                return -EPERM; //Can't write anymore to a file; ran out of memory on disk.
              }
            } else{ //We still have empty room on the file, so just continue writing into the next block
              curr_block = fat.tbl[curr_block];
            }

            fseek(disk, BLOCK_SIZE*curr_block, SEEK_SET);
            if(buffer_bytes_remaining >= BLOCK_SIZE){ //We'll still have to do more iterations after this because we have more data to write than one block
              char* new_buf_address = buf + (buffer_size - buffer_bytes_remaining);
              fwrite(new_buf_address, BLOCK_SIZE, 1, disk);
              buffer_bytes_remaining -= BLOCK_SIZE;
            } else{ //This is our final write because we don't have anything left in the buffer to write; so just write it and finish
              char* new_buf_address = buf + (buffer_size - buffer_bytes_remaining);
              fwrite(new_buf_address, buffer_bytes_remaining, 1, disk);

              buffer_bytes_remaining = 0; //buffer_bytes_remaining - buffer_bytes_remaining = 0 bytes to write
            }
          }

          //Calculate the number of bytes written and appended to the file
          int write_bytes = buffer_size - buffer_bytes_remaining; //The number of bytes written so far
          int appended_bytes = write_bytes - write_bytes_until_append;
          if(appended_bytes > 0){ //Increase the file size if there was an append
            file_dir.fsize += appended_bytes;
          }

          dir_entry.files[file_directory_index] = file_dir;
          fseek(disk, dir.nStartBlock*BLOCK_SIZE, SEEK_SET);
          fwrite(&dir_entry, BLOCK_SIZE, 1, disk);

          fclose(disk);

          set_root(&root);
          set_fat(&fat);

          size = buffer_size;

        } else{ //The directory entry failed to be read from disk; so return that this is a directyory and we can't write to it
          return -EISDIR;
        }
      } else{ //Directory name is empty; return there is no permissions
        return -EPERM;
      }
    } else{ //Directory name is empty or null, so return success since we don't have to write anything or that there are no permissions
      if(strcmp(directory, "") == 0){
        return 0;
      } else if(strcmp(file_name, "") == 0){
        return -EPERM;
      }
    }
  }

  return size; //Return the amount of data that was written to file from the buffer
}

/******************************************************************************
 *
 *  DO NOT MODIFY ANYTHING BELOW THIS LINE
 *
 *****************************************************************************/

/*
 * truncate is called when a new file is created (with a 0 size) or when an
 * existing file is made shorter. We're not handling deleting files or
 * truncating existing ones, so all we need to do here is to initialize
 * the appropriate directory entry.
 */
static int fuse_truncate(const char *path, off_t size)
{
  (void) path;
  (void) size;

    return 0;
}


/*
 * Called when we open a file
 */
static int fuse_open(const char *path, struct fuse_file_info *fi)
{
  (void) path;
  (void) fi;
    /*
        //if we can't find the desired file, return an error
        return -ENOENT;
    */

    //It's not really necessary for this project to anything in open

    /* We're not going to worry about permissions for this project, but
     if we were and we don't have them to the file we should return an error

        return -EACCES;
    */

    return 0; //success!
}

/*
 * Called when close is called on a file descriptor, but because it might
 * have been dup'ed, this isn't a guarantee we won't ever need the file
 * again. For us, return success simply to avoid the unimplemented error
 * in the debug log.
 */
static int fuse_flush (const char *path , struct fuse_file_info *fi)
{
  (void) path;
  (void) fi;

  return 0; //success!
}


//register our new functions as the implementations of the syscalls
static struct fuse_operations fs_oper = {
  .getattr	= fuse_getattr,
  .readdir	= fuse_readdir,
  .mkdir	  = fuse_mkdir,
  .rmdir    = fuse_rmdir,
  .read	    = fuse_read,
  .write	  = fuse_write,
  .mknod	  = fuse_mknod,
  .unlink   = fuse_unlink,
  .truncate = fuse_truncate,
  .flush    = fuse_flush,
  .open	    = fuse_open,
};

//Don't change this.
int main(int argc, char *argv[])
{
  return fuse_main(argc, argv, &fs_oper, NULL);
}
