
//zmm15@pitt.edu
//project 2. String utility
//4020473
//6/28/17

#include <stdio.h>
#include <stdlib.h>

unsigned int stringLen;
int string_start_offset=0;
int string_end_offset=0;
int size_of_file;
char* file_buff;
int main(int argc, char * argv[]){
	FILE* f=fopen(argv[1],"rb");                        //binary mode
	if(f==NULL){
		printf("file does not exist...\n");
		return -1;
	}
	fseek(f,0,SEEK_END);
	size_of_file=ftell(f);                               //find size of file
	fseek(f, 0, SEEK_SET);
	
	file_buff=malloc(size_of_file);			//allocate heap for the files
	fread(file_buff,1,size_of_file,f);
	
	
	while(1){
		while((file_buff[string_end_offset]<126)&&(file_buff[string_end_offset]>32)){//check for printable chars
			if(string_end_offset==size_of_file-1){
				break;
			}
			string_end_offset++;
			
		}
		if(string_end_offset-string_start_offset>=4){ //only print if more than 4
			printf("%.*s\n",string_end_offset-string_start_offset,&file_buff[string_start_offset]);
		}
		
		string_end_offset++;
		string_start_offset=string_end_offset;
		if(string_end_offset==size_of_file){
				break;
		}		
	}
	
	free(file_buff);
	
	return 0;
}