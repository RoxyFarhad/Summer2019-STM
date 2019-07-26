#include <iostream>
#include <stdio.h>
#include <string.h>


void error (const char *errorMessage) {
	printf("errorMessage");
	exit(1);
}



//navigate to the proper folder before running
int main(int argc, char** argv) 
{
	//check for proper args
	if (argc < 3) {
		error("args are filein fileout\n");
	}
	
	
	//vars for marker
	char marker[] = "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff";
	char outputMark[] = "|,";
	int markerFound = 0;
	char *markerStart;
	
	//vars for tracking reading and the input buffer
	size_t numRead = 0;
	long offset = 0;
	const size_t bufLen = 512;
	char buf[bufLen + 1];
	
	//null terminate the buffer for ease of computation
	buf[bufLen] = 0;

	//vars for the file objects and file names
	FILE *fileIn;
	FILE *fileOut;
	char *inName = argv[1];
	char *outName = argv[2];
	
	//open the file
	fileIn = fopen(inName, "rb");
	if (!fileIn) {
		error("failed fopen of input \n");
	}
	
	
	//find first set of 10 0xff
	while (!markerFound) {
		//read in bufLen bytes
		numRead = fread(buf, 1, bufLen, fileIn);
		if (numRead != bufLen) {
			//exit
			printf("failed fread at start \n");
			printf("numRead: %d\n", numRead);			
			return 1;
		}
		
		//check for marker in the read string
		markerStart = strstr(buf, marker);
		
		//if marker found, set markerFound, fseek to start of marker
		if (markerStart) {
			markerFound = 1;
			
			//calculate offset, move pointer back
			offset = -1 * strlen(markerStart);	
			fseek(fileIn, offset, SEEK_CUR);
			
			
			
			
			/*
			//test
			numRead = fread(buf, 1, bufLen, fileIn);
			if (numRead != bufLen) {
				//exit
				printf("failed fread \n");
				printf("numRead: %d\n", numRead);			
				return 1;
			}
			
			
			//test: print out 20 from buf start
			printf("offset \n");
			for (int i = 0; i < 20; i++) {
				printf("%d: %u\n", i, (unsigned char)buf[i]);
			}
			*/
		
		}
	}
	
	
	//open the output file
	fileOut = fopen(outName, "w");
	if (!fileIn) {
		error("failed fopen of output file \n");
	}
	
	
	//convert: take chunks of 512. if less than 512, assume is eof
	while (fread(buf, 1, bufLen, fileIn)) {
	
		//check for marker
		if (strstr(buf, marker) != buf) {
			error("marker not found \n");
		}
	
		//output "|," characters
		fputs(outputMark, fileOut);
		
		//i'm be slow: convert each to ascii and print it out
		for (int i = 0; i < bufLen; i++) {
			fprintf(fileOut, "%u,", (unsigned char)buf[i]);
		}
	}
	
	//close the files
	fclose(fileIn);
	fclose(fileOut);
	
	
	
	
	return 0;
}
