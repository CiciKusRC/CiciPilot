/* 
 *
 * Copyright 2020 The MathWorks, Inc.
 *
 * File: px4_file_logging.cpp
 *
 *
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "rtwtypes.h"
#include <sys/stat.h>

extern "C" {  /* sbcheck:ok:extern_c needed because ert_targets_logging.c is a C file and PX4 is a C++ language target*/
#include "ert_targets_logging.h"
}
#include "nuttxinitialize.h"
#include "MW_PX4_TaskControl.h"

//Reference : http://nuttx.org/Documentation/NuttxUserGuide.html#standardio

FILE* MW_fopen(const char* filename, const char* mode) 
{
	char sdCardPath[200] = "/fs/microsd/";
	//Ensure the size of full file name of the file does not create memory issues	
	if((strlen(sdCardPath) + strlen(filename))>=200)
    {
        return NULL;
	}
	strcat(sdCardPath,filename);//Append filename to SD card Path
	return fopen(sdCardPath, mode);
}


size_t MW_fwrite(const void* ptr, size_t size, size_t nmemb, FILE* stream) 
{
    size_t temp;
    temp = fwrite(ptr, size, nmemb, stream);
    return temp; 
}

size_t MW_fread(void* ptr, size_t size, size_t nmemb, FILE* stream) 
{
    size_t temp;
    temp = fread(ptr, size, nmemb, stream);
    return temp;
}

void MW_rewind(FILE* stream) {
    fseek(stream, 0, SEEK_SET);
}

int MW_fclose(FILE* stream) {
    return fclose(stream);
}

int MW_remove(const char* filename) {
    return remove(filename);
}

