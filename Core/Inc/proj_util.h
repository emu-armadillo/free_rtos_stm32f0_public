#ifndef UTIL_HHH
//triple h because it is a common name
#define UTIL_HHH
#include "cmsis_os.h"
typedef struct circular_buffer{
	int size;
	int start;
	int end;
	char * buffer;
} CircularBuffer;


void init_cir_buf(char * buf, int size, CircularBuffer * b);



#endif
