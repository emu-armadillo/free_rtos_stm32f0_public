#include <proj_util.h>


void init_cir_buf(char * buf, int size, CircularBuffer * b){
	b->start = 0;
	b->end = 0;
	b->size = size;
	b->buffer = buf;

}
