

#include <stdio.h>
#include <stdlib.h>
 
int main(void)
{
    // This program will create Hex file with random content

    unsigned char iNum;	

    for(int offset = 0; offset< (1024*8*16); offset++) {
        printf("@%07x\n",offset*256);
        for(int j = 0; j<16; j++) {
           for(int i = 0; i<16; i++) {
               iNum = rand();
               printf(" %02x ", iNum);
           }
           printf("\n");
        }
    }
    return 0;
}

