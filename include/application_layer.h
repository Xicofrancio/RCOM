// application_layer.h

#ifndef _APPLICATION_LAYER_H_
#define _APPLICATION_LAYER_H_

#include <stdio.h>

typedef struct LinkLayer LinkLayer;

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename);

void transmitData(int fd, const char *filename);
void receiveData(int fd);

unsigned char* parseControlPacket(unsigned char* packet, int size, unsigned long int *fileSize);
void parseDataPacket(const unsigned char* packet, const unsigned int packetSize, unsigned char* buffer);

unsigned char * getControlPacket(const unsigned int c, const char* filename, long int length, unsigned int* size);
unsigned char * getDataPacket(unsigned char sequence, unsigned char *data, int dataSize, int *packetSize);
unsigned char * getData(FILE* fd, long int fileLength);

#endif // _APPLICATION_LAYER_H_
