#ifndef _ALEX_SERIALPORT_H
#define _ALEX_SERIALPORT_H

#include "uavInc.h"
#include "uavtype.h"

int Alex_SerialPort_Close();
int Alex_SerialPort_BufferClear();
void ALex_SerialPort_ReadBufferSize(size_t *size);
int Alex_SerialPort_Send(const char *buf, size_t len);
int Alex_SerialPort_Recv(char *buf, size_t len);
int Alex_SerialPort_Setup(const char *device, int baudrate);


#endif

