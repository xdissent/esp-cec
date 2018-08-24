#ifndef SERIAL_H__
#define SERIAL_H__

#include "Common.h"

#define SERIAL_BUFFER_SIZE 16

class SerialLine
{
public:
	SerialLine();

	void ClearTransmitBuffer();
	virtual bool Transmit(unsigned char* buffer, int count);
	virtual bool TransmitPartial(unsigned char* buffer, int count);

protected:
	unsigned char _transmitBuffer[SERIAL_BUFFER_SIZE];
	int _transmitBufferCount;
	int _transmitBufferBit;
	int _transmitBufferByte;

	virtual void OnTransmitBegin() {;}

	int PopTransmitBit();
	int RemainingTransmitBytes();
	int TransmitSize();
	void ResetTransmitBuffer();
};

#endif // SERIAL_H__
