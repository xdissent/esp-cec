#include "Serial.h"

SerialLine::SerialLine()
{
	_transmitBufferCount = 0;
	_transmitBufferBit = 7;
	_transmitBufferByte = 0;
}

void SerialLine::ClearTransmitBuffer()
{
	_transmitBufferCount = 0;
	_transmitBufferBit = 7;
	_transmitBufferByte = 0;
}

bool SerialLine::Transmit(unsigned char* buffer, int count)
{
	if (!TransmitPartial(buffer, count))
		return false;

	OnTransmitBegin();
	return true;
}

bool SerialLine::TransmitPartial(unsigned char* buffer, int count)
{
	if (count < 0 || count >= (SERIAL_BUFFER_SIZE - _transmitBufferCount))
		return false;

	for (int i = 0; i < count; i++)
		_transmitBuffer[_transmitBufferCount + i] = buffer[i];
	_transmitBufferCount += count;
	_transmitBufferBit = 7;
	_transmitBufferByte = 0;
	return true;
}

int SerialLine::PopTransmitBit()
{
	if (_transmitBufferByte == _transmitBufferCount)
		return 0;

	int bit = (_transmitBuffer[_transmitBufferByte] >> _transmitBufferBit) & 1;
	if( _transmitBufferBit-- == 0 )
	{
		_transmitBufferBit = 7;
		_transmitBufferByte++;
	}
	return bit;
}

int SerialLine::RemainingTransmitBytes()
{
	return _transmitBufferCount - _transmitBufferByte;
}

int SerialLine::TransmitSize()
{
	return _transmitBufferCount;
}

void SerialLine::ResetTransmitBuffer()
{
	_transmitBufferBit = 7;
	_transmitBufferByte = 0;
}
