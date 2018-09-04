#ifndef CECWIRE_H__
#define CECWIRE_H__

#include "Common.h"

class CEC_Electrical
{
public:
	CEC_Electrical(int address);
	void Initialize();
	void SetAddress(int address);

	unsigned long Process();
	bool TransmitPending() { return _state == CEC_XMIT_WAIT; }

	int Promiscuous;
	int MonitorMode;

protected:
	virtual bool LineState() = 0;
	virtual void SetLineState(bool) = 0;
	virtual void OnTransmitComplete(unsigned char* buffer, int count, bool ack) = 0;
	virtual void OnReceiveComplete(unsigned char* buffer, int count, bool ack) = 0;
	bool Transmit(int sourceAddress, int targetAddress, unsigned char* buffer, unsigned int count);

private:
	typedef enum {
		CEC_IDLE,

		CEC_RCV_STARTBIT1,
		CEC_RCV_STARTBIT2,
		CEC_RCV_DATABIT1,
		CEC_RCV_DATABIT2,
		CEC_RCV_EOM1,
		CEC_RCV_EOM2,
		CEC_RCV_ACK_SENT,
		CEC_RCV_ACK1,
		CEC_RCV_ACK2,
		CEC_RCV_LINEERROR,

		CEC_XMIT_WAIT,
		CEC_XMIT_STARTBIT1,
		CEC_XMIT_STARTBIT2,
		CEC_XMIT_DATABIT1,
		CEC_XMIT_DATABIT2,
		CEC_XMIT_EOM1,
		CEC_XMIT_EOM2,
		CEC_XMIT_ACK1,
		CEC_XMIT_ACK_TEST,
		CEC_XMIT_ACK_WAIT,
		CEC_XMIT_ACK2,
	} CEC_STATE;

	enum {
		STARTBIT_TIME_LOW   = 3700, // 3.7ms
		STARTBIT_TIME       = 4500, // 4.5ms
		BIT_TIME_LOW_0      = 1500, // 1.5ms
		BIT_TIME_LOW_1      =  600, // 0.6ms
		BIT_TIME_SAMPLE     = 1050, // 1.05ms
		BIT_TIME            = 2400, // 2.4ms
		BIT_TIME_LOW_MARGIN =  300, // 0.2ms  plus some additional margin since we poll the bitline
		BIT_TIME_MARGIN     =  450, // 0.35ms plus some additional margin since we poll the bitline
	};

	// Receive buffer
	unsigned char _receiveBuffer[16];
	unsigned int _receiveBufferBits;

	// Transmit buffer
	unsigned char _transmitBuffer[16];
	unsigned int _transmitBufferBytes;
	unsigned int _transmitBufferBitIdx;

	bool ResetState();
	void ResetTransmit(bool retransmit);
	void ProcessFrame(bool ack);

	// Helper functions
	bool Raise();
	bool Lower();
	unsigned long LineError();

	int _address;
	bool _lastLineState;
	unsigned long _lastStateChangeTime;
	unsigned long _bitStartTime;

	int _xmitretry;
	enum {
		CEC_MAX_RETRANSMIT = 5,
	};

	bool _eom;
	bool _follower;
	bool _broadcast;
	bool _amLastTransmittor;
	bool _transmitPending;

	CEC_STATE _state;
};



#endif // CECWIRE_H__
