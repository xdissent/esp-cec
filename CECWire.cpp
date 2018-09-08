#include "CECWire.h"

CEC_Electrical::CEC_Electrical(int address) :
	MonitorMode(false),
	Promiscuous(false),
	_address(address & 0x0f),
	_state(CEC_IDLE),
	_receiveBufferBits(0),
	_transmitBufferBytes(0),
	_amLastTransmittor(false),
	_waitTime(0)
{
}

void CEC_Electrical::Initialize()
{
	_lastLineState = LineState();
	_lastStateChangeTime = micros();
}

void CEC_Electrical::SetAddress(int address)
{
	_address = address & 0x0f;
}

bool CEC_Electrical::Raise()
{
	if (MonitorMode)
		return LineState();

	unsigned long time = micros();
	SetLineState(1);
	// Only update state if the line was actually changed (i.e. it wasn't already in its new state)
	if (LineState())
	{
		_lastLineState = true;
		_lastStateChangeTime = time;
		return true;
	}
	return false;
}

bool CEC_Electrical::Lower()
{
	if (MonitorMode)
		return LineState();

	unsigned long time = micros();
	SetLineState(0);
	// Only update state if the line was actually changed (i.e. it wasn't already in its new state)
	if (!LineState())
	{
		_lastLineState = false;
		_lastStateChangeTime = time;
		return false;
	}
	return true;
}

///
/// CEC_Electrical::Process implements our main state machine
/// which includes all reading and writing of state including
/// acknowledgements and arbitration
///

unsigned long CEC_Electrical::Process()
{
	// We are either called because of an INTerrupt in which case
	// state has changed or because of a poll (during write).

	bool currentLineState = LineState();
	bool lastLineState = _lastLineState;
	unsigned long waitTime = -1;	// INFINITE; (== wait until an external event has occurred)
	unsigned long time = micros();
	unsigned long lasttime = _lastStateChangeTime;
	unsigned long difftime = time - _bitStartTime;
	bool bit;

	if( currentLineState != lastLineState )
	{
		// Line state has changed, update our internal state
		_lastLineState = currentLineState;
		_lastStateChangeTime = time;

		if (_state >= CEC_XMIT_WAIT && _state != CEC_XMIT_ACK_TEST && _state != CEC_XMIT_ACK_WAIT)
		// We are in a transmit state and someone else is mucking with the line.  Wait for the
		// line to clear before appropriately before (re)transmit
		// However it is OK for a follower to ACK if we are in an ACK state
		{
			if (_state == CEC_XMIT_WAIT)
			{
				// If a state changed during transmit wait, someone else could be legitimately transmitting
				_state = CEC_IDLE;
			}
			else
			{
				// Transmit collision
				ResetTransmit();
				waitTime = 0;
			}
		}
	}

	switch (_state)
	{
	case CEC_IDLE:
		// If a high to low transition occurs, then we must be
		// beginning a start bit
		if (lastLineState && !currentLineState)
		{
			_state = CEC_RCV_STARTBIT1;
			_bitStartTime = time;
			_ack = true;
			_follower = false;
			_broadcast = false;
			_amLastTransmittor = false;
			break;
		}
		
		// Nothing to do until we have a need to transmit
		// or we detect the falling edge of the start bit
		break;

		case CEC_RCV_STARTBIT1:
			// We're waiting for the rising edge of the start bit
			if (difftime >= (STARTBIT_TIME_LOW - BIT_TIME_LOW_MARGIN) &&
			    difftime <= (STARTBIT_TIME_LOW + BIT_TIME_LOW_MARGIN))
			{
				// We now need to wait for the next falling edge
				_state = CEC_RCV_STARTBIT2;
				break;
			}
			// Illegal state.  Go back to CEC_IDLE to wait for a valid start bit
			waitTime = ResetState() ? 0 : (unsigned long)-1;
			break;
		
		case CEC_RCV_STARTBIT2:
			// This should be the falling edge of the start bit
			if (difftime >= (STARTBIT_TIME - BIT_TIME_LOW_MARGIN))
			{
				// We've fully received the start bit.  Begin receiving
				// a data bit
				_state = CEC_RCV_DATABIT1;
				_bitStartTime = time;
				break;
			}
			// Illegal state.  Go back to CEC_IDLE to wait for a valid start bit
			waitTime = ResetState() ? 0 : (unsigned long)-1;
			break;
		
		case CEC_RCV_DATABIT1:
		case CEC_RCV_EOM1:
		case CEC_RCV_ACK1:
			// We've received the rising edge of the data/eom/ack bit
			if (difftime >= (BIT_TIME_LOW_1 - BIT_TIME_LOW_MARGIN) &&
			    difftime <= (BIT_TIME_LOW_1 + BIT_TIME_LOW_MARGIN))
				bit = true;
			else if (difftime >= (BIT_TIME_LOW_0 - BIT_TIME_LOW_MARGIN) &&
			         difftime <= (BIT_TIME_LOW_0 + BIT_TIME_LOW_MARGIN))
				bit = false;
			else
			{
				// Illegal state.  Send NAK later.
				bit = true;
				_ack = false;
			}
			if (_state == CEC_RCV_EOM1)
			{
				_eom = bit;
			}
			else if (_state == CEC_RCV_ACK1)
			{
				_ack = (bit == _broadcast);
				if (_eom || !_ack)
				{
					// We're not going to receive anything more from the initiator.
					// Go back to the IDLE state and wait for another start bit.
					ProcessFrame(_ack);
					waitTime = ResetState() ? 0 : (unsigned long)-1;
					break;
				}
			}
			else
			{
				// Save the received bit
				unsigned int idx = _receiveBufferBits >> 3;
				if (idx < sizeof(_receiveBuffer))
				{
					_receiveBuffer[idx] = (_receiveBuffer[idx] << 1) | bit;
					_receiveBufferBits++;
				}
			}
			_state = (CEC_STATE)(_state + 1);
			break;

		case CEC_RCV_DATABIT2:
		case CEC_RCV_EOM2:
		case CEC_RCV_ACK2:
			// We've received the falling edge of the data/eom/ack bit
			if (difftime >= (BIT_TIME - BIT_TIME_MARGIN))
			{
				_bitStartTime = time;
				if (_state == CEC_RCV_EOM2)
				{
					_state = CEC_RCV_ACK1;

					// Check to see if the frame is addressed to us
					// or if we are in promiscuous mode (in which case we'll receive everything)
					int address = _receiveBuffer[0] & 0x0f;
					if (address == 0x0f)
						_broadcast = true;
					else if (address == _address)
						_follower = true;

					// Go low for ack/nak
					if ((_follower && _ack) || (_broadcast && !_ack))
					{
						Lower();
						_state = CEC_RCV_ACK_SENT;
						waitTime = _bitStartTime + BIT_TIME_LOW_0;
					}
					else if (!_ack || (!Promiscuous && !_broadcast))
					{
						// It's broken or not addressed to us.  Reset and wait for the next start bit
						waitTime = ResetState() ? 0 : (unsigned long)-1;
					}
					break;
				}
				// Receive another bit
				_state = (_state == CEC_RCV_DATABIT2 &&
				          (_receiveBufferBits & 7) == 0) ? CEC_RCV_EOM1 : CEC_RCV_DATABIT1;
				break;
			}
			// Line error.
			Lower();
			_state = CEC_RCV_LINEERROR;
			waitTime = time + BIT_TIME_ERR;
			break;

		case CEC_RCV_ACK_SENT:
			// We're done holding the line low...  release it
			Raise();
			if (_eom || !_ack)
			{
				// We're not going to receive anything more from
				// the initiator (EOM has been received)
				// or we've sent the NAK for the most recent bit
				// therefore this message is all done.  Go back
				// to the IDLE state and wait for another start bit.
				ProcessFrame(_ack);
			        waitTime = ResetState() ? 0 : (unsigned long)-1;
				break;
			}
			// We need to wait for the falling edge of the ACK
			// to finish processing this ack
			_state = CEC_RCV_ACK2;
			break;

		case CEC_RCV_LINEERROR:
			Raise();
			waitTime = ResetState() ? 0 : (unsigned long)-1;
			break;
		
		case CEC_XMIT_WAIT:
			// We need to wait a certain amount of time before we can
			// transmit..

			// If the line is low, we can't do anything now.  Wait
			// indefinitely until a line state changes
			if (currentLineState == 0)
				break;
			// The line is high.  Have we waited long enough?
			unsigned long neededIdleTime;
			neededIdleTime = ((_xmitretry) ?  3 * BIT_TIME :
			                  (_amLastTransmittor) ? 7 * BIT_TIME :
			                  5 * BIT_TIME);
			if (time - _lastStateChangeTime < neededIdleTime)
			{
				// not waited long enough, wait some more!
				waitTime = lasttime + neededIdleTime;
				break;
			}
			// we've wait long enough, begin start bit
			Lower();
			_bitStartTime = time;
			_transmitBufferBitIdx = 0;
			_amLastTransmittor = true;
			_broadcast = (_transmitBuffer[0] & 0x0f) == 0x0f;
			waitTime = _bitStartTime + STARTBIT_TIME_LOW;
			_state = CEC_XMIT_STARTBIT1;
			break;

		case CEC_XMIT_STARTBIT1:
			// We're at start bit low, send the rising edge of the start bit
			if (!Raise())
			{
				//DbgPrint("%p: Received Line Error\n", this);
				ResetTransmit();
				break;
			}
			waitTime = _bitStartTime + STARTBIT_TIME;
			_state = CEC_XMIT_STARTBIT2;
			break;

		case CEC_XMIT_DATABIT1:
		case CEC_XMIT_EOM1:
			// We finished the first half of the bit, send the rising edge
			if (!Raise())
			{
				//DbgPrint("%p: Received Line Error\n", this);
				ResetTransmit();
				break;
			}
			waitTime = _bitStartTime + BIT_TIME;
			_state = (CEC_STATE)(_state + 1);
			break;

		case CEC_XMIT_STARTBIT2:
		case CEC_XMIT_DATABIT2:
		case CEC_XMIT_EOM2:
		case CEC_XMIT_ACK2:
			// We finished the second half of the previous bit, send the falling edge of the new bit
			Lower();
			_bitStartTime = time;
			if (_state == CEC_XMIT_DATABIT2 && (_transmitBufferBitIdx & 7) == 0)
			{
				_state = CEC_XMIT_EOM1;
				// Get EOM bit: transmit buffer empty?
				bit = _eom = (_transmitBufferBytes == (_transmitBufferBitIdx >> 3));
			}
			else if (_state == CEC_XMIT_EOM2)
			{
				_state = CEC_XMIT_ACK1;
				bit = true; // We transmit a '1'
			}
			else
			{
				_state = CEC_XMIT_DATABIT1;
				// Pull bit from transmit buffer
				unsigned char b = _transmitBuffer[_transmitBufferBitIdx >> 3] << (_transmitBufferBitIdx++ & 7);
				bit = b >> 7;
			}
			waitTime = _bitStartTime + (bit ? BIT_TIME_LOW_1 : BIT_TIME_LOW_0);
			break;

		case CEC_XMIT_ACK1:
			// We finished the first half of the ack bit, release the line
			Raise();                         // No check, maybe follower pulls low for ACK
			waitTime = _bitStartTime + BIT_TIME_SAMPLE;
			_state = CEC_XMIT_ACK_TEST;
			break;

		case CEC_XMIT_ACK_TEST:
			//DbgPrint("%p: Testing ack: %d\n", this, (currentLineState == 0) != _broadcast?1:0);
			if ((currentLineState != 0) != _broadcast)
			{
				// not being acknowledged
				OnTransmitComplete(_transmitBuffer, _transmitBufferBitIdx >> 3, false);

				// normally we retransmit.  But this is NOT the case for <Polling Message> as its
				// function is basically to 'ping' a logical address in which case we just want 
				// acknowledgement that it has succeeded or failed
				if (_transmitBufferBytes == 1)
				{
					_transmitBufferBytes = 0;
					_state = CEC_IDLE;
				}
				else
				{
					ResetTransmit();
					waitTime = 0;
				}
				break;
			}
			_lastStateChangeTime = lasttime;
			if (_eom)
			{
				// Nothing left to transmit, go back to idle
				OnTransmitComplete(_transmitBuffer, _transmitBufferBitIdx >> 3, true);
				_transmitBufferBytes = 0;
				_state = CEC_IDLE;
				break;
			}
			// We have more to transmit, so do so...
			_state = CEC_XMIT_ACK_WAIT; // wait for line going high
			if (currentLineState != 0)  // already high, no ACK from follower
			{
				waitTime = _bitStartTime + BIT_TIME;
				_state = CEC_XMIT_ACK2;
			}
			break;
		case CEC_XMIT_ACK_WAIT:
			// We received the rising edge of ack from follower
			waitTime = _bitStartTime + BIT_TIME;
			_state = CEC_XMIT_ACK2;
			break;
	}
	return waitTime;	
}

bool CEC_Electrical::ResetState()
{
	if (_transmitBufferBytes != 0)
        {
	        _state = CEC_XMIT_WAIT;
                return true;
        }
	_state = CEC_IDLE;
        return false;
}

void CEC_Electrical::ResetTransmit()
{
	if (++_xmitretry == CEC_MAX_RETRANSMIT)
	{
		// No more
		_transmitBufferBytes = 0;
		_state = CEC_IDLE;
	}
	else
	{
		// Retransmit
		_state = CEC_XMIT_WAIT;
	}
}

void CEC_Electrical::ProcessFrame(bool ack)
{
	// We've successfully received a frame, allow it to be processed
	OnReceiveComplete(_receiveBuffer, _receiveBufferBits >> 3, ack);
	_receiveBufferBits = 0;
}

bool CEC_Electrical::Transmit(int sourceAddress, int targetAddress, unsigned char* buffer, unsigned int count)
{
	if (MonitorMode)
		return false; // we must not transmit in monitor mode
	if (_transmitBufferBytes != 0)
		return false; // pending transmit packet
	if (count >= sizeof(_transmitBuffer))
		return false; // packet too big

	_transmitBuffer[0] = (sourceAddress << 4) | (targetAddress & 0xf);
	for (int i = 0; i < count; i++)
		_transmitBuffer[i+1] = buffer[i];
	_transmitBufferBytes = count + 1;
	_xmitretry = 0;

	if (_state == CEC_IDLE)
		_state = CEC_XMIT_WAIT;
	return true;
}

void CEC_Electrical::Run()
{
	if (((_waitTime == (unsigned long)-1 && !TransmitPending()) ||
	     (_waitTime != (unsigned long)-1 && _waitTime > micros())) && LineState() == _lastLineState)
		return;

        _waitTime = Process();
}
