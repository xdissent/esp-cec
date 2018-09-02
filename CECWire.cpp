#include "CECWire.h"

CEC_Electrical::CEC_Electrical(int address)
{
	MonitorMode = false;
	Promiscuous = false;

	_address = address & 0x0f;
	_amLastTransmittor = false;
	_transmitPending = false;
	_xmitretry = 0;
	ResetState();
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

unsigned long CEC_Electrical::LineError()
{
        DbgPrint("%p: Line Error!\n", this);
	if (_follower || _broadcast)
	{
		_secondaryState = CEC_RCV_LINEERROR;
		Lower();
		return micros() + 3600;
	}
	return ResetState() ? micros() : (unsigned long)-1;
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

//DbgPrint("%d %d %d\n", _primaryState, _secondaryState, _tertiaryState);
	// If the state hasn't changed and we're not in a transmit
	// state then this is spurrious.
	if( currentLineState == lastLineState &&
		!(_primaryState == CEC_TRANSMIT ||
		  (_primaryState == CEC_RECEIVE &&
		   (_secondaryState == CEC_RCV_ACK_SENT ||
		    _secondaryState == CEC_RCV_LINEERROR))))
		return waitTime;

	unsigned long lasttime = _lastStateChangeTime;
	unsigned long difftime = time - _bitStartTime;

	if( currentLineState != lastLineState )
	{
		// Line state has changed, update our internal state
		_lastLineState = currentLineState;
		_lastStateChangeTime = time;
	}

	switch (_primaryState)
	{
	case CEC_IDLE:
		// If a high to low transition occurs, then we must be
		// beginning a start bit
		if (lastLineState && !currentLineState)
		{
			_primaryState = CEC_RECEIVE;
			_secondaryState = CEC_RCV_STARTBIT1;
			_bitStartTime = time;
			_follower = false;
			_broadcast = false;
			_amLastTransmittor = false;
			break;
		}
		
		// Nothing to do until we have a need to transmit
		// or we detect the falling edge of the start bit
		break;

	case CEC_RECEIVE:
		switch (_secondaryState)
		{
		case CEC_RCV_STARTBIT1:
			// We're waiting for the rising edge of the start bit
			if (difftime >= 3500 && difftime <= 3900)
			{
				// We now need to wait for the next falling edge
				_secondaryState = CEC_RCV_STARTBIT2;
				break;
			}
			// Illegal state.  Go back to CEC_IDLE to wait for a valid
			// start bit
//DbgPrint("1: %ld %ld\n", difftime, micros());
			waitTime = ResetState() ? micros() : (unsigned long)-1;
			break;
		
		case CEC_RCV_STARTBIT2:
			// This should be the falling edge of the start bit			
			if (difftime >= 4300 && difftime <= 4700)
			{
				// We've fully received the start bit.  Begin receiving
				// a data bit
				_secondaryState = CEC_RCV_DATABIT1;
				_bitStartTime = time;
				break;
			}
			// Illegal state.  Go back to CEC_IDLE to wait for a valid
			// start bit
//DbgPrint("2: %ld %ld\n", difftime, micros());
			waitTime = ResetState() ? micros() : (unsigned long)-1;
			break;
		
		case CEC_RCV_DATABIT1:
		case CEC_RCV_EOM1:
			// We've received the rising edge of the data/eom bit
			bool bit;
			if (difftime >= 400 && difftime <= 800)
				bit = true;
			else if (difftime >= 1300 && difftime <= 1700)
				bit = false;
			else
			{
				// Illegal state.  Go back to CEC_IDLE to wait for a valid
				// start bit
				waitTime = LineError();
				break;
			}
			// Save the received bit
			if (_secondaryState == CEC_RCV_EOM1)
			{
				_eom = bit;
			}
			else
			{
				unsigned int idx = _receiveBufferBits >> 3;
				if (idx < sizeof(_receiveBuffer))
				{
					_receiveBuffer[idx] = (_receiveBuffer[idx] << 1) | bit;
					_receiveBufferBits++;
				}
			}
			_secondaryState = (CEC_SECONDARY_STATE)(_secondaryState + 1);
			break;

		case CEC_RCV_DATABIT2:
		case CEC_RCV_EOM2:
			// We've received the falling edge of the data bit
			if (difftime >= 2050 && difftime <= 2750)
			{
				_bitStartTime = time;
				if (_secondaryState == CEC_RCV_EOM2)
				{
					_secondaryState = CEC_RCV_ACK1;

					// Check to see if the frame is addressed to us
					// or if we are in promiscuous mode (in which case we'll receive everything)
					int address = _receiveBuffer[0] & 0x0f;
					if (address == 0x0f)
						_broadcast = true;
					else if (address == _address)
						_follower = true;

					// If we're the follower, go low for a while
					if (_follower)
					{
						Lower();

						_secondaryState = CEC_RCV_ACK_SENT;
						waitTime = _bitStartTime + 1500;
					}
					else if (!Promiscuous && !_broadcast)
					{
						// It's not addressed to us.  Reset and wait for the next start bit
						waitTime = ResetState() ? micros() : (unsigned long)-1;
					}
					break;
				}
				// Receive another bit
				_secondaryState = ((_receiveBufferBits & 7) == 0) ? CEC_RCV_EOM1 : CEC_RCV_DATABIT1;
				break;
			}
			// Illegal state.  Go back to CEC_IDLE to wait for a valid
			// start bit
			waitTime = LineError();
			break;

		case CEC_RCV_ACK_SENT:
			// We're done holding the line low...  release it
			Raise();
			if (_eom)
			{
				// We're not going to receive anything more from
				// the initiator (EOM has been received)
				// And we've sent the ACK for the most recent bit
				// therefore this message is all done.  Go back
				// to the IDLE state and wait for another start bit.
				ProcessFrame(true);
			        waitTime = ResetState() ? micros() : (unsigned long)-1;
				break;
			}
			// We need to wait for the falling edge of the ACK
			// to finish processing this ack
			_secondaryState = CEC_RCV_ACK2;
			break;

		case CEC_RCV_ACK1:
			{
				bool ack;
				if (difftime >= 400 && difftime <= 800)
					ack = _broadcast;
				else if (difftime >= 1300 && difftime <= 1700)
					ack = !_broadcast;
				else
				{
					// Illegal state.  Go back to CEC_IDLE to wait for a valid
					// start bit
					waitTime = LineError();
					break;
				}

				if (!_eom && ack)
				{
					// receive the rest of the ACK (or rather the beginning of the next bit)
					_secondaryState = CEC_RCV_ACK2;
					break;
				}
				// We're not going to receive anything more from
				// the initiator (EOM has been received).  Go back
				// to the IDLE state and wait for another start bit.
				ProcessFrame(ack);
				waitTime = ResetState() ? micros() : (unsigned long)-1;
				break;
			}

		case CEC_RCV_ACK2:
			// We're receiving the falling edge of the ack
			if (difftime >= 2050 && difftime <= 2750)
			{
				_secondaryState = CEC_RCV_DATABIT1;
				_bitStartTime = time;
				break;
			}
			// Illegal state (or NACK).  Either way, go back to CEC_IDLE
			// to wait for next start bit (maybe a retransmit)..
			waitTime = LineError();
			break;

		case CEC_RCV_LINEERROR:
			//DbgPrint("%p: Done signaling line error\n", this);
			Raise();
			waitTime = ResetState() ? micros() : (unsigned long)-1;
			break;
		
		}

		break;

	case CEC_TRANSMIT:
		if (lastLineState != currentLineState)
		{
			// Someone else is mucking with the line.  Wait for the
			// line to clear before appropriately before (re)transmit

			// However it is OK for a follower to ACK if we are in an
			// ACK state
			if (_secondaryState != CEC_XMIT_ACK3 &&
			    _secondaryState != CEC_XMIT_ACK_TEST)
			{
				// If a state changed TO LOW during IDLE wait, someone could be legitimately transmitting
				if (_secondaryState == CEC_IDLE_WAIT)
				{
					if (currentLineState == false)
					{
						_primaryState = CEC_RECEIVE;
						_secondaryState = CEC_RCV_STARTBIT1;
						_bitStartTime = time;
						_transmitPending = true;
					}
					break;
				}
				else
				{
					// Transmit collision
					ResetTransmit(true);
					waitTime = 0;
					break;
				}
			}
		}

		unsigned long neededIdleTime = 0;
		switch (_secondaryState)
		{
		case CEC_IDLE_WAIT:
			// We need to wait a certain amount of time before we can
			// transmit..

			// If the line is low, we can't do anything now.  Wait
			// indefinitely until a line state changes which will
			// catch in the code just above
			if (currentLineState == 0)
				break;

			// The line is high.  Have we waited long enough?
			neededIdleTime = 0;
			switch (_tertiaryState)
			{
			case CEC_IDLE_RETRANSMIT_FRAME:
				neededIdleTime = 3 * 2400;
				break;

			case CEC_IDLE_NEW_FRAME:
				neededIdleTime = 5 * 2400;
				break;

			case CEC_IDLE_SUBSEQUENT_FRAME:
				neededIdleTime = 7 * 2400;
				break;
			}

			if (time - _lastStateChangeTime < neededIdleTime)
			{
				// not waited long enough, wait some more!
				waitTime = lasttime + neededIdleTime;
				break;
			}

			// we've wait long enough, begin start bit
			Lower();
			_bitStartTime = time;
			_amLastTransmittor = true;
			_broadcast = (_transmitBuffer[0] & 0x0f) == 0x0f;

			// wait 3700 microsec
			waitTime = _bitStartTime + 3700;
			
			// and transition to our next state
			_secondaryState = CEC_XMIT_STARTBIT1;
			break;

		case CEC_XMIT_STARTBIT1:
			if (!Raise())
			{
				//DbgPrint("%p: Received Line Error\n", this);
				ResetTransmit(true);
				break;
			}

			waitTime = _bitStartTime + 4500;
			_secondaryState = CEC_XMIT_STARTBIT2;
			_tertiaryState = CEC_XMIT_START;
			break;

		case CEC_XMIT_DATABIT1:
			if (!Raise())
			{
				//DbgPrint("%p: Received Line Error\n", this);
				ResetTransmit(true);
				break;
			}

			waitTime = _bitStartTime + 2400;

			if (_tertiaryState == CEC_XMIT_BIT_EOM)
			{
				// We've just finished transmitting the EOM
				// move on to the ACK
				_secondaryState = CEC_XMIT_ACK;
			}			
			else
				_secondaryState = CEC_XMIT_DATABIT2;
			break;

		case CEC_XMIT_STARTBIT2:
		case CEC_XMIT_DATABIT2:
		case CEC_XMIT_ACK4:
			Lower();
			_bitStartTime = time;

			_tertiaryState = (CEC_TERTIARY_STATE)(_tertiaryState + 1);

			if (_tertiaryState == CEC_XMIT_BIT_EOM)
			{
				// get EOM bit: transmit buffer empty?
				_eom = (_transmitBufferBytes == (_transmitBufferBitIdx >> 3));
				waitTime = _bitStartTime + ((_eom) ? 600 : 1500);
			}
			else
			{
				// pull bit from transmit buffer
				unsigned char b = _transmitBuffer[_transmitBufferBitIdx >> 3] << (_transmitBufferBitIdx++ & 7);
				waitTime = _bitStartTime + (( b >> 7) ? 600 : 1500);
			}
			_secondaryState = CEC_XMIT_DATABIT1;
			break;

		case CEC_XMIT_ACK:
			Lower();
			_bitStartTime = time;

			// We transmit a '1'
			//DbgPrint("%p: Sending ack\n", this);
			waitTime = _bitStartTime + 600;
			_secondaryState = CEC_XMIT_ACK2;
			break;

		case CEC_XMIT_ACK2:
			Raise();

			// we need to sample the state in a little bit
			waitTime = _bitStartTime + 1050;
			_secondaryState = CEC_XMIT_ACK_TEST;
			break;

		case CEC_XMIT_ACK_TEST:
			//DbgPrint("%p: Testing ack: %d\n", this, (currentLineState == 0) != _broadcast?1:0);
			if ((currentLineState != 0) != _broadcast)
			{
				// not being acknowledged
				// normally we retransmit.  But this is NOT the case for <Polling Message> as its
				// function is basically to 'ping' a logical address in which case we just want 
				// acknowledgement that it has succeeded or failed
				if (_transmitBufferBytes == 1)
				{
					OnTransmitComplete(_transmitBuffer, _transmitBufferBitIdx >> 3, false);
					ResetState();
				}
				else
				{
					ResetTransmit(true);
					waitTime = 0;
				}
				break;
			}

			_lastStateChangeTime = lasttime;

			if (_eom)
			{
				// Nothing left to transmit, go back to idle
				OnTransmitComplete(_transmitBuffer, _transmitBufferBitIdx >> 3, true);
				ResetState();
				break;
			}

			// We have more to transmit, so do so...
			if (currentLineState != 0)
			{
				waitTime = _bitStartTime + 2400;
				_secondaryState = CEC_XMIT_ACK4;
			}
			else
			{
				_secondaryState = CEC_XMIT_ACK3;
			}
			_tertiaryState = CEC_XMIT_START;
			break;
		case CEC_XMIT_ACK3:
			// received rising edge of ack
			waitTime = _bitStartTime + 2400;
			_secondaryState = CEC_XMIT_ACK4;
			break;

		}
	}
	return waitTime;	
}

bool CEC_Electrical::ResetState()
{
	_primaryState = CEC_IDLE;
	_secondaryState = (CEC_SECONDARY_STATE)0;
	_tertiaryState = (CEC_TERTIARY_STATE)0;
	_eom = false;
	_follower = false;
	_broadcast = false;
	_receiveBufferBits = 0;
	_transmitBufferBytes = 0;
	_transmitBufferBitIdx = 0;

	if (_transmitPending)
        {
		ResetTransmit(false);
                return true;
        }
        return false;
}

void CEC_Electrical::ResetTransmit(bool retransmit)
{
	_primaryState = CEC_TRANSMIT;
	_secondaryState = CEC_IDLE_WAIT;
	_tertiaryState = CEC_IDLE_NEW_FRAME;
	_transmitPending = false;

	if (retransmit)
	{
		if (++_xmitretry == CEC_MAX_RETRANSMIT)
		{
			// No more
			OnTransmitComplete(_transmitBuffer, _transmitBufferBitIdx >> 3, false);
			ResetState();
		}
		else
		{
			//DbgPrint("%p: Retransmitting current frame\n", this);
			_tertiaryState = CEC_IDLE_RETRANSMIT_FRAME;
			_transmitBufferBitIdx = 0;
		}
	}
	else 
	{
		_xmitretry = 0;
		if (_amLastTransmittor)
		{
			_tertiaryState = CEC_IDLE_SUBSEQUENT_FRAME;
		}
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

	if (_primaryState == CEC_IDLE)
		ResetTransmit(false);
	else
		_transmitPending = true;
	return true;
}
