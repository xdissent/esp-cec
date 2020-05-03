#include "CEC_Device.h"

bool XX_GetLineState();
void XX_SetLineState(CEC_Device* device, bool state);

CEC_Device::CEC_Device(int physicalAddress)
: CEC_LogicalDevice(physicalAddress)
{
}

CEC_Device::~CEC_Device()
{
}

void CEC_Device::OnReady(int logicalAddress)
{
	// This is called after the logical address has been allocated
	DbgPrint("Device ready, Logical address assigned: %d\n", logicalAddress);
}

void CEC_Device::OnReceiveComplete(unsigned char* buffer, int count, bool ack)
{
  // This is called when a frame is received.  To transmit
  // a frame call TransmitFrame.  To receive all frames, even
  // those not addressed to this device, set Promiscuous to true.
  DbgPrint("Packet received at %ld: %02X", millis(), buffer[0]);
  for (int i = 1; i < count; i++)
    DbgPrint(":%02X", buffer[i]);
  if (!ack)
    DbgPrint(" NAK");
  DbgPrint("\n");
}

void CEC_Device::OnTransmitComplete(unsigned char* buffer, int count, bool ack)
{

  DbgPrint("Packet sent at %ld: %02X", millis(), buffer[0]);
  for (int i = 1; i < count; i++)
    DbgPrint(":%02X", buffer[i]);
  if (!ack)
    DbgPrint(" NAK");
  DbgPrint("\n");
}

bool CEC_Device::LineState()
{
  return XX_GetLineState();
}

void CEC_Device::SetLineState(bool state)
{
  XX_SetLineState(this, state);
}
