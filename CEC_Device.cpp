#include "CEC_Device.h"

bool XX_GetLineState();
void XX_SetLineState(CEC_Device* device, bool state);

CEC_Device::CEC_Device(int physicalAddress)
: CEC_LogicalDevice(physicalAddress)
, _isrTriggered(false)
, _lastLineState2(true)
{
}

CEC_Device::~CEC_Device()
{
}

void CEC_Device::OnReady()
{
  // This is called after the logical address has been
  // allocated
  DbgPrint("Device ready\n");
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
  CEC_LogicalDevice::OnTransmitComplete(buffer, count, ack);
}

bool CEC_Device::LineState()
{
  return XX_GetLineState();
}

void CEC_Device::SetLineState(bool state)
{
  XX_SetLineState(this, state);
}

void CEC_Device::SignalIRQ()
{
  // This is called when the line has changed state
  _isrTriggered = true;
}

bool CEC_Device::IsISRTriggered()
{
  if (_isrTriggered)
  {
    _isrTriggered = false;
    return true;
  }
  return false;
}

void CEC_Device::Run()
{
  bool state = XX_GetLineState();
  if (_lastLineState2 != state)
  {
    _lastLineState2 = state;
    SignalIRQ();
  }
  CEC_LogicalDevice::Run();
}
