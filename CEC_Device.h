#ifndef CEC_DEVICE_H__
#define CEC_DEVICE_H__

#include "CEC.h"

class CEC_Device : public CEC_LogicalDevice
{
public:
  CEC_Device(int physicalAddress);
  virtual ~CEC_Device();
  
protected:
  virtual bool LineState();
  virtual void SetLineState(bool);

  virtual void OnReady(int logicalAddress);
  virtual void OnReceiveComplete(unsigned char* buffer, int count, bool ack);
  virtual void OnTransmitComplete(unsigned char* buffer, int count, bool ack);

private:
friend void XX_SetLineState(CEC_Device* device, bool state);
friend bool XX_GetLineState();
};

#endif // CEC_DEVICE_H__
