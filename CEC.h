#ifndef CEC_H__
#define CEC_H__

#include "CECWire.h"

class CEC_LogicalDevice : public CEC_Electrical
{
public:
	CEC_LogicalDevice(int physicalAddress);

private:
	int _physicalAddress;
};

#endif // CEC_H__
