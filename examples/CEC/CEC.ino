#include "CEC_Device.h"

// NodeMCU ESP8266 Pins   bootstrap
// D0  GPIO16  DeepSleep  blue LED connected to 3V3
// D1  GPIO5
// D2  GPIO4
// D3  GPIO0              pull-up (SPI boot) / pull-down (UART boot)
// D4  GPIO2   TxD1       pull-up, blue LED on ESP12E module
// D5  GPIO14
// D6  GPIO12
// D7  GPIO13
// D8  GPIO15             pull-down
// D9  GPIO3   RxD0
// D10 GPIO1   TxD0
// CLK GPIO6   SD_CLK
// SD0 GPIO7   SD_D0
// SD1 GPIO8   SD_D1
// SD2 GPIO9   SD_D2
// SD3 GPIO10  SD_D3
// CMD GPIO11  SD_CMD

#define CEC_GPIO 5

// implement application specific CEC device
class MyCEC_Device : public CEC_Device
{
protected:
	virtual bool LineState();
	virtual void SetLineState(bool);
	virtual void OnReady(int logicalAddress);
	virtual void OnReceiveComplete(unsigned char* buffer, int count, bool ack);
	virtual void OnTransmitComplete(unsigned char* buffer, int count, bool ack);
};

bool MyCEC_Device::LineState()
{
	int state = digitalRead(CEC_GPIO);
	return state != LOW;
}

void MyCEC_Device::SetLineState(bool state)
{
	if (state) {
		pinMode(CEC_GPIO, INPUT_PULLUP);
	} else {
		digitalWrite(CEC_GPIO, LOW);
		pinMode(CEC_GPIO, OUTPUT);
	}
	// give enough time for the line to settle before sampling it
	delayMicroseconds(50);
}

void MyCEC_Device::OnReady(int logicalAddress)
{
	// This is called after the logical address has been allocated
	DbgPrint("Device ready, Logical address assigned: %d\n", logicalAddress);
}

void MyCEC_Device::OnReceiveComplete(unsigned char* buffer, int count, bool ack)
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

void MyCEC_Device::OnTransmitComplete(unsigned char* buffer, int count, bool ack)
{
	// This is called after a frame is transmitted.
	DbgPrint("Packet sent at %ld: %02X", millis(), buffer[0]);
	for (int i = 1; i < count; i++)
		DbgPrint(":%02X", buffer[i]);
	if (!ack)
		DbgPrint(" NAK");
	DbgPrint("\n");
}


MyCEC_Device device;

void setup()
{
	pinMode(CEC_GPIO, INPUT_PULLUP);

	Serial.begin(115200);
	device.Initialize(0x1000, CEC_Device::CDT_PLAYBACK_DEVICE, true);
}

void loop()
{
	if (Serial.available()) {
		unsigned char c = Serial.read();
		unsigned char buffer[3];
    
		switch (c) {
		case '1':
			buffer[0] = 0x36;
			device.TransmitFrame(4, buffer, 1);
			break;
		}
	}
	device.Run();
}
