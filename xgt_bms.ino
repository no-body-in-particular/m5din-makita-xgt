
#include <XGTBms.h>
xgt_bms bms(9); // RX, TX

//BMS for xgt batteries, simple protocol implementation
//not needed for the battery analysis tool, just included for complete documentation of the protocol

void setup() {

}

void loop() {
	while(true) {
		bms.process();
	}
	// put your main code here, to run repeatedly:

}
