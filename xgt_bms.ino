
#include <XGTBms.h>
xgt_bms bms(9); // RX, TX


void setup() {

}

void loop() {
	while(true) {
		bms.process();
	}
	// put your main code here, to run repeatedly:

}
