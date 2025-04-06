#ifndef XGT_BMS_H
#define XGT_BMS_H

#include <XGTSerial.h>

class xgt_bms {
public:
	xgt_bms(int pin):serial1(pin) {
		reset();
	}


	void reset() {
		for(int i=0; i<10; i++)cell_voltages[i]=4.1f;
		for(int i=0; i<6; i++)current_histogram[i]=0;
		for(int i=0; i<6; i++)temperature_histogram[i]=0;
		pack_temperature[0]=25;
		pack_temperature[1]=25;
		capacity=6000;
		remaining_capacity=100;
		charge_count=0;
		charge_pct=100;
	}

	void send_rpy(char * rpy, int len) {
		delay(1);
		set_crc(rpy,len);
		serial1.write(rpy,8);
	}

	void set_charge_count(uint16_t cnt) {
		charge_count=cnt;
	}

	void set_cell_voltage(int idx, float voltage) {
		cell_voltages[idx]=voltage;
	}

	void set_charge_pct(float pct) {
		charge_pct=pct;
	}

	void set_current_histogram(int idx, uint8_t value) {
		current_histogram[idx]=value;
	}
	void set_temperature_histogram(int idx, uint8_t value) {
		temperature_histogram[idx]=value;
	}

	void set_temperature(int idx, float value) {
		pack_temperature[idx]=value;
	}

    void process_c0(uint8_t * data,uint8_t * rpy_buff){
		if( data[3]==0 && data[4]==0x54) {
			rpy_buff[4]=charge_count&0xff;
			rpy_buff[5]=(charge_count&0xff00)>>8;
		}

		if( data[3]==0x03) {
			//cell voltage
			uint8_t idx=data[4]>>1;

			if(idx==0) {
				float sum=0;
				for(int i=0; i<10; i++)sum+=cell_voltages[i];
				uint16_t voltage=sum*1000;
				rpy_buff[4]=voltage&0xff;
				rpy_buff[5]=(voltage&0xff00)>>8;

			} else {
				idx--;
				if(idx<10) {
					uint16_t voltage=cell_voltages[idx]*1000;
					rpy_buff[4]=voltage&0xff;
					rpy_buff[5]=(voltage&0xff00)>>8;
				}
			}
		}

		if( data[3]==0x03 && (data[4]==0x1A || data[4]==0x1C)) {
			uint8_t idx=data[4]==0x1A?0:1;
			uint16_t temp=( pack_temperature[idx]  +273.15f)*10;
			rpy_buff[4]=temp&0xff;
			rpy_buff[5]=(temp&0xff00)>>8;
		}

		if(data[3]==0x00 && (data[4]>=0xc0 && data[4]<=0xc4)) {
			uint8_t idx=data[4]-0xc0;
			memcpy(rpy_buff+4,temperature_histogram+idx,2);
		}

		if( data[3]==0x00 && (data[4]>=0xd8 && data[4]<=0xdc)) {
			uint8_t idx=data[4]-0xd8;
			memcpy(rpy_buff+4,current_histogram+idx,2);
		}

		if(data[3]==0x00 && data[4]==0x64) {
			uint16_t cap=(capacity *  remaining_capacity)/100.0f;
			rpy_buff[4]=cap&0xff;
			rpy_buff[5]=(cap&0xff00)>>8;
		}

		if( data[3]==0x01 && data[4]==0x08 ) {
			uint16_t chg=charge_pct*0xff;
			rpy_buff[4]=chg&0xff;
			rpy_buff[5]=(chg&0xff00)>>8;
		}
    }
    
	void process_shortcmd(uint8_t * data) {
		uint8_t rpy_buff[]= {0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33  } ;
		rpy_buff[2]=data[2];
		rpy_buff[3]=data[3];

        if(data[2]==0xc0){
            process_c0(data,rpy_buff);
        }else if(data[2]==0xdd){ 
		if( data[3]==0x08 ) {
			uint8_t cap=capacity/100;
			rpy_buff[4]=0x24;
			rpy_buff[5]=cap;
		}

		if( data[3]==0x0a ) {
			uint8_t string_size=10;
			uint8_t paralel_cnt=1;
			rpy_buff[4]=paralel_cnt;
			rpy_buff[5]=string_size;
		}
    }

        
		send_rpy(rpy_buff,8);
	}

	void process() {
		uint8_t inp=serial1.recv();

		if(inp==0xcc) {     //read 8 byte command and send 8 byte rpy
			uint8_t buff[8];
			buff[0]=0xcc;
			serial1.read(buff+1,7);
			process_shortcmd(buff);

		}

		//read long command and send rpy
		if(inp==0xA5) {
			serial1.recv();
			uint16_t message_len= serial1.recv()<<8 |  serial1.recv();//word 1
			message_len>>=3;
			message_len&=0xf;
			message_len<<=4;
			message_len+=16;

			uint16_t message_id= serial1.recv()<<8 |  serial1.recv();//word 2
			serial1.recv();
			serial1.recv();
			serial1.recv();
			serial1.recv();
			uint16_t cmd= serial1.recv()<<8| serial1.recv();//word 5
			int time=millis();
			//recieve whole command
			for(; (millis()-time)<300;) {
				if(serial1.recv()) {
					time=millis();
				}
			}
			//send response
			if(cmd==0x1300) {
				uint8_t rpy_buf[]= {0xA5,0xA5,0x00, 0x00, 0x90, 0x01, 0x4d, 0x4c, 0x00, 0x0c, 0xb3, 0x00, 0x00, 0x00, 0x01, 0xe9};
				send_rpy(rpy_buf,sizeof(rpy_buf));
			} else {
				//1302
				uint8_t rpy_buf[]=   {0xA5,0xA5,0x00, 0x6f, 0x90, 0x02, 0x4d, 0x4c, 0x00, 0x0c, 0x33, 0x02, 0x00, 0x51, 0x00, 0x01, 0x00, 0x00, 0x13, 0x01, 0xff, 0xff, 0x13, 0x06, 0x0d, 0xfc, 0x13, 0x07, 0x09, 0xc4, 0x13, 0x0c, 0xbd, 0x13, 0x0d, 0x0a, 0xcd, 0x13, 0x0e, 0x31, 0xe0, 0x4a, 0xe0, 0x13, 0x0f, 0x00, 0x29, 0x32, 0x00, 0x76, 0x3c, 0x70, 0x6b, 0x6b, 0x9a, 0x69, 0x61, 0x64, 0xab, 0x60, 0xc0, 0x5b, 0x8b, 0x51, 0x20, 0x51, 0x20, 0x00, 0x29, 0x32, 0x00, 0x76, 0x3c, 0x70, 0x6b, 0x6b, 0x9a, 0x69, 0x61, 0x64, 0xab, 0x60, 0xc0, 0x5b, 0x8b, 0x51, 0x20, 0x51, 0x20, 0x0d, 0x02, 0x13, 0x10, 0x00, 0x00, 0x1a, 0x27, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff   };
				send_rpy(rpy_buf,sizeof(rpy_buf));
			}
		}
	}


	uint8_t shortcrc(uint8_t * rxBuf, uint8_t length) { //calculate CRC based on data recieved from battery.
		uint16_t crc = rxBuf[0];

		for (uint8_t i = 2; i < length; i++) { //loop through array of data starting at position 3 to exclude CRC
			crc += (rxBuf[i]); //sum bytes in packet
		}

		return crc;
	}

	uint16_t longcrc(uint8_t * rxBuf, uint8_t length) {
		uint16_t crc = 0;
		length -= (rxBuf[3] & 0xF); //subtract padding bytes

		for (uint8_t i = 2; i < length - 2; i++) { //loop through array of data starting at position 3 to exclude A5A5 header and excluding final word which is CRC
			crc += rxBuf[i];
		}
		return crc;
	}


	void set_crc(uint8_t * rxBuf, uint8_t length) {
		if (rxBuf[0] == 0xCC) { //short message type
			rxBuf[1] = shortcrc(rxBuf, length);
		} else { //long message type
			uint16_t crc = longcrc(rxBuf, length);
			length -= (rxBuf[3] & 0xF); //subtract padding bytes
			rxBuf[length - 2] = (crc & 0xff00) >> 8;
			rxBuf[length - 1] = crc & 0xff;
		}
	}

private:
	XGTSerial serial1;
	uint16_t charge_count=0;
	float cell_voltages[10]= {4.0f};
	uint8_t temperature_histogram[6];
	uint8_t current_histogram[6];
	float pack_temperature[2];
	float capacity=6000;
	float remaining_capacity=100;
	float charge_pct=100;
};

#endif XGT_BMS_H