#include <EEPROM.h>
#include "M5DinMeter.h"
#include <XGTSerial.h>


// pins used by this application, change as needed
#define MAKITAPIN 2
#define TURNOFF_SECONDS 60  // seconds without input before the screen turns off


XGTSerial serial1(MAKITAPIN);  // RX, TX

void get_model(char* out) {
  uint8_t rxLength = 0;
  int8_t error = 0;
  uint8_t buffer[32];
  uint8_t cmd[] = { 0xA5, 0xA5, 0x00, 0x1A, 0x50, 0x2B, 0x4D, 0x4C, 0x00, 0xCB, 0x13, 0x07, 0x00, 0x06, 0x00, 0x03, 0x00, 0x01, 0x13, 0x0B, 0x02, 0x3B, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  memset(buffer, 0, 32);
  int i = 0;
  for (; i < 9 && buffer[0] == 0; i++) {
    serial1.write(cmd, 32);  //write command to battery
    serial1.read(buffer, 32);
  }

  if (i != 9) {
    int idx = 32 - ((buffer[3] & 0xF) + 3);
    for (int i = 0; i < 8; i++) {
      out[i] = buffer[idx - i];
    }
  }
  out[8] = 0;
}

uint8_t shortcrc(uint8_t* rxBuf, uint8_t length) {  //calculate CRC based on data recieved from battery.
  uint16_t crc = rxBuf[0];

  for (uint8_t i = 2; i < length; i++) {  //loop through array of data starting at position 3 to exclude CRC
    crc += (rxBuf[i]);                    //sum bytes in packet
  }

  return crc;
}

void send_cmd(uint8_t* rpy, uint8_t cmd, uint8_t num_args, uint8_t* args) {
  uint8_t buffer[] = { 0xCC, 0x00, cmd, 0x00, 0x00, 0x00, 0x00, 0x33 };
  memset(rpy, 0, 8);
  memcpy(buffer + 3, args, num_args);
  buffer[1] = shortcrc(buffer, 8);

  for (int i = 0; !(rpy[0] == 0xcc && rpy[1] == shortcrc(rpy, 8)) && i < 16; i++) {
    serial1.write(buffer, 8);  //write command to battery
    serial1.read(rpy, 8);
  }

  delay(5);
}

void unlock() {
  uint8_t args[]={0x96, 0xA5};
  uint8_t args2[] = { 0x2F};
  uint8_t rpy[8];
  send_cmd(rpy,0xD9,2,args);
  send_cmd(rpy,0xD2,1,args2);
}


void temp_histogram(uint8_t * dest) {
  uint8_t args[] = { 0x00, 0xc0 };
  uint8_t rpy[8] = { 0 };
  memset(rpy, 0, 8);
  send_cmd(rpy, 0xc0, 2, args);
  dest[0]=rpy[4];
  dest[1]=rpy[5];

  args[1]=0xc2;
  send_cmd(rpy, 0xc0, 2, args);
  dest[2]=rpy[4];
  dest[3]=rpy[5];

    args[1]=0xc4;
  send_cmd(rpy, 0xc0, 2, args);
  dest[4]=rpy[4];
  dest[5]=rpy[5];
}


void current_histogram(uint8_t * dest) {
  uint8_t args[] = { 0x00, 0xd8 };
  uint8_t rpy[8] = { 0 };
  memset(rpy, 0, 8);
  send_cmd(rpy, 0xc0, 2, args);
  dest[0]=rpy[4];
  dest[1]=rpy[5];

  args[1]=0xDA;
  send_cmd(rpy, 0xc0, 2, args);
  dest[2]=rpy[4];
  dest[3]=rpy[5];

  args[1]=0xDC;
  send_cmd(rpy, 0xc0, 2, args);
  dest[4]=rpy[4];
  dest[5]=rpy[5];
}

uint16_t num_charges() {
  uint8_t args[] = { 0x00, 0x54 };
  uint8_t rpy[8] = { 0 };
  memset(rpy, 0, 8);
  send_cmd(rpy, 0xc0, 2, args);
  return rpy[4] | rpy[5] << 8;
}


float temperature_a() {
  uint8_t args[] = { 0x03, 0x1A };
  uint8_t rpy[8] = { 0 };
  memset(rpy, 0, 8);
  send_cmd(rpy, 0xc0, 2, args);
  return ((rpy[4] | rpy[5] << 8) / 10.0f) - 273.15f;
}

float temperature_b() {
  uint8_t args[] = { 0x03, 0x1C };
  uint8_t rpy[8] = { 0 };
  memset(rpy, 0, 8);
  send_cmd(rpy, 0xc0, 2, args);
  return ((rpy[4] | rpy[5] << 8) / 10.0f) - 273.15f;
}


//gets cell voltage, 0 = pack voltage
float get_voltage(int cell_idx) {
  uint8_t args[] = { 0x03, 0x00 };
  uint8_t rpy[8] = { 0 };
  memset(rpy, 0, 8);
  args[1] = cell_idx * 2;
  send_cmd(rpy, 0xc0, 2, args);
  return ((rpy[4] | rpy[5] << 8) / 1000.0f);
}


uint8_t get_lockout() {
  uint8_t args[] = { 0x00, 0x60 };
  uint8_t rpy[8] = { 0 };
  memset(rpy, 0, 8);
  send_cmd(rpy, 0xc0, 2, args);
  return rpy[4];
}


float charge_percentage() {
  uint8_t args[] = { 0x01, 0x08 };
  uint8_t rpy[8] = { 0 };
  memset(rpy, 0, 8);
  send_cmd(rpy, 0xc0, 2, args);
  return (rpy[4] | rpy[5] << 8) / 255.0f;
}


uint16_t remaining_capacity() {
  uint8_t args[] = { 0x00, 0x64 };
  uint8_t rpy[8] = { 0 };
  memset(rpy, 0, 8);
  send_cmd(rpy, 0xc0, 2, args);
  return rpy[4] | rpy[5] << 8;
}


uint16_t cell_capacity() {
  uint8_t args[] = { 0x08 };
  uint8_t rpy[8] = { 0 };
  memset(rpy, 0, 8);
  send_cmd(rpy, 0xDD, 1, args);
  return rpy[5] *100;
}

uint8_t cell_parallel() {
  uint8_t args[] = { 0x0A };
  uint8_t rpy[8] = { 0 };
  memset(rpy, 0, 8);
  send_cmd(rpy, 0xDD, 1, args);
  return rpy[4];
}

void setup() {
  auto cfg = M5.config();
  DinMeter.begin(cfg, true);
  DinMeter.Display.setRotation(1);
  DinMeter.Display.setTextColor(GREEN);
  DinMeter.Display.setTextDatum(top_left);
  DinMeter.Display.setTextFont(&fonts::FreeMono9pt7b);

  DinMeter.Display.setTextSize(1);
  DinMeter.update();
}

void showDone() {
  DinMeter.Display.clear();
  DinMeter.Display.drawString("done.", 20, 20);
  delay(2000);
}

void loop() {
  // variables we update each loop
  static int screen = 0;
  static long old_position = DinMeter.Encoder.read();
  static int autoTurnoff = -1;
  static bool nextNewScr = true;

  if (autoTurnoff < 0) {
    autoTurnoff = millis() + (TURNOFF_SECONDS * 1000);
  }

  DinMeter.update();

  long newPosition = DinMeter.Encoder.read();
  bool btnPressed = DinMeter.BtnA.wasPressed();
  bool newScr = false;

  if (DinMeter.BtnA.pressedFor(5000) || (millis() > autoTurnoff)) {
    DinMeter.Power.powerOff();
    return;
  }

  if (abs(newPosition - old_position) > 2) {
    if (newPosition < old_position) {
      screen++;
    }

    if (newPosition > old_position) {
      screen--;
    }

    screen = screen < 0 ? 0 : screen;
    screen = screen > 4 ? 4 : screen;

    newScr = true;

    old_position = newPosition;
  }

  if (nextNewScr) {
    newScr = true;
    nextNewScr = false;
  }

  if (newScr || btnPressed) {
    autoTurnoff = millis() + (TURNOFF_SECONDS * 1000);
  }

  switch (screen) {
    case 0:
      if (btnPressed || newScr) {
        DinMeter.Display.clear();
        unlock();
        char model[10];get_model(model);
        float paralel_cells=cell_parallel();
        float cap=cell_capacity();
        DinMeter.Display.drawString("Model: " + String(model), 5, 3);
        DinMeter.Display.drawString("Charge count: " + String(num_charges()), 5, 18);
        DinMeter.Display.drawString("Lockout flag: " + String(get_lockout()), 5, 33);
        DinMeter.Display.drawString("OrigCap: " + String(cap*paralel_cells) + " MaH", 5, 48);
        DinMeter.Display.drawString("RemCap: " + String(remaining_capacity()) + " MaH", 5, 63);
        DinMeter.Display.drawString("Charge: " + String(charge_percentage()) + "%", 5, 78);
        DinMeter.Display.drawString("Cells parallel: " + String(paralel_cells), 5, 93);
        DinMeter.Display.drawString("T1:" + String(temperature_a())+"c  T2:" + String(temperature_b()), 5, 108);
      }
      break;
    case 1:
      if (btnPressed || newScr) {
        DinMeter.Display.clear();
        float voltages[11];
        float max_v=-5;
        float min_v=5;
        float vDiff=0;
        for(int i=0;i<11;i++)voltages[i]=get_voltage(i);
        for(int i=1;i<11;i++){
          max_v=max(max_v,voltages[i]);
          min_v=min(min_v,voltages[i]);
        }
        vDiff=max_v-min_v;

        DinMeter.Display.drawString("vPack: " + String(voltages[0]),5,3);
        DinMeter.Display.drawString( "vDiff: " + String(vDiff), 5, 18);
        for(int i=1;i<11;i+=2){
                  DinMeter.Display.drawString("V" + String(i)+":" + String(voltages[i]) + " " + "V" + String(i+1)+":" + String(voltages[i+1]) , 5, 33+((i>>1)*15));

        }
      }
      break;

    case 2:
      if (btnPressed || newScr) {
        DinMeter.Display.clear();
        uint8_t levels[]={0,0,0,0,0,0};
        int step_current=(cell_parallel()*30)/6;

        current_histogram(levels);

        for(int i=0;i<6;i++){
           float level=levels[i]*(90.0f/255.0f);
           int x=5 + (i*40);
           DinMeter.Display.drawRect(x, 20, 25, 90,GREEN);
           DinMeter.Display.fillRect(x, 110-level, 25, level,GREEN);
           DinMeter.Display.drawString(String((i+1) * step_current) + "A"  , x, 114);
        }
      }
      break;

    case 3:
      if (btnPressed || newScr) {
      DinMeter.Display.clear();
        uint8_t levels[]={0,0,0,0,0,0};
        int step_temp=75/6;

        temp_histogram(levels);

        for(int i=0;i<6;i++){
           float level=levels[i]*(90.0f/255.0f);
           int x=5 + (i*40);
           DinMeter.Display.drawRect(x, 20, 25, 90,GREEN);
           DinMeter.Display.fillRect(x, 110-level, 25, level,GREEN);
           DinMeter.Display.drawString(String((i+1) * step_temp) + "c"  , x, 114);
        }
      }
     break;
        case 4:
          if (newScr) {
            DinMeter.Display.clear();
            DinMeter.Display.drawString("Reset lockout.", 5, 5);
          }
          if (btnPressed) {
          unlock();
          showDone();
          nextNewScr = true;
          }
          break;
      }
  }
