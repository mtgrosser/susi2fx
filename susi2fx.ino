#include <TimerOne.h>
#include <EEPROM.h>
#include <Rcn600.h>

/*
#define PULSE_FX      (1.0/38462)  // 26 us
//#define PULSE_FX      (1.0/47000)  // 22 us
//#define PAUSE_1_US    1525
//#define PAUSE_2_US    1025
*/

#define PULSE_FX_US 26
#define PAUSE_1_PULSES  58  // (1525 / 26)
#define PAUSE_2_PULSES  39  // (1025 / 26)

#define PROTOCOL_MAX   16

#define PIN_SIG_P 6       // control signal, positive
#define PIN_SIG_N 7       // control signal, negative
#define PIN_SUSI_CLK 3    // SUSI clock
#define PIN_SUSI_DATA 4   // SUSI data

enum PacketType   { PT_SPEED, PT_F1, PT_F2, PT_F3, PT_F4 };
enum SignalState  { SST_PACKET_1, SST_PAUSE_1, SST_PACKET_2, SST_PAUSE_2 };
enum Direction    { DIR_FWD, DIR_REV };
enum ConfigVar    { CV_ADDRESS, CV_MAX };

const byte addresses[] = {
  0xaa, 0xc0, 0x80, 0x30, 0xf0, 0xb0, 0x20, 0xe0, 0xa0, 0x0c,
  0xcc, 0x8c, 0x3c, 0xfc, 0xbc, 0x2c, 0xec, 0xac, 0x08, 0xc8,
  0x88, 0x38, 0xf8, 0xb8, 0x28, 0xe8, 0xa8, 0x03, 0xc3, 0x83, 
  0x33, 0xf3, 0xb3, 0x23, 0xe3, 0xa3, 0x0f, 0xcf, 0x8f, 0x3f, 
  0xff, 0xbf, 0x2f, 0xef, 0xaf, 0x0b, 0xcb, 0x8b, 0x3b, 0xfb, 
  0xbb, 0x2b, 0xeb, 0xab, 0x02, 0xc2, 0x82, 0x32, 0xf2, 0xb2, 
  0x22, 0xe2, 0xa2, 0x0e, 0xce, 0x8e, 0x3e, 0xfe, 0xbe, 0x2e, 
  0xee, 0xae, 0x0a, 0xca, 0x8a, 0x3a, 0xfa, 0xba, 0x2a, 0xea,

  0x00, 0x40, 0x60, 0x97, 0x70, 0x48, 0x68, 0x58, 0x78, 0x44, 
  0x64, 0x54, 0x74, 0x4c, 0x6c, 0x5c, 0x7c, 0x42, 0x62, 0x52,
  0x72, 0x4a, 0x6a, 0x5a, 0x7a, 0x46, 0x66, 0x56, 0x76, 0x4e,
  0x6e, 0x5e, 0x7e, 0x41, 0x61, 0x51, 0x71, 0x49, 0x69, 0x59,
  0x79, 0x46, 0x65, 0x9f, 0x75, 0x4d, 0x6d, 0x5d, 0x7d, 0x43,
  0x63, 0x53, 0x73, 0x4b, 0x6b, 0x5b, 0x7b, 0x47, 0x67, 0x57,
  0x77, 0x4f, 0x6f, 0x5f, 0x7f, 0x10, 0x18, 0x14, 0x1c, 0x12,
  0x1a, 0x16, 0x1e, 0x11, 0x19, 0x15, 0x1d, 0x13, 0x1b, 0x17,
  0x1f, 0xd0, 0xd8, 0xd4, 0xdc, 0xd2, 0xda, 0xd6, 0xde, 0xd1,
  0xd9, 0xd5, 0xdd, 0xd3, 0xdb, 0xd7, 0xdf, 0x90, 0x98, 0x94, 
  0x9c, 0x92, 0x9a, 0x96, 0x9e, 0x91, 0x99, 0x95, 0x9d, 0x93,
  0x9b, 0x50, 0x55, 0x04, 0x06, 0x05, 0x07, 0xc4, 0xc6, 0xc5,
  0xc7, 0x84, 0x86, 0x85, 0x87, 0x34, 0x36, 0x35, 0x37, 0xf4, 
  0xf6, 0xf5, 0xf7, 0xb4, 0xb6, 0xb5, 0xb7, 0x24, 0x26, 0x25,
  0x27, 0xe4, 0xe6, 0xe5, 0xe7, 0xa4, 0xa6, 0xa5, 0xa7, 0x01,
  0xc1, 0x81, 0x31, 0xf1, 0xb1, 0x21, 0xe1, 0xa1, 0x0d, 0xcd, 
  0x8d, 0x3d, 0xfd, 0xbd, 0x2d, 0xed, 0xad, 0x09, 0xc9, 0x89,
  0x39, 0xf9, 0xb9, 0x29, 0xe9, 0xa9 };

struct State {
  byte address;
  byte binary_address;
  Direction direction;
  byte speed;
  boolean f0, f1, f2, f3, f4;

  State(byte address) :
    address(address),
    binary_address(addresses[address]),
    direction(DIR_FWD),
    speed(0),
    f0(false),
    f1(false),
    f2(false),
    f3(false),
    f4(false) {}

  setAddress(byte newAddr) {
    address = newAddr;
    binary_address = addresses[address];
  }
};

struct Packet {
  byte address[8];
  byte f0[2];
  byte a, e, b, f, c, g, d, h;
};

const PacketType protocol[] = {
  PT_SPEED, PT_SPEED,
  PT_F1, PT_F1,
  PT_SPEED, PT_SPEED,
  PT_F2, PT_F2,
  PT_SPEED, PT_SPEED,
  PT_F3, PT_F3,
  PT_SPEED, PT_SPEED,
  PT_F4, PT_F4 };

volatile State state = { 28 };
volatile Packet packet;
const byte * bits = (byte *) &packet;

Rcn600 SUSI(EXTERNAL_CLOCK, PIN_SUSI_DATA);


/*
const double PULSE_FX = 1.0/38400.0;  //0.5/100000.0;
const byte PAUSE_1_PULSES = 0.001525 / PULSE_FX; // / 2;
const byte PAUSE_2_PULSES = 0.001025 / PULSE_FX; // / 2;
*/

// Address 24 = 0t0220 (base 3)
// 0220 = 0 - open - open - 0
// binary 1 = 0b11111110
// binary 0 = 0b10000000
// ternary 1 = 0b11111110 0b11111110 = "1" "1"
// ternary 0 = 0b10000000 0b10000000 = "0" "0"
// ternary 2 = 0b11111110 0b10000000 = "1" "0"


void updatePacket(PacketType type) {
  byte bit;

  for (bit = 0; bit < 8; bit++) {
    packet.address[bit] = state.binary_address & (1 << (7 - bit)) ? 1 : 0;
  }
  
  if (state.f0) {
    packet.f0[0] = 1;
    packet.f0[1] = 1;
  } else {
    packet.f0[0] = 0;
    packet.f0[1] = 0;
  }
  
  byte speed = (state.speed + 1) & 0b1111;
  if (speed == 1) speed = 0;
  packet.a = speed & 0b0001;
  packet.b = (speed & 0b0010) ? 1 : 0;
  packet.c = (speed & 0b0100) ? 1 : 0;
  packet.d = (speed & 0b1000) ? 1 : 0;
  
  switch (type) {
    
    case PT_SPEED:
      if (state.direction == DIR_FWD) {
        if (state.speed <= 6) {
          packet.e = 0;
          packet.f = 1;
          packet.g = 0;
          packet.h = 1;
        } else {
          packet.e = 0;
          packet.f = 1;
          packet.g = 0;
          packet.h = 0;
        }
      } else {
        if (state.speed <= 6) {
          packet.e = 1;
          packet.f = 0;
          packet.g = 1;
          packet.h = 1;
        } else {
          packet.e = 1;
          packet.f = 0;
          packet.g = 1;
          packet.h = 0;
        }
      }
      break;
      
    case PT_F1:
      packet.e = 1;
      packet.f = 1;
      packet.g = 0;
      packet.h = state.f1 ? 1 : 0;
      if (state.speed == 2 && !state.f1) {
        packet.f = 0;
        packet.g = 1;
      } else if (state.speed == 10 && state.f1) {
        packet.e = 0;
      }
      break;
      
    case PT_F2:
      packet.e = 0;
      packet.f = 0;
      packet.g = 1;
      packet.h = state.f2 ? 1 : 0;
      if (state.speed == 3 && !state.f2) {
        packet.e = 1;
      } else if (state.speed == 11 && state.f2) {
        packet.f = 1;
        packet.g = 0;
      }
      break;
      
    case PT_F3:
      packet.e = 0;
      packet.f = 1;
      packet.g = 1;
      packet.h = state.f3 ? 1 : 0;
      if (state.speed == 5 && !state.f3) {
        packet.e = 1;
        packet.f = 0;
      } else if (state.speed == 13 && state.f3) {
        packet.g = 0;
      }
      break;
      
    case PT_F4:
      packet.e = 1;
      packet.f = 1;
      packet.g = 1;
      packet.h = state.f4 ? 1 : 0;
      if (state.speed == 6 && !state.f4) {
        packet.f = 0;
      } else if (state.speed == 14 && state.f4) {
        packet.e = 0;
        packet.g = 0;
      }
      break;
 
  }
}

void generateSignal() {
  static boolean working = false;
  static byte currentPulse = 0;
  static byte currentBit = 0;
  static byte currentBitValue = 0;
  static byte currentPause = 0;
  static byte protocolIndex = PROTOCOL_MAX - 1;
  static SignalState sigState = SST_PAUSE_2;
  byte sig;

  if (working) return;
  working = true;
  
  switch (sigState) {
    case SST_PAUSE_1:
      //Serial.print("SST_PAUSE_1");
      //if (currentPause == 0) Serial.println();
      sig = LOW;
      currentPause++;
      if (currentPause >= PAUSE_1_PULSES) {
        currentPause = 0;
        sigState = SST_PACKET_2;
      }
      break;
    
    case SST_PAUSE_2:
      //Serial.print("SST_PAUSE_2");
      //if (currentPause == 0) Serial.println();
      sig = LOW;
      currentPause++;
      if (currentPause >= PAUSE_2_PULSES) {
        currentPause = 0;
        sigState = SST_PACKET_1;
        protocolIndex = (protocolIndex + 1) % PROTOCOL_MAX;
        updatePacket(protocol[protocolIndex]);
      }
      break;

    default:
      currentBitValue = *(bits + currentBit);
      
      if (currentBitValue) {
        sig = (currentPulse < 7) ? HIGH : LOW;
      } else {
        sig = (currentPulse == 0) ? HIGH : LOW;
      }

      //if (currentPulse == 0) Serial.println();
      currentPulse++;
  
      if (currentPulse > 7) {
        currentPulse = 0;

        if (sigState == SST_PACKET_1 || sigState == SST_PACKET_2) {
          currentBit++;
          if (currentBit >= sizeof(Packet)) {
            currentBit = 0;
            sigState = sigState + 1;
          }
        }
      }
     
  }

  //digitalWrite(LED_BUILTIN, sig);
  //digitalWrite(PIN_OUT, sig);

  // PORTD = sig ? (PORTD & 0b11111011) | 0b00010000 : (PORTD & 0b11101111) | 0b00000100;
  PORTD = sig ? (PORTD & ~(1 << PIN_SIG_N)) | (1 << PIN_SIG_P) : (PORTD & ~(1 << PIN_SIG_P) | (1 << PIN_SIG_N));
  
  working = false;
}


void notifySusiRealSpeed(uint8_t Speed, SUSI_DIRECTION Dir) {
  state.speed = (Speed * 14) / 127;
  state.direction = (Dir == SUSI_DIR_FWD) ? DIR_FWD : DIR_REV;
}


void notifySusiFunc(SUSI_FN_GROUP SUSI_FuncGrp, uint8_t SUSI_FuncState) {
  if (SUSI_FuncGrp == SUSI_FN_0_4) {
    state.f0 = (SUSI_FuncState & SUSI_FN_BIT_00) ? 1 : 0;
    state.f1 = (SUSI_FuncState & SUSI_FN_BIT_01) ? 1 : 0;
    state.f2 = (SUSI_FuncState & SUSI_FN_BIT_02) ? 1 : 0;
    state.f3 = (SUSI_FuncState & SUSI_FN_BIT_03) ? 1 : 0;
    state.f4 = (SUSI_FuncState & SUSI_FN_BIT_04) ? 1 : 0;
  }
}


uint8_t notifySusiCVRead(uint16_t CV) {
  return 88;
}

void cvWrite(ConfigVar cv, byte value) {
  if (cv < CV_MAX) {
    EEPROM[cv] = value;
  }
}

/*


void toggle() {
  static byte ledState = 0;

  ledState = (ledState + 1) % 2;
  digitalWrite(LED_BUILTIN, ledState);
}
*/

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  //digitalPinFast(PIN_SIG_P);
  //digitalPinFast(PIN_SIG_N);

  pinMode(PIN_SIG_P, OUTPUT);
  pinMode(PIN_SIG_N, OUTPUT);

  Rcn600(PIN_SUSI_CLK, PIN_SUSI_DATA);

  const byte address = EEPROM[CV_ADDRESS];
  if (address != 255) {
    state.setAddress(address);
  }

  Serial.println("READY");
  Serial.println(address, DEC);
  Serial.println(PULSE_FX_US, 10);
  Serial.println(PAUSE_1_PULSES);
  Serial.println(PAUSE_2_PULSES);
  Serial.println(sizeof(Packet));

  delay(1000);

  updatePacket(PT_SPEED);

  Timer1.initialize(PULSE_FX_US);
  Timer1.attachInterrupt(generateSignal);
  Timer1.start();
}

void loop() {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  byte bit;
  byte value;

  // process SUSI
  SUSI.process();

  if (currentMillis - previousMillis >= 5000) {
    previousMillis = currentMillis;
    Serial.println("# # # # # # # # * * A E B F C G D H");
    for (bit = 0; bit < sizeof(Packet); bit++) {
      value = *(bits + bit);
      Serial.print(value, DEC);
      Serial.print(" ");
    }
    Serial.println("\n");
    state.f0 = !state.f0;
    state.direction = state.direction == DIR_FWD ? DIR_REV : DIR_FWD;
    state.speed = 4 - state.speed;
    digitalWrite(LED_BUILTIN, state.direction);
    /*
    Serial.println(state.f0);
    Serial.println(state.speed);*/
    
  }
}
