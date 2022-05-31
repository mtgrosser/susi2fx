#include <TimerOne.h>

//#define PULSE_FX      (1.0/38462)  // 26 us
//#define PULSE_FX      (1.0/47000)  // 22 us

#define PULSE_FX_US 26

//#define PAUSE_1_US    1525
//#define PAUSE_2_US    1025

//#define PAUSE_1_PULSES    (1525 / 26)
//#define PAUSE_2_PULSES    (1025 / 26)

#define PROTOCOL_MAX   8

#define PIN_OUT 7

enum PacketType   { PT_SPEED, PT_F1, PT_F2, PT_F3, PT_F4 };
enum SignalState  { SST_PACKET_1, SST_PAUSE_1, SST_PACKET_2, SST_PAUSE_2 };
enum Direction    { FWD, REV };

struct State {
  const byte address;
  Direction direction;
  byte speed;
  boolean f0, f1, f2, f3, f4;
  
  State(const byte address) : address(address), direction(FWD), speed(0), f0(false), f1(false), f2(false), f3(false), f4(false) { }
};

struct Packet {
  byte address[8];
  byte f0[2];
  byte a, e, b, f, c, g, d, h;
};

const PacketType protocol[] = { PT_SPEED, PT_F1, PT_SPEED, PT_F2, PT_SPEED, PT_F3, PT_SPEED, PT_F4 };

volatile State state = { 24 };
volatile Packet packet;
const char * bits = (char *) &packet;

const double PULSE_FX = 1.0/38400.0;  //0.5/100000.0;
const byte PAUSE_1_PULSES = 0.001525 / PULSE_FX; // / 2;
const byte PAUSE_2_PULSES = 0.001025 / PULSE_FX; // / 2;

// Address 24 = 0220 (base 3)
// 0220 = 0 - open - open - 0
// binary 1 = 0b11111110
// binary 0 = 0b10000000
// ternary 1 = 0b11111110 0b11111110 = "1" "0"
// ternary 0 = 0b10000000 0b10000000 = "0" "1"
// ternary 2 = 0b11111110 0b10000000 = "1" "0"


void updatePacket(PacketType type) {
  packet.address[0] = 0;
  packet.address[1] = 1;
  packet.address[2] = 1;
  packet.address[3] = 0;
  packet.address[4] = 1;
  packet.address[5] = 0;
  packet.address[6] = 0;
  packet.address[7] = 1;
  
  if (state.f0) {
    packet.f0[0] = 1;
    packet.f0[1] = 0;
  } else {
    packet.f0[0] = 0;
    packet.f0[1] = 1;
  }
  
  byte speed = (state.speed + 1) & 0b1111;
  if (speed == 1) speed = 0;
  packet.a = speed & 0b0001;
  packet.b = (speed & 0b0010) ? 1 : 0;
  packet.c = (speed & 0b0100) ? 1 : 0;
  packet.d = (speed & 0b1000) ? 1 : 0;
  
  switch (type) {
    
    case PT_SPEED:
      if (state.direction == FWD) {
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
  static byte currentBitValue = 0;
  static byte currentPause = 0;
  static byte protocolIndex = 0;
  static SignalState sigState = SST_PAUSE_2;
  byte sig;

  if (working) return;
  working = true;
  
  //timerStop();

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
      if (currentBitValue) {
        sig = (currentPulse < 7) ? HIGH : LOW;
      } else {
        sig = (currentPulse == 0) ? HIGH : LOW;
      }

      //if (currentPulse == 0) Serial.println();
      currentPulse++;
  
      if (currentPulse > 7) {
        currentPulse = 0;
        currentBitValue = nextBit();
      }
     
  }

  digitalWrite(LED_BUILTIN, sig);
  digitalWrite(PIN_OUT, sig);

  //Serial.print(sig);

  // timerStart();
  working = false;
}

void timerStart() {
  Timer1.start();
}

//void timerStop() {
  //FlexiTimer2::stop();
  //timer.cancel();
//}

byte nextBit() {
  static byte currentBit = 0;
  byte value;
  
  if (sigState == SST_PACKET_1 || sigState == SST_PACKET_2) {
    value = *(bits + currentBit);
    currentBit++;
    if (currentBit >= sizeof(Packet)) {
      currentBit = 0;
      sigState++;
    }
  }
  
  return value;
}


void toggle() {
  static byte ledState = 0;

  ledState = (ledState + 1) % 2;
  digitalWrite(LED_BUILTIN, ledState);
}


void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_OUT, OUTPUT);

  Serial.println("READY");
  Serial.println(PULSE_FX, 10);
  Serial.println(PAUSE_1_PULSES);
  Serial.println(PAUSE_2_PULSES);

  delay(1000);

  updatePacket(PT_SPEED);

  Timer1.initialize(PULSE_FX_US);
  Timer1.attachInterrupt(generateSignal);
  timerStart();
}

void loop() {
}
