#include <TimerOne.h>

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

#define PIN_OUT 7

enum PacketType   { PT_SPEED, PT_F1, PT_F2, PT_F3, PT_F4 };
enum SignalState  { SST_PACKET_1, SST_PAUSE_1, SST_PACKET_2, SST_PAUSE_2 };
enum Direction    { DIR_FWD, DIR_REV };

const byte addresses[] = { 
  192, 128, 48, 240, 176, 32, 224, 160, 12, 204,
  140, 60, 252, 188, 44, 236, 172, 8, 200, 136,
  56, 248, 184, 40, 232, 168, 3, 195, 131, 51,
  243, 179, 35, 227, 163, 15, 207, 143, 63, 255,
  191, 47, 239, 175, 11, 203, 139, 59, 251, 187,
  43, 235, 171, 2, 194, 130, 50, 242, 178, 34,
  226, 162, 14, 206, 142, 62, 254, 190, 46, 238,
  174, 10, 202, 138, 58, 250, 186, 42, 234, 170 };

struct State {
  const byte address;
  const byte binary_address;
  Direction direction;
  byte speed;
  boolean f0, f1, f2, f3, f4;

  State(const byte address) : address(address), binary_address(addresses[address - 1]), direction(DIR_FWD), speed(0), f0(false), f1(false), f2(false), f3(false), f4(false) { }
};

struct Packet {
  byte address[8];
  byte f0[2];
  byte a, e, b, f, c, g, d, h;
};

const PacketType protocol[] = { PT_SPEED, PT_SPEED, PT_F1, PT_F1, PT_SPEED, PT_SPEED, PT_F2, PT_F2, PT_SPEED, PT_SPEED, PT_F3, PT_F3, PT_SPEED, PT_SPEED, PT_F4, PT_F4 };

volatile State state = { 28 };
volatile Packet packet;
const byte * bits = (byte *) &packet;

/*
const double PULSE_FX = 1.0/38400.0;  //0.5/100000.0;
const byte PAUSE_1_PULSES = 0.001525 / PULSE_FX; // / 2;
const byte PAUSE_2_PULSES = 0.001025 / PULSE_FX; // / 2;
*/

// Address 24 = 0220 (base 3)
// 0220 = 0 - open - open - 0
// binary 1 = 0b11111110
// binary 0 = 0b10000000
// ternary 1 = 0b11111110 0b11111110 = "1" "0"
// ternary 0 = 0b10000000 0b10000000 = "0" "1"
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

  PORTD = sig ? (PORTD & 0b11111011) | 0b00010000 : (PORTD & 0b11101111) | 0b00000100;
  
  working = false;
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
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);

  Serial.println("READY");
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
