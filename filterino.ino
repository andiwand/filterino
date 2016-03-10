#include <stdint.h>
#include <limits.h>
#include <EEPROM.h>


#define VERSION_MAJOR 0
#define VERSION_MINOR 0

//#define USE_EEPROM

#define CONFIG_MAGIC 0x616e6469
#define CONFIG_START 0
#define CONFIG_SIZE sizeof(config)

#define SERIAL_TYPE_MESSAGE         0
#define SERIAL_TYPE_RECEIVED        1
#define SERIAL_TYPE_SEND_REQUEST    2
#define SERIAL_TYPE_SEND_RESPONSE   3
#define SERIAL_MESSAGE_TYPE_DEBUG   0
#define SERIAL_MESSAGE_TYPE_ERROR   1
#define SERIAL_MAX_LENGTH           512

#define RADIO_MAX_LOCK_TIME 5000

#define MAC_NONE    0
#define MAC_CSMA    1
#define MAC_CSMACD  2


typedef void (*received_cb_t) (uint8_t repeat, uint16_t* times, uint16_t length);
typedef void (*sent_cb_t) ();

typedef struct config_t {
  struct {
    uint16_t receiveBuffer = 256;
    uint16_t sendBuffer = 265;

    uint8_t  mac = MAC_CSMA;

    int8_t   pinReceive   = 2;
    int8_t   pinSend      = 4;
    uint16_t minTime      = 50;
    uint16_t minGap       = 3000;
    uint8_t  minPulses    = 40;
    uint16_t maxErrorAbs  = 100;

    uint8_t  waitReceivedFactor     = 5;
    uint16_t waitCollisionMin       = 200;
    uint16_t waitCollisionUnit      = 50;
    uint8_t  waitCollisionMaxUnits  = 5;
  } radio;

  struct {
    uint32_t baud         = 115200;
    uint16_t timeoutBase  = 1000;
  } serial;

  struct {
    uint16_t evaluationInterval = 1000;
  } statistics;
};

typedef struct state_t {
  struct {
    uint16_t* receiveBuffer   = NULL;
    uint16_t  receiveIndex    = 0;
    uint32_t  receiveDuration = 0;

    bool      send          = false;
    uint8_t   sendRepeat    = 0;
    uint16_t* sendBuffer    = NULL;
    uint16_t  sendLength    = 0;
    uint32_t  sendLockTime  = 0;

    received_cb_t receivedCallback  = NULL;
    sent_cb_t     sentCallback      = NULL;
  } radio;

  struct {
    uint32_t loopsCounter = 0;
    uint32_t isrsCounter  = 0;
    uint32_t lastEvaluation = 0;

    uint32_t loops  = 0;
    uint16_t isrs   = 0;
  } statistics;
};


config_t config;
state_t state;


void setup();
void loop();
void received(uint8_t repeat, uint16_t* times, uint16_t length);
void sent();

void configInit();
bool configLoad();
void configSave();

void statisticInit();
void statisticLoop();
void statisticEvaluate();

void radioInit(received_cb_t receivedCallback, sent_cb_t sentCallback);
void radioLoop();
void radioIsr();
void radioPulse(uint16_t time);
void radioFrame();
void radioLock(uint16_t time);
uint32_t radioCheckTimes(uint32_t a, uint32_t b);
bool radioSend(const uint16_t* times, uint16_t length, uint8_t repeat);
bool radioSendImpl();
bool radioSendImpl(int8_t out, int8_t in, const uint16_t* times, uint16_t length, uint8_t repeat);
void radioSendImpl(int8_t out, const uint16_t* times, uint16_t length);
bool radioSendImpl(int8_t out, int8_t in, const uint16_t* times, uint16_t length);

void serialInit();
void serialLoop();
void serialHandle(uint16_t length);
void serialHandleSendRequest(uint16_t& length);
uint16_t serialSkip(uint16_t n);
bool serialRead(void* buffer, uint16_t length);
void serialWriteMessage(uint8_t type, uint8_t code, const uint8_t* extra, uint16_t extraLength);
void serialWriteReceived(uint8_t repeat, const uint16_t* times, uint16_t length);

uint32_t timeDiff(uint32_t a, uint32_t b);
uint32_t absDiff(uint32_t a, uint32_t b);


void setup() {
  configInit();
  statisticInit();
  serialInit();
  radioInit(received, sent);
}

void loop() {
  statisticLoop();
  serialLoop();
  radioLoop();
}

void received(uint8_t repeat, uint16_t* times, uint16_t length) {
  serialWriteReceived(repeat, times, length);
}

void sent() {

}


void configInit() {
  if (!configLoad()) configSave();
}

#ifdef USE_EEPROM
bool configLoad() {
  uint16_t address = CONFIG_START;

  uint32_t magic;
  EEPROM.get(address, magic);
  if (magic != CONFIG_MAGIC) return false;
  address += sizeof(magic);

  EEPROM.get(address, config);
  address += sizeof(config);

  return true;
}

void configSave() {
  uint16_t address = CONFIG_START;

  uint32_t magic = CONFIG_MAGIC;
  EEPROM.put(address, magic);
  address += sizeof(magic);

  EEPROM.put(address, config);
  address += sizeof(config);
}
#else
bool configLoad() {
  return false;
}
void configSave() {}
#endif


void statisticInit() {
  state.statistics.lastEvaluation = micros();
}

void statisticLoop() {
  uint32_t now = micros();
  uint32_t time = timeDiff(now, state.statistics.lastEvaluation);
  time /= 1000;

  if (time > config.statistics.evaluationInterval) {
    // TODO: maybe remove interrupt lock
    noInterrupts();
    statisticEvaluate(time);
    state.statistics.lastEvaluation = now;
    interrupts();
  }
}

void statisticEvaluate(uint16_t time) {
  float seconds = time / 1000.0;
  state.statistics.loops = (uint32_t) (state.statistics.loopsCounter / seconds);
  state.statistics.isrs = (uint16_t) (state.statistics.isrsCounter / seconds);

  // TODO: notify
  //String extra = String("") + "statistics - loops: " +
  //               state.statistics.loops + ", isrs: " + state.statistics.isrs;
  //serialWriteMessage(0, 0, (const uint8_t*) extra.c_str(), extra.length());

  state.statistics.loopsCounter = 0;
  state.statistics.isrsCounter = 0;
}


void radioInit(received_cb_t receivedCallback, sent_cb_t sentCallback) {
  state.radio.receivedCallback = receivedCallback;
  state.radio.sentCallback = sentCallback;

  if (config.radio.pinReceive >= 0) {
    state.radio.receiveBuffer = (uint16_t*) malloc(config.radio.receiveBuffer);

    pinMode(config.radio.pinReceive, INPUT);
    attachInterrupt(digitalPinToInterrupt(config.radio.pinReceive), radioIsr, CHANGE);
  }

  if (config.radio.pinSend >= 0) {
    state.radio.sendBuffer = (uint16_t*) malloc(config.radio.sendBuffer);
    state.radio.sendLockTime = 0;

    pinMode(config.radio.pinSend, OUTPUT);
  }
}

void radioLoop() {
  state.statistics.loopsCounter++;

  if (state.radio.send) {

  }
}

void radioIsr() {
  state.statistics.isrsCounter++;

  static uint32_t lastTime = 0;
  uint32_t newTime = micros();
  uint32_t time = timeDiff(newTime, lastTime);

  radioPulse(time);

  lastTime = newTime;
}

void radioPulse(uint16_t time) {
  if (state.radio.receiveIndex >= config.radio.receiveBuffer) {
    state.radio.receiveIndex = 0;
    state.radio.receiveDuration = 0;
  }
  if (time < config.radio.minTime) return;

  state.radio.receiveBuffer[state.radio.receiveIndex++] = time;
  state.radio.receiveDuration += time;

  if (state.radio.receiveIndex > config.radio.minPulses & time > config.radio.minGap) {
    radioFrame();
    state.radio.receiveIndex = 0;
    state.radio.receiveDuration = 0;
  }
}

void radioFrame() {
  static uint32_t index = 0;
  static uint16_t lastLength = 0;
  static uint16_t lastGap = 0;

  if (state.radio.receiveIndex == lastLength &
      radioCheckTimes(state.radio.receiveBuffer[state.radio.receiveIndex - 1], lastGap)) {
    state.radio.receivedCallback(3, state.radio.receiveBuffer, state.radio.receiveIndex);
    radioLock(state.radio.receiveDuration * config.radio.waitReceivedFactor);
  }

  lastLength = state.radio.receiveIndex;
  lastGap = state.radio.receiveBuffer[state.radio.receiveIndex - 1];
}

void radioLock(uint16_t time) {
  if (config.radio.mac == MAC_NONE) return;
  uint32_t end = millis() + time;
  if (end <= state.radio.sendLockTime) return;
  state.radio.sendLockTime = end;
}

bool radioIsLocked() {
  if (config.radio.mac == MAC_NONE) return false;
  uint32_t time = timeDiff(state.radio.sendLockTime, millis());
  return time < RADIO_MAX_LOCK_TIME;
}

uint32_t radioCheckTimes(uint32_t a, uint32_t b) {
  unsigned int error = absDiff(a, b);
  return error <= config.radio.maxErrorAbs;
}

bool radioSend(uint16_t length, uint8_t repeat) {
  if (config.radio.pinSend < 0) return false;
  if (state.radio.send) return false;
  if (length > config.radio.sendBuffer) return false;

  state.radio.send = true;
  state.radio.sendLength = length;
  state.radio.sendRepeat = repeat;
  return true;
}

bool radioSendImpl() {
  if (radioIsLocked()) return false;

  noInterrupts();
  bool success = radioSendImpl(config.radio.pinSend,
                               config.radio.pinReceive,
                               state.radio.sendBuffer,
                               state.radio.sendLength,
                               state.radio.sendRepeat);
  interrupts();

  if (!success) {
    uint8_t unit = random(config.radio.waitCollisionMaxUnits);
    uint16_t time = config.radio.waitCollisionMin;
    time += config.radio.waitCollisionUnit * unit;
    radioLock(time);
    return false;
  }

  if (state.radio.sentCallback != NULL)
    state.radio.sentCallback();
  state.radio.send = false;
  return true;
}

bool radioSendImpl(int8_t out, int8_t in, const uint16_t* times, uint16_t length, uint8_t repeat) {
  if (in >= 0 & config.radio.mac == MAC_CSMACD) {
    for (uint8_t i = 0; i < repeat; i++) {
      if (!radioSendImpl(out, in, times, length)) return false;
    }
  } else {
    for (uint8_t i = 0; i < repeat; i++) {
      radioSendImpl(out, times, length);
    }
  }

  return true;
}

void radioSendImpl(int8_t out, const uint16_t* times, uint16_t length) {
  for (uint16_t t = 0; t < length; t++) {
    bool c = !(t & 1);
    digitalWrite(out, c);
    delayMicroseconds(times[t]);
  }
  digitalWrite(out, 0);
}

bool radioSendImpl(int8_t out, int8_t in, const uint16_t* times, uint16_t length) {
  uint32_t sent;
  bool last, first;
  uint16_t r;

  first = true;
  last = digitalRead(in);
  r = 0;

  for (uint16_t t = 0; t < length; t++) {
    uint16_t time = times[t] - 11;
    bool c = !(t & 1);
    digitalWrite(out, c);
    sent = micros();

    do {
      if (digitalRead(in) != last) {
        if (first) first = false;
        else if (r++ > t) goto collision;
        last = !last;
      }
    } while (timeDiff(micros(), sent) < time);
  }

  digitalWrite(out, 0);
  return true;
collision:
  digitalWrite(out, 0);
  return false;
}


void serialInit() {
  Serial.begin(config.serial.baud);
  Serial.setTimeout(config.serial.timeoutBase);
}

void serialLoop() {
  uint16_t length;
  if (Serial.available() < sizeof(length)) return;
  if (!serialRead(&length, sizeof(length))) return;

  if (length > SERIAL_MAX_LENGTH) {
    // TODO: error
    serialSkip(length);
    return;
  }

  serialHandle(length);
}

void serialHandle(uint16_t length) {
  uint8_t type;
  if (!serialRead(&type, sizeof(type))) return;
  length -= sizeof(type);

  switch (type) {
    case SERIAL_TYPE_SEND_REQUEST:
      serialHandleSendRequest(length);
      break;
    case SERIAL_TYPE_MESSAGE:
    case SERIAL_TYPE_RECEIVED:
    case SERIAL_TYPE_SEND_RESPONSE:
    default:
      // TODO: error
      break;
  }

  if (length > 0) serialSkip(length);
}

void serialHandleSendRequest(uint16_t& length) {
  uint8_t repeat;
  if (!serialRead(&repeat, sizeof(repeat))) return;
  length -= sizeof(repeat);
  uint16_t time;
  if (length % sizeof(time) != 0) return;

  uint16_t len = length / sizeof(time);
  bool success = radioSend(len, repeat);
  if (!success) return;

  for (uint16_t i = 0; i < len; i++) {
    if (!serialRead(&time, sizeof(time))) return;
    length -= sizeof(time);
    state.radio.sendBuffer[i] = time;
  }
}

uint16_t serialSkip(uint16_t n) {
  uint32_t last = millis();
  uint32_t now;
  uint16_t i = 0;
  while (i < n) {
    now = millis();
    if (timeDiff(now, last) >= config.serial.timeoutBase) break;

    if (Serial.read() != -1) {
      last = now;
      i++;
    }
  }
  return i;
}

bool serialRead(void* buffer, uint16_t length) {
  uint16_t read = Serial.readBytes((uint8_t*) buffer, length);
  return read == length;
}

void serialWriteHead(uint16_t length, uint8_t type) {
  length += sizeof(type);
  Serial.write((uint8_t*) &length, sizeof(length));
  Serial.write((uint8_t*) &type, sizeof(type));
}

void serialWriteMessage(uint8_t type, uint8_t code, const uint8_t* extra, uint16_t extraLength) {
  uint16_t length = sizeof(type) + sizeof(code) + extraLength;
  if (length > SERIAL_MAX_LENGTH) {
    // TODO: error
    return;
  }
  serialWriteHead(length, SERIAL_TYPE_MESSAGE);

  Serial.write((uint8_t*) &type, sizeof(type));
  Serial.write((uint8_t*) &code, sizeof(code));
  Serial.write((uint8_t*) extra, sizeof(uint8_t) * extraLength);

  Serial.flush();
}

void serialWriteReceived(uint8_t repeat, const uint16_t* times, uint16_t length) {
  uint16_t packetLength = sizeof(repeat) + sizeof(uint16_t) * length;
  if (length > SERIAL_MAX_LENGTH) {
    // TODO: error
    return;
  }
  serialWriteHead(packetLength, SERIAL_TYPE_RECEIVED);

  Serial.write((uint8_t*) &repeat, sizeof(repeat));
  Serial.write((uint8_t*) times, sizeof(uint16_t) * length);

  Serial.flush();
}


uint32_t timeDiff(uint32_t a, uint32_t b) {
  if (b > a) return ULONG_MAX - b + a;
  return a - b;
}

uint32_t absDiff(uint32_t a, uint32_t b) {
  if (a >= b) return a - b;
  return b - a;
}

