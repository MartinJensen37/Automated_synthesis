/*
   Syringe Pump Code.

   Either choose the ESP32 Dev Module or the NodeMCU-32S board.
*/
#include "EspMQTTClient.h"
/*
   INTERNAL UNITS
     length: mm
     volume: uL
     time: min
     flowrate: uL/min
*/
EspMQTTClient client(
  "PolyBotNet",
  "SwarmBot",
  "192.168.1.100",  // MQTT Broker server ip
  "",   // Can be omitted if not needed
  "",   // Can be omitted if not needed
  "SyringePump"      // Client name that uniquely identify your device
);
/*
   ESP-32S Specifics
*/
// CPU Freqency for the chip
const float kCPUFrequency = 80000000.0; // Hz
// Prescaler for the timer
const float kTimerPrescaler = 80.0;
// the number of ticks per second given the above prescaler
const float kTimerFrequency = kCPUFrequency / kTimerPrescaler;


/*
   STEPPER MOTOR
*/
// number of steps for the stepper motor to complete one revolution
const float kStepsPerRotation = 200;
// the number of mm that the T8 rod enables in one revolution
const float kLengthPerRotation = 8;
// microstepping settings
const float kTicksPerStep = 32;
// the conversion factor between steps and length
const float kStepsPerLength = kStepsPerRotation / kLengthPerRotation;
// signal for forward direction
const byte kDirectionForward = LOW;
// signal for backward direction
const byte kDirectionBackward = HIGH;

/*
   SYRINGE
*/
// the measured length per volume for the attached syringe
//const float kLengthPerVolume = 40.0 / 7000.0; // mm / uL (large pipette)
const float kLengthPerVolume = 40.0 / 5000.0; // mm / uL (small pipette)
// the maximum pumpable volume (in uL) for the attached syringe
const float kSyringeMaxPumpableVolume = 12000.0;
// the maximum flowrate (in uL/min) for the attached syringe
const float kMaxFlowRate = 10000.0 * 60.0 / 10.0; // 10 mL in 10 secs
// the minimum flowrate (in uL/min)
const float kMinFlowRate = 1000.0;
// the conversion coefficient between volume (or flowrate) and ticks (or ticks per minute)
const float kSyringeCoefficient = kLengthPerVolume * kStepsPerLength * kTicksPerStep;
/*
   VARIABLES
*/
byte pump_direction = kDirectionForward;
uint64_t tick_limit;
volatile uint64_t tick_counter;
volatile bool state;
float user_flowrate = kMaxFlowRate;
volatile bool pump_ready = false;

/*
 * PINS
 */
const int kPinLed = 2;
const int kPinStepperDirection = 18;
const int kPinStepperStep = 19;
const int kPinBackwardStop = 16;
const int kPinForwardStop = 17;


hw_timer_t* pump_timer = NULL;
portMUX_TYPE pump_timerMux = portMUX_INITIALIZER_UNLOCKED;
uint64_t status_millis;

/*
 * Performs various actions in relation to stepping the motor
 */
void IRAM_ATTR OnTick() {
  digitalWrite(kPinStepperStep, HIGH);
  if (tick_limit == tick_counter - 1) DisableTimerInterrupts();
  if (pump_direction == kDirectionForward && digitalRead(kPinForwardStop) == LOW) DisableTimerInterrupts();
  if (pump_direction == kDirectionBackward && digitalRead(kPinBackwardStop) == LOW) DisableTimerInterrupts();
  state = !state;
  delayMicroseconds(10);
  digitalWrite(kPinStepperStep, LOW);
  digitalWrite(kPinLed, state);
  tick_counter++;
}

void setup() {
  pinMode(kPinBackwardStop, INPUT_PULLUP);
  pinMode(kPinForwardStop, INPUT_PULLUP);
  pinMode(kPinLed, OUTPUT);
  pinMode(kPinStepperDirection, OUTPUT);
  pinMode(kPinStepperStep, OUTPUT);

  Serial.begin(115200);

  pump_timer = timerBegin(0, kTimerPrescaler, true);
  timerAttachInterrupt(pump_timer, &OnTick, true);

  // MQTT client library settings
  client.enableDebuggingMessages();

  Serial.println("Booted");
  status_millis = millis();
  DisableTimerInterrupts();
}

void loop() {
  client.loop();
  
  if (client.isMqttConnected() && !pump_ready) {
    pump_ready = true;
    client.publish("pump/status", "1");
  }
}

void Empty() {
  Empty(kSyringeMaxPumpableVolume);
}

void Empty(float volume) {
  Pump(volume, kMaxFlowRate, kDirectionForward);
}

void Empty(float volume, float flowrate) {
  Pump(volume, flowrate, kDirectionForward);
}

void Fill() {
  Fill(kSyringeMaxPumpableVolume);
}

void Fill(float volume) {
  Pump(volume, kMaxFlowRate, kDirectionBackward);
}

void Fill(float volume, float flowrate) {
  Pump(volume, flowrate, kDirectionBackward);
}

void Limits() {
  Empty(); pump_ready = true;
  Fill(); pump_ready = true;
  Empty();
}

void Rinse() {
  for (int i=0; i<2; i++) {
    Empty(); pump_ready = true;
    Fill(); pump_ready = true;
  }
  Empty();
}

/*
 * Pumps a volume (in uL) at a flowrate (in uL/min) in a
 * direction (either kDirectionForward or kDirectionBackward).
*/
void Pump(float volume, float flowrate, byte direction) {

  // communicate to external listener that we are NOT ready
  // to recieve new commands for now
  if (!pump_ready) {
    return;
  }
  
  if (client.isMqttConnected() && pump_ready) {
    pump_ready = false;
    client.publish("pump/status", "0");
  }
  
  // set direction
  pump_direction = direction;
  digitalWrite(kPinStepperDirection, pump_direction);

  // volume to ticks
  tick_counter = 1;
  tick_limit = (uint64_t)(kSyringeCoefficient * volume + 0.5);

  // flowrate to ticks
  float ticks_per_second = kSyringeCoefficient * flowrate / 60.0;
  if (ticks_per_second < 100) {
    Serial.println("Flowrate is too low. Aborting.");
    return;
  }
  
  // compute timer interval
  float timer_f = kTimerFrequency / ticks_per_second;
  uint64_t timer_count = (uint64_t)(timer_f + 0.5);
  timerAlarmWrite(pump_timer, timer_count, true);
  ActivateTimerInterrupts();

  uint64_t next_millis = millis();
  uint64_t actual_counter;
  while (timerAlarmEnabled(pump_timer)) {
    if (next_millis <= millis()) {
      portENTER_CRITICAL_ISR(&pump_timerMux);
      actual_counter = tick_counter;
      portEXIT_CRITICAL_ISR(&pump_timerMux);
      next_millis += 1000;
    }
  }
  state = LOW;
  digitalWrite(kPinLed, state);
}

/*
   Activates pump timer
*/
void ActivateTimerInterrupts() {
  timerAlarmEnable(pump_timer);
}

/*
   Disables pump timer
*/
void DisableTimerInterrupts() {
  timerAlarmDisable(pump_timer);
}

/*
 * MQTT responses to events from the broker
 */
void onConnectionEstablished() {
  client.subscribe("pump/settings/flowrate", [](const String & payload) {
    float f_payload = payload.toFloat();
    if (f_payload > kMinFlowRate && f_payload < kMaxFlowRate) {
      user_flowrate = payload.toFloat();
    }
  });

  client.subscribe("pump", [](const String & payload) {
    if (payload.equals("fill")) { Fill(); }
    if (payload.equals("empty")) { Empty(); }
    if (payload.equals("limits")) { Limits(); }
    if (payload.equals("rinse")) { Rinse(); }
  });

  client.subscribe("pump/fill", [](const String & payload) {
    float volume = payload.toFloat();
    Fill(volume, user_flowrate);
  });

  client.subscribe("pump/empty", [](const String & payload) {
    float volume = payload.toFloat();
    Empty(volume, user_flowrate);
  });
}
