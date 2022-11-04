/*
not gonna use serial, use pulse since analog is only in .5s increments? and seems more reliable
*/

#include <SoftwareSerial.h>

//   MaxSonar-EZ pins
#define MSE_PW_PIN 5 //pw 
#define MSE_READ_MODE_PIN 4 //rx
#define MSE_SDATA_PIN 7 //tx but only used for chaining?

//  GPIO Pins (analog)
#define MSE_AN_PIN A0 //analog

#define VCC 5

// MaxSonar to Arduino control/adjustments
#define MSE_READ_CONTINUOUS HIGH 
//low read idle is trigger, high is continuous
#define MSE_READ_IDLE LOW
#define MSE_READ_TRIGGER HIGH
#define MSE_READ_REQUIRED_DURATION_mS 50
#define MSE_PW_START_DELAY_MSE_uS 3000
#define MSE_PW_uS_PER_INCH 147.46
#define MSE_PW_uS_PER_CM 73.73
#define MSE_ANALOG_DIVISOR 2
//  Speed of Sound @ 22°C (71.6°F) = 344.5 m/S = 34450.0 cm/S
//  0.0000290275 S/cm = 29.0275762 uS/cm = 73.73 uS/in
#define SOUND_SPEED_uSpIN 73.73

// scaling formula: [(Vcc/1024) = Vi] = volts per 5 mm
/*
- Outputs analog voltage with a scaling factor of (Vcc/512) per inch. A supply of 5V yields ~9.8mV/in. and
3.3V yields ~6.4mV/in. The output is buffered and corresponds to the most recent range data.
*/
// range formula: [5*(Vm/Vi) = Ri]

SoftwareSerial mseSerial(MSE_SDATA_PIN, 3, true); // RX,TX,inverse_logic (TX is not used)
int _loopCount = 0; // Used to track the times through loop
int _voltage_scale;
double prevMeasurement;
double displacementArr[58]; //58 since measurements taken ever 51ms, and aroudn 58 of these in 3 sec

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400); // Open serial communications
  mseSerial.begin(9600); // Set the rate for MaxSonar communications
  mseSerial.listen();
  // Wait for the serial port and allow time for the MaxSonar to initialize (>500mS from power-up)
  while (!Serial || millis() < 500) {
    ; // wait...
  }
  pinMode(MSE_PW_PIN, INPUT);
  pinMode(MSE_AN_PIN, INPUT);
  digitalWrite(MSE_READ_MODE_PIN, MSE_READ_IDLE); // Use 'triggered' mode
  pinMode(MSE_READ_MODE_PIN, OUTPUT);
//  startMillis = millis();
//  _voltage_scale = VCC / 1024; //0.009766V per inch
}

void loop() {
// Trigger a measurement and read the distance using the pulse-width method
  // then the serial and analog methods
  unsigned long startMillis = millis();
  double pwDistance = triggerAndReadDistanceFromPulse();
  if (_loopCount++ % 20 == 0) {
    Serial.println("Pulse:\tAnalog:\t"); 
  }
  Serial.print(pwDistance);
  Serial.print("\t");
  Serial.print(analogDistance);
  Serial.println("\t");

  if(prevMeasurement == NULL){
    prevMeasurement = pwDistance;
  }
//  unsigned long endMill = millis();

  double diff = pwDistance - prevMeasurement;
  double diffTime = endMill - startMillis;
  Serial.print(diff);
  Serial.print("\t");
  Serial.println(diffTime);
  Serial.println(" ");
  prevMeasurement = pwDistance;
//  startMillis = endMill;
//  **PRINT SOMETHING** 
  delay(250 - MSE_READ_REQUIRED_DURATION_mS);
}

/**
 * To assure that the distance pulse is detected going from low to high,
 * this triggers the MaxSonar to read and then measures the pulse.
 *
 * The analog input is held after a reading and the serial will be
 * held in the serial input buffer since it is only 6 characters.
 *
 * Refer to the Timing Diagram and Description in the datasheet for timing details.
 */
double triggerAndReadDistanceFromPulse() {
  // Assure that the trigger is set to IDLE (HOLD)
  digitalWrite(MSE_READ_MODE_PIN, MSE_READ_IDLE);
  delayMicroseconds(10);
  //
  // Minimum time for readings (start of reading to start of reading)
  // is 49mS.
  //
  // To guarentee readings are not taken too quickly this method makes
  // sure that it takes at least this minimum time before returning by
  // recording the start time and waiting for the minimum time to have elapsed.
  //
  unsigned long startMillis = millis();

  // Trigger and measure
  digitalWrite(MSE_READ_MODE_PIN, MSE_READ_TRIGGER);
  delayMicroseconds(10);
  unsigned long pulseWidth = pulseIn(MSE_PW_PIN, HIGH, (MSE_PW_START_DELAY_MSE_uS + (MSE_READ_REQUIRED_DURATION_mS * 1000L)));
  double distance = (double)(pulseWidth / MSE_PW_uS_PER_INCH);
  digitalWrite(MSE_READ_MODE_PIN, MSE_READ_IDLE);
 
  // Wait for the minimum time to pass (first check for overflow)
  if (millis() < startMillis) {
    startMillis = millis();
  }
  while (millis() - startMillis < MSE_READ_REQUIRED_DURATION_mS) {
    delay(1);
  } 
  return distance;
}
