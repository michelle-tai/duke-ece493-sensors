//not gonna use serial, use pulse since analog is only in .5s increments? and seems more reliable
#include <SoftwareSerial.h>
#include <Wire.h>

//   MaxSonar-EZ pins
#define MSE_PW_PIN 5 //pw 
#define MSE_READ_MODE_PIN 4 //rx
#define MSE_SDATA_PIN 2 //tx but only used for chaining?
//  GPIO Pins (analog)
#define MSE_AN_PIN A0 //analog

// mosfet pin
#define MOSFET_GATE_PIN1 12 // 1B Top
#define MOSFET_GATE_PIN2 11 // 1A Bottom
#define MOSFET_GATE_PIN3 10 // 2B Top
#define MOSFET_GATE_PIN4 9 // 2A Bottom

#define BUTTON_PIN 7 // will be pullup, so usually high

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
//#define WINDOW_SIZE 12 //39 //58 //3÷0.250 = 12
#define ZEROES_THRESHOLD 5 //30 // 46 
#define LOOP_DELAY 250
#define DISPLACEMENT_THRESHOLD 0.75

SoftwareSerial mseSerial(MSE_SDATA_PIN, 3, true); // RX,TX,inverse_logic (TX is not used)
double prevMeasurement;
double prevDisplacement;
//double displaceArr[WINDOW_SIZE]; //58 since measurements taken ever 51ms, and around 58 of these in 3 sec
int nearZeroCount; // threshold set to 46, which is ~80%. we can play with this later
double prevMeasurement2;
double prevDisplacement2;
//double displaceArr[WINDOW_SIZE]; //58 since measurements taken ever 51ms, and around 58 of these in 3 sec
int nearZeroCount2; // threshold set to 46, which is ~80%. we can play with this later
bool isUp = false;
bool isSensing = false;
bool justStarting = true;
int lastState = HIGH;
int currentState;
bool hasDoneFirstDistMeas = false;
bool hasDoneFirstDisplCalc = false;

void setup() {
  nearZeroCount = 0;
  nearZeroCount2 = 0;
  serialSetup();
  pinSetup();
  solenoidSetup();
  I2Csetup();
  Serial.println("done setting up");
//  sensorLoop();
  isSensing = false;
}

void pinSetup(){
  // LV Maxsonar setup
  pinMode(MSE_PW_PIN, INPUT);
  pinMode(MSE_AN_PIN, INPUT);
  digitalWrite(MSE_READ_MODE_PIN, MSE_READ_IDLE); // Use 'triggered' mode
  pinMode(MSE_READ_MODE_PIN, OUTPUT);

  pinMode(7, INPUT_PULLUP);
}

void reset(){
  hasDoneFirstDistMeas = false;
  hasDoneFirstDisplCalc = false;
  nearZeroCount = 0;
  isUp = false;
}

void loop() {
  currentState = digitalRead(7);
  Serial.print("state: ");
  Serial.println(currentState);
  if(lastState == HIGH && currentState == LOW){
    if(isUp){
      Serial.println("bringing arms down");
      isUp = false;
      isSensing = false;
      bringArmsDown();
    } 
    else{ // arms are down, so could be sensing or just not
      if(!isSensing){
        Serial.println("starting sensing in 1 second");
        isSensing = true;
        reset();
        delay(1000);
      } else {
        Serial.println("reset");
        reset(); // reset
        
        bringArmsDown();
      }
    }
  } 
  lastState = currentState;
  delay(50);
  
  while(isSensing){
    pulseSonarSensorRead();
//    I2CsensorRead();
    Serial.println("----");
    if(nearZeroCount >= ZEROES_THRESHOLD || nearZeroCount2 >= ZEROES_THRESHOLD){
      Serial.println("FAILURE DETECTED");
      Serial.print(nearZeroCount);
      Serial.print(" ");
      Serial.println(nearZeroCount2);

      Serial.println("Raising arms");
      digitalWrite(MOSFET_GATE_PIN1, LOW);
      digitalWrite(MOSFET_GATE_PIN2, HIGH);
      digitalWrite(MOSFET_GATE_PIN3, LOW);
      digitalWrite(MOSFET_GATE_PIN4, HIGH);
      isUp = true;

      delay(1000);
      Serial.println("Turning off pins");
      digitalWrite(MOSFET_GATE_PIN2, LOW);
      digitalWrite(MOSFET_GATE_PIN4, LOW);
     
      isSensing = false;
    }
    delay(LOOP_DELAY - MSE_READ_REQUIRED_DURATION_mS);
  }
  
}

void I2Csetup(){
  //uses default SCL and SDA pins on the uno
  Wire.begin();
  Wire.beginTransmission(112); //112 is the peripheral i2c address
  Wire.write(byte(81)); // 81 is read
  Wire.endTransmission(); 
//  delay(90);
}

//TODO: add other side too
void solenoidSetup(){
  pinMode(MOSFET_GATE_PIN1, OUTPUT);
  pinMode(MOSFET_GATE_PIN2, OUTPUT);
  pinMode(MOSFET_GATE_PIN3, OUTPUT);
  pinMode(MOSFET_GATE_PIN4, OUTPUT);
  
  digitalWrite(MOSFET_GATE_PIN1, HIGH);
  digitalWrite(MOSFET_GATE_PIN2, LOW);
  digitalWrite(MOSFET_GATE_PIN3, HIGH);
  digitalWrite(MOSFET_GATE_PIN4, LOW);
  
  delay(3000);
  
  digitalWrite(MOSFET_GATE_PIN1, LOW);
  digitalWrite(MOSFET_GATE_PIN3, LOW);
//  unsigned long startMillis = millis();
}

void bringArmsDown(){
  digitalWrite(MOSFET_GATE_PIN1, HIGH);
  digitalWrite(MOSFET_GATE_PIN2, LOW);
  digitalWrite(MOSFET_GATE_PIN3, HIGH);
  digitalWrite(MOSFET_GATE_PIN4, LOW);
  
  delay(1000);
  
  digitalWrite(MOSFET_GATE_PIN1, LOW);
  digitalWrite(MOSFET_GATE_PIN3, LOW);
}

int calcNearZeroCount(double previousDisplacement, double currDisplacement, int nearZeroCount){
   if(abs(previousDisplacement) < DISPLACEMENT_THRESHOLD){ // if prev was a 0
    if(abs(currDisplacement) < DISPLACEMENT_THRESHOLD){ //if curr 0, then add to zero count
      return nearZeroCount + 1;
    }
    else{ //else if curr is not 0, then set the near zero count to = 0
      return 0;
    }
  }
  else { // if prev was not 0
    if(abs(currDisplacement) < DISPLACEMENT_THRESHOLD){ // and curr is 0, reset count to 1
      return 1;
    }
    else{ // curr is not 0 too
      return 0;
    }
  } 
}

void pulseSonarSensorRead(){
  double pwDistance = triggerAndReadDistanceFromPulse();
  Serial.print("Pulse:\t");
  Serial.println(pwDistance);
  if(!hasDoneFirstDistMeas){
    prevMeasurement = pwDistance;
    hasDoneFirstDistMeas = true;
    return;
  }

  double displacement = pwDistance - prevMeasurement;
  Serial.println(prevDisplacement);
  if(!hasDoneFirstDisplCalc){
    prevDisplacement = displacement;
    hasDoneFirstDisplCalc = true;
    Serial.println("is returning");
    return;
  }
  
  Serial.print("Displ for pulse: ");
  Serial.println(displacement);
//  Serial.println(abs(prevDisplacement));
//  Serial.println(abs(displacement));

  int startMillis = millis();
//  int newZeroCount;
//  if(millis() - startMillis <= 25){
//    newZeroCount = calcNearZeroCount(prevDisplacement, displacement, nearZeroCount);
//  }
  int newZeroCount = calcNearZeroCount(prevDisplacement, displacement, nearZeroCount);
  nearZeroCount = newZeroCount; 
  Serial.println(nearZeroCount);
  prevMeasurement = pwDistance;
  prevDisplacement = displacement;
  delay(5);
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
  mseSerial.stopListening();
  mseSerial.listen();
  // Assure that the trigger is set to IDLE (HOLD)
  digitalWrite(MSE_READ_MODE_PIN, MSE_READ_IDLE);
  delayMicroseconds(10);

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

void serialSetup(){
  // put your setup code here, to run once:
  Serial.begin(38400); // Open serial communications
  mseSerial.begin(9600); // Set the rate for MaxSonar communications
  mseSerial.listen();
  // Wait for the serial port and allow time for the MaxSonar to initialize (>500mS from power-up)
  while (!Serial || millis() < 500) {
    ; // wait...
  }
}

void I2CsensorRead(){
  double i2c_range_inch;
  Wire.beginTransmission(112); //112 is the peripheral i2c address
  Wire.write(byte(81)); // 81 is read
  Wire.endTransmission();
  delay(90);
  Wire.requestFrom(112, 2);
  if (2 <= Wire.available()) { // if two bytes were received
    int reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    i2c_range_inch = reading / 2.54;
    Serial.print("I2C: ");
    Serial.println(i2c_range_inch);
  } 
  else {
    Serial.println("I2C ERROR");
    return;
  }
  
  if(prevMeasurement2 == NULL){
    prevMeasurement2 = i2c_range_inch;
    return;
  }

  double displacement2 = i2c_range_inch - prevMeasurement2;
  int newZeroCount2 = calcNearZeroCount(prevDisplacement2, displacement2, nearZeroCount2);
  nearZeroCount2 = newZeroCount2; 
  Serial.print("Displ for I2c: ");
  Serial.println(displacement2);
//  Serial.print("nearZeroCount2: ");
  Serial.println(nearZeroCount2);
  prevMeasurement2 = i2c_range_inch;
  prevDisplacement2 = displacement2;
}
