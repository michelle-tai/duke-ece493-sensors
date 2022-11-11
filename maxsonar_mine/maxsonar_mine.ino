//not gonna use serial, use pulse since analog is only in .5s increments? and seems more reliable
#include <SoftwareSerial.h>

//   MaxSonar-EZ pins
#define MSE_PW_PIN 5 //pw 
#define MSE_READ_MODE_PIN 4 //rx
#define MSE_SDATA_PIN 7 //tx but only used for chaining?
//  GPIO Pins (analog)
#define MSE_AN_PIN A0 //analog

// mosfet pin
#define MOSFET_GATE_PIN1 12
#define MOSFET_GATE_PIN2 11

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
#define ZEROES_THRESHOLD 12 //30 // 46 
#define LOOP_DELAY 250

SoftwareSerial mseSerial(MSE_SDATA_PIN, 3, true); // RX,TX,inverse_logic (TX is not used)
double prevMeasurement;
double prevDisplacement;
//double displaceArr[WINDOW_SIZE]; //58 since measurements taken ever 51ms, and around 58 of these in 3 sec
int nearZeroCount; // threshold set to 46, which is ~80%. we can play with this later
int winStart;
int winEnd;
double winStartVal; 
double winEndVal;

void setup() {
  nearZeroCount = 0;
  winStart = 0;
  winEnd = 0;
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
  solenoidSetup();
  sensorLoop();
}

void loop() {
  
}

void solenoidSetup(){
  pinMode(MOSFET_GATE_PIN1, OUTPUT);
  digitalWrite(MOSFET_GATE_PIN1, HIGH);
  pinMode(MOSFET_GATE_PIN2, OUTPUT);
  digitalWrite(MOSFET_GATE_PIN2, LOW);
  delay(3000);
  digitalWrite(MOSFET_GATE_PIN1, LOW);
}


void sensorLoop(){
  while(true){
    double pwDistance = triggerAndReadDistanceFromPulse();
    Serial.print("Pulse:\t");
    Serial.println(pwDistance);
  
    if(prevMeasurement == NULL){
      prevMeasurement = pwDistance;
      continue;
    }

    double displacement = pwDistance - prevMeasurement;
    if(prevDisplacement == NULL){
      prevDisplacement = displacement;
      continue;
    }
    
    Serial.print("Displ:\t");
    Serial.println(displacement);
//    updateWindow(displacement);
    Serial.println(abs(prevDisplacement));
    Serial.println(abs(displacement));

    if(abs(prevDisplacement) < 1){ // if prev was a 0
      if(abs(displacement) < 1){ //if curr 0, then add to zero count
        nearZeroCount++;
      }
      else{ //else if curr is not 0, then set the near zero count to = 0
        nearZeroCount = 0;
      }
    }
    else { // if prev was not 0
      if(abs(displacement) < 1){ // and curr is 0, reset count to 1
        nearZeroCount = 1;
      }
      else{ // curr is not 0 too
        nearZeroCount = 0;
      }
    } 
    Serial.println(nearZeroCount);



    if(nearZeroCount >= ZEROES_THRESHOLD){
      Serial.println("FAILURE DETECTED");
      Serial.println(nearZeroCount);
      
      digitalWrite(MOSFET_GATE_PIN2, HIGH);
      delay(5000);
      digitalWrite(MOSFET_GATE_PIN2, LOW);
      delay(2000);
      digitalWrite(MOSFET_GATE_PIN1, HIGH);
      delay(5000);
      digitalWrite(MOSFET_GATE_PIN1, LOW);
      break;
    }
    prevMeasurement = pwDistance;
    prevDisplacement = displacement;
    delay(LOOP_DELAY - MSE_READ_REQUIRED_DURATION_mS);
  }
  
}

//void zeroCheck(){
//  double pwDistance = triggerAndReadDistanceFromPulse();
//  if(prevMeasurement == NULL){
//    prevMeasurement = pwDistance;
//    continue;
//  }
//  
//  double displacement = pwDistance - prevMeasurement;
//  // if see 0 & previous was 0
//  if(abs(prevMeasurement) < 1){ // if prev was a 0
//    if(abs(displacement) < 1){ //if curr 0, then add to zero count
//      nearZeroCount++;
//    }
//    else{ //else if curr is not 0, then set the near zero count to = 0
//      nearZeroCount = 0;
//    }
//  }
//  else { // if prev was not 0
//    if(abs(displacement) < 1){ // and curr is 0, reset count to 1
//      nearZeroCount = 1;
//    }
//    else{ // curr is not 0 too
//      nearZeroCount = 0;
//    }
//  }
// 
//}

//void updateWindow(double displ){
//  Serial.print("displace: ");
//  Serial.println(displ);
//
//  // empty window and put in the displ as the first item in window
//  if(abs(displaceArr[winEnd % WINDOW_SIZE]) < 1.00 && displ > 1.00){
//    winStart = 0;
//    winEnd = 0;
//    displaceArr[winEnd] = displ;
//    winEnd++;
//    nearZeroCount = 0;
//  }
//  else{
//    // filling window
//    if(winEnd < WINDOW_SIZE){
//      winEnd++;
//      displaceArr[winEnd] = displ;
//      //Serial.println("filling window");
//      if(abs(displ) < 1.00){
//        nearZeroCount++;
//        //Serial.print("within threshold # entering unfilled window || count now: ");
//        //Serial.println(nearZeroCount);
//      }
//    } 
//    // window was already filled
//    else {
//      
//    }
//  }
//  
//
//  //window already full?
//  else{
////    int winStartMod = winStart % WINDOW_SIZE;
////    double winStartVal = displaceArr[winStartMod];
////    double oldWinEndVal = displaceArr[winEnd % WINDOW_SIZE];
//
//    // replace the whole array when we see a zero to nonzero transition
//    if(abs(displaceArr[winEnd % WINDOW_SIZE]) < 1.00 && displ > 1.00){
//      winStart = 0;
//      winEnd = 0;
//      displaceArr[winEnd] = displ;
//      winEnd++;
//      nearZeroCount = 0;
//    } 
//    else {
//      winStart++;
//      winEnd++;
//      
//      int winEndMod = winEnd % WINDOW_SIZE;
//      displaceArr[winEndMod] = displ;
//      if(abs(oldWinEndVal) < 1.00 && displ > 1.00){
//        winStart = 0;
//        winEnd = 0;
//        displaceArr[winEnd] = displ;
//        winEnd++;
//        nearZeroCount = 0;
//      }
//    }
////    if(abs(winStartVal) < 1.00 && nearZeroCount > 0){
////      nearZeroCount--;
//////      Serial.print("within threshold # (");
////    }
//
////    Serial.print(winStartVal);
////    Serial.println(") leaving window");
////    Serial.println(nearZeroCount);
//    
////    winStart++;
////    winEnd++;
////    
////    int winEndMod = winEnd % WINDOW_SIZE;
////    displaceArr[winEndMod] = displ;
////    if(abs(oldWinEndVal) < 1.00 && displ > 1.00){
////      winStart = 0;
////      winEnd = 0;
////      displaceArr[winEnd] = displ;
////      winEnd++;
////      nearZeroCount = 0;
////    }
//    else if(abs(displ) < 1.00){
//      nearZeroCount++;
//      Serial.print("within threshold # (");
////      Serial.print(displ);
////      Serial.print(") entering window || count now: ");
////      Serial.println(nearZeroCount);
//    }
////    if(displ >= -1.00 && displ <= 1.00){
////      nearZeroCount++;
//////      Serial.print("within threshold # (");
//////      Serial.print(displ);
//////      Serial.print(") entering window || count now: ");
//////      Serial.println(nearZeroCount);
////    }
////    Serial.print(displ);
////    Serial.println(") entering window");
//  }
//  Serial.print("count is now: ");
//  Serial.println(nearZeroCount);
//}

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
