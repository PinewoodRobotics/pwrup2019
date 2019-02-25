/*
 * This sketch implements a line tracking algorithm optimised to find 2" wide white tape on
 * dark carpet using a Pololu QTRX-MD-16A 16-Channel Reflectance Sensor Array (8mm Pitch).
 * 
 * Samples are sent at about 40 per second on serial port 0 (USB) at 57,600 b/s.
 * Each sample is a single byte; 255 indicates the tape is not detected.
 * Values from 0 to 15 indicate under which sensor on the array the tape is centered.
 */

// Comment out the following line to test using the Arduino IDE's Serial Monitor
#define ROBORIO

// Adjustable constants
#define MIN_CONTRAST (250)  // Minumum data range to consider a valid tape match
#define MAX_WIDTH (8)       // 8 sensors @ 8mm = ~2.5".  Wider white patches are not tape.


// Defines what is detected under each sensor
#define TAPE    (1)
#define CARPET  (0)

// Defines for the pins that interface with the light sensor
#define INPUT_COUNT (16)
const byte input[INPUT_COUNT] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15 };

void setup() {
  Serial.begin(57600);
  
#ifndef ROBORIO
  Serial.println("PWR Line tracker v1.0");
#endif

  byte i;

  for (i=0; i<INPUT_COUNT; i++)
    pinMode(input[i],  INPUT);
}

void loop() {
  int maxValue = 0;
  int minValue = 32767;
  int avgValue = 0;
  int val[16];
  bool valType[16];
  int lightedgeleft;
  int lightedgeright;
  int lightmiddle;
  int edgelooptracker=1;
  int linewidth;
  bool foundtape = false;
  byte i;

  // Read all sensors and compute Min/Avg/Max
  for (i=0; i<16; i++) {
    val[i] = analogRead(input[i]);
    maxValue = max(maxValue, val[i]);
    minValue = min(minValue, val[i]);
    avgValue += val[i];
  }
  avgValue /= INPUT_COUNT;

  // Identify each sensor as CARPET or TAPE
  for (i=0; i<16; i++) {
    if (maxValue-val[i] > val[i]-minValue) 
      valType[i]=TAPE; 
    else
      valType[i]=CARPET;  
  }

  // Look for the right-most TAPE sample
  for (i=0; i<16; i++) {
    if (valType[i]==TAPE) {
      lightedgeright=i;
      break;
    }
  }

  
  // Look for the left-most TAPE sample
  for (i=16; i>0; i--) {
    if (valType[i-1]==TAPE) {
      lightedgeleft=i-1;
      break;
    }
  }

  // Compute the center and width of the tape
  linewidth = lightedgeleft-lightedgeright;
  lightmiddle = (lightedgeleft+lightedgeright)/2;

  // Determine if we have a line match
  if ((linewidth <= MAX_WIDTH) && (maxValue-minValue >= MIN_CONTRAST))
    foundtape = true;

  #ifdef ROBORIO
  // Serial is connected to the ROBORIO, so send a one-byte sample
  
  byte sendval = 255;

  if (foundtape)
    sendval=lightmiddle;

  Serial.write(sendval);

  #else
  // Serial is connected to the Arduino IDE Serial Monitor, so show test data

  // Print an "ASCII art" representation of the sensor array
  for (i=16; i; --i) {
    char c = '_'; // Default to CARPET
    
    if (valType[i] == TAPE)
      c = '-';    // Sensor is seeing TAPE

    // Add special markers if we have a valid line match
    if (foundtape) {
      if (i==lightmiddle) c = '*';
      else if (i==lightedgeleft) c = '[';
      else if (i==lightedgeright) c = ']';
      else if (valType[i] == TAPE) c = '-';
    }
    Serial.write(c);
  }

  // Print our computed variables
  Serial.print(" Min="); Serial.print(minValue);
  Serial.print(" Max="); Serial.print(maxValue);
  Serial.print(" Avg="); Serial.print(avgValue);
  Serial.print(" middle="); Serial.print(lightmiddle);
  Serial.print(" linewidth="); Serial.println(linewidth);  
  
  #endif
  
  delay(25);  // wait 25ms between samples (about 40 samples per second)
}
