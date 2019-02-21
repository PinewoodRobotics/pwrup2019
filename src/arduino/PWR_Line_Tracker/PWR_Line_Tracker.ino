
#define ROBORIO

#define TAPE (1)
#define CARPET (0)

#define INPUT_COUNT (16)
const byte input[INPUT_COUNT] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15 };

void setup() {
  Serial.begin(57600);
  Serial.println("Line tracker test v0.1");

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
  byte sendval;
  //1 is light, 0 is dark

  byte i;

  for (i=0; i<16; i++) {
    val[i] = analogRead(input[i]);
    maxValue = max(maxValue, val[i]);
    minValue = min(minValue, val[i]);
    avgValue += val[i];
  }
  avgValue /= INPUT_COUNT;
  for (i=0; i<16; i++) {
      if (maxValue-val[i] > val[i]-minValue) 
        valType[i]=TAPE; 
        else valType[i]=CARPET;
      
    }
  for (i=0; i<16; i++) {
    if (valType[i]==TAPE) {
      lightedgeleft=i;
      break;
    }
  }
  for (i=16; i>0; i--) {
    if (valType[i-1]==TAPE) {
      lightedgeright=i-1;
      break;
    }
  }
  linewidth = lightedgeright-lightedgeleft;
  lightmiddle = (lightedgeright+lightedgeleft)/2;

  #ifdef ROBORIO
  
  if ((linewidth <= 8) && (maxValue-minValue >=250)) {
    sendval=lightmiddle;
  } else {
    sendval=255;
  }
  Serial.write(sendval);

  #else

  for (i=0; i<16; i++) {
    if (maxValue-minValue >= 250) {
      char c;
      if (i==lightedgeleft) Serial.write('[');
      if (val[i] == minValue) c = '^';
      else if (val[i] < avgValue) c = '-';
      else c = '_';
      if ((lightedgeright-lightedgeleft <= 8) && (i==lightmiddle)) c = '*';
      Serial.write(c);
      if (i==lightedgeright) Serial.write(']');
    } else {
      char c;
      if (val[i] < avgValue) c = '-';
      else c = '_';
      Serial.write(c);
    }
  }

  Serial.print(" Min="); Serial.print(minValue);
  Serial.print(" Max="); Serial.print(maxValue);
  Serial.print(" Avg="); Serial.print(avgValue);
  Serial.print(" middle="); Serial.print(lightmiddle);
  Serial.print(" linewidth="); Serial.println(linewidth);  
  
  #endif
  delay(25);
}
