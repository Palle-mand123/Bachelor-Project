int s0 = 4, s1 = 5, s2 = 6, s3 = 7; //Colorsensor pins
int out = 8; 
int RED_OUT = A0; //Color sensor output
int BLUE_OUT = A2;
int YELLOW_OUT = A1;
int Distance_sensor_3 = A4; //Distance sensor pin
int Distance_out_3 = 10; //Distance sensor out pin
int COLOR_PIN = 2; //Color on pin
int distance_pin = 3; //distance on pin
const int numSamples = 3; //number of samples color sensor takes
double threshold_close =  490-20; //thresholds for distance sensor, 490 is found by testing
double threshold_long = 232+20;  //
double sensorValue_3; 
int counter = 0; //counter for reset
bool red_force = HIGH;
bool yellow_force = HIGH;
bool blue_force = HIGH;
bool lastColorPinState = LOW;




// Your saved calibration data, should be recalibrated upon envoriment change.
unsigned int calRed[3]   = {34000,37000,45000};
unsigned int calYellow[3] = {54500,9000,58800};
unsigned int calBlue[3]  = {10000,41666,7000};
// Your thresholds, Should be changed upon envoriment change 
const int redThresholds[3]   = {7000, 8000, 5000};
const int yellowThresholds[3] = {9000, 8000,20000}; 
const int blueThresholds[3]  = {5000, 11000, 6000};

void setup() {
  Serial.begin(115200);
  pinMode(s0, OUTPUT); pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT); pinMode(s3, OUTPUT);
  pinMode(out, INPUT);
  pinMode(Distance_sensor_3,INPUT);
  pinMode(COLOR_PIN,INPUT);
  pinMode(distance_pin,INPUT);
  pinMode(RED_OUT,OUTPUT);
  pinMode(BLUE_OUT,OUTPUT);
  pinMode(YELLOW_OUT,OUTPUT);
  pinMode(Distance_out_3,OUTPUT);
  digitalWrite(s0, HIGH); // 100% frequency scaling
  digitalWrite(s1, HIGH);
  digitalWrite(YELLOW_OUT,HIGH);
  digitalWrite(BLUE_OUT,HIGH);
  digitalWrite(RED_OUT,HIGH);
  digitalWrite(Distance_out_3,HIGH);

}

unsigned int readColor(byte s2State, byte s3State) //Reads the sqaure wave on output with color filter on.
{
  digitalWrite(s2, s2State);
  digitalWrite(s3, s3State);
  delay(50);
  unsigned long duration = pulseIn(out, LOW); //pulse in starts to trigger when low, waits times until HIGH again.
  if (duration == 0) duration = 1;
  return 1000000UL / duration; 
}

void readAveragedRGB(unsigned int &r, unsigned int &g, unsigned int &b) //RGB values are sampled and averaged across the samples. Unsinged int boost the resolutiuon from 0-255 to 0-65535. Could be a potential mistake.
{
  unsigned long rSum = 0, gSum = 0, bSum = 0;
  for (int i = 0; i < numSamples; i++) {
    rSum += readColor(LOW, LOW);
    gSum += readColor(HIGH, HIGH);
    bSum += readColor(LOW, HIGH);
  }
  r = rSum / numSamples;
  g = gSum / numSamples;
  b = bSum / numSamples;
}



void Distance_sensor_3_read() //read and write function. Delays could be change if needed.
{
  sensorValue_3 = analogRead(Distance_sensor_3);
  Serial.println(sensorValue_3);
  if (sensorValue_3 > threshold_close) 
  {
    Serial.println("OBJECT CLOSE DIST 3");
    digitalWrite(Distance_out_3,LOW);
  }
  if (sensorValue_3 < threshold_long) 
  {
    Serial.println("OBJECT FAR DIST 3");
    digitalWrite(Distance_out_3,HIGH);
  }
  delay(100);
}


void loop() {
  unsigned int r, g, b;

  bool currentColorPinState = digitalRead(COLOR_PIN);

  // Detect rising edge
  if (currentColorPinState == HIGH && lastColorPinState == LOW) {
    // First time COLOR_PIN is HIGH -> Reset
    counter = 0;
    digitalWrite(RED_OUT, HIGH);
    digitalWrite(YELLOW_OUT, HIGH);
    digitalWrite(BLUE_OUT, HIGH);
    Serial.println("=== New color reading started ===");
  }
  //Reset
  if (currentColorPinState == HIGH) {
    if (counter == 10) {
      digitalWrite(RED_OUT, HIGH);
      digitalWrite(YELLOW_OUT, HIGH);
      digitalWrite(BLUE_OUT, HIGH);
      Serial.println(">>> No calibrated color detected.");
      counter = 0;
    }
    //read average RGB over sample
    readAveragedRGB(r, g, b);
    //prints the values
    Serial.print("Averaged RGB: ");
    Serial.print(r); Serial.print(" / ");
    Serial.print(g); Serial.print(" / ");
    Serial.println(b);
    //Bools to check what color based on calibration, current reading and thresholds.
    bool isRed =
      abs((long)r - calRed[0]) < redThresholds[0] &&
      abs((long)g - calRed[1]) < redThresholds[1] &&
      abs((long)b - calRed[2]) < redThresholds[2];

    bool isYellow =
      abs((long)r - calYellow[0]) < yellowThresholds[0] &&
      abs((long)g - calYellow[1]) < yellowThresholds[1] &&
      abs((long)b - calYellow[2]) < yellowThresholds[2];

    bool isBlue =
      abs((long)r - calBlue[0]) < blueThresholds[0] &&
      abs((long)g - calBlue[1]) < blueThresholds[1] &&
      abs((long)b - calBlue[2]) < blueThresholds[2];

    //Set pins accordinly based on color and resets
    if (isRed) {
      digitalWrite(RED_OUT, LOW);
      digitalWrite(YELLOW_OUT, HIGH);
      digitalWrite(BLUE_OUT, HIGH);
      counter = 0;
      Serial.println(">>> Detected Color: RED");
    } else if (isYellow) {
      digitalWrite(YELLOW_OUT, LOW);
      digitalWrite(RED_OUT, HIGH);
      digitalWrite(BLUE_OUT, HIGH);
      counter = 0;
      Serial.println(">>> Detected Color: YELLOW");
    } else if (isBlue) {
      digitalWrite(BLUE_OUT, LOW);
      digitalWrite(RED_OUT, HIGH);
      digitalWrite(YELLOW_OUT, HIGH);
      counter = 0;
      Serial.println(">>> Detected Color: BLUE");
    }

    counter += 1;
  }

  if (digitalRead(distance_pin) == HIGH) {
    Distance_sensor_3_read();
  }

  if (Serial.available()) //If color sensor is faulty, force values via serial
  {
    char cmd = Serial.read();
    if (cmd == 'r') {
      red_force = !red_force;
      digitalWrite(RED_OUT, red_force);
    }
    if (cmd == 'y') {
      yellow_force = !yellow_force;
      digitalWrite(YELLOW_OUT, yellow_force);
    }
    if (cmd == 'b') {
      blue_force = !blue_force;
      digitalWrite(BLUE_OUT, blue_force);
    }
    if (cmd == 'd') {
      dist_force = !dist_force;
      digitalWrite(Distance_out_3, dist_force);
    }
  }

  // Update last state
  lastColorPinState = currentColorPinState;

  delay(100);
}

