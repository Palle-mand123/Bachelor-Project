int s0 = 4, s1 = 5, s2 = 6, s3 = 7;
int out = 9;

const int numSamples = 5;
unsigned int calRed[3] = {0};
unsigned int calYellow[3] = {0};
unsigned int calBlue[3] = {0};

void setup() {
  Serial.begin(115200);
  pinMode(s0, OUTPUT); pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT); pinMode(s3, OUTPUT);
  pinMode(out, INPUT);

  digitalWrite(s0, HIGH); // 100% scaling
  digitalWrite(s1, HIGH);

  Serial.println("=== Manual Calibration (Raw, Averaged) ===");
  Serial.println("Place object and type:");
  Serial.println("  r - save as RED");
  Serial.println("  y - save as Yellow");
  Serial.println("  b - save as BLUE");
  Serial.println("  x - show calibration data");
}

unsigned int readColor(byte s2State, byte s3State) {
  digitalWrite(s2, s2State);
  digitalWrite(s3, s3State);
  delay(50);
  unsigned long duration = pulseIn(out, LOW);
  if (duration == 0) duration = 1;
  return 1000000UL / duration;
}

void readAveragedRGB(unsigned int &r, unsigned int &g, unsigned int &b) {
  unsigned long rSum = 0, gSum = 0, bSum = 0;

  for (int i = 0; i < numSamples; i++) {
    rSum += readColor(LOW, LOW);      // Red filter
    gSum += readColor(HIGH, HIGH);    // Green filter
    bSum += readColor(LOW, HIGH);     // Blue filter
  }

  r = rSum / numSamples;
  g = gSum / numSamples;
  b = bSum / numSamples;
}

void loop() {
  unsigned int r, g, b;
  readAveragedRGB(r, g, b);

  Serial.print("Averaged RGB: ");
  Serial.print(r); Serial.print(" / ");
  Serial.print(g); Serial.print(" / ");
  Serial.println(b);

  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'r') {
      calRed[0] = r; calRed[1] = g; calRed[2] = b;
      Serial.println("[Saved] Red calibration.");
    } else if (cmd == 'y') {
      calYellow[0] = r; calYellow[1] = g; calYellow[2] = b;
      Serial.println("[Saved] Yellow calibration.");
    } else if (cmd == 'b') {
      calBlue[0] = r; calBlue[1] = g; calBlue[2] = b;
      Serial.println("[Saved] Blue calibration.");
    } else if (cmd == 'x') {
      Serial.println("== Calibrated Raw RGB ==");
      Serial.print("Red:   "); Serial.print(calRed[0]); Serial.print(" / ");
      Serial.print(calRed[1]); Serial.print(" / "); Serial.println(calRed[2]);
      Serial.print("Yellow: "); Serial.print(calYellow[0]); Serial.print(" / ");
      Serial.print(calYellow[1]); Serial.print(" / "); Serial.println(calYellow[2]);
      Serial.print("Blue:  "); Serial.print(calBlue[0]); Serial.print(" / ");
      Serial.print(calBlue[1]); Serial.print(" / "); Serial.println(calBlue[2]);
    }
  }

  delay(500);
}