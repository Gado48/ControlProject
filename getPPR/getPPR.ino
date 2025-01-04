const int encoderPinA = 22; // Encoder A channel
const int encoderPinB = 23; // Encoder B channel

volatile long encoderTicks = 0;

void setup() {
  Serial.begin(115200);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
}

void loop() {
  // Rotate the motor shaft manually or using a power supply
  // After one full revolution, print the number of ticks
  Serial.print("Encoder Ticks: ");
  Serial.println(encoderTicks);
  delay(1000);
}

void encoderISR() {
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);

  if (stateA == HIGH) {
    if (stateB == LOW) encoderTicks++;
    else encoderTicks--;
  } else {
    if (stateB == LOW) encoderTicks--;
    else encoderTicks++;
  }
}