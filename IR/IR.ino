int emitterPin = 3;
int receiverPin = A0;
int bottomPin = A1;

void setup() {
  // put your setup code here, to run once:
  pinMode(emitterPin, OUTPUT);
  pinMode(emitterPin + 1, OUTPUT);
  pinMode(receiverPin, INPUT);
  pinMode(bottomPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  static float data = 0;
  digitalWrite(emitterPin, HIGH);
  digitalWrite(emitterPin + 1, HIGH);

  Serial.print("Top = ");
  Serial.println(analogRead(receiverPin));

  Serial.print("Bottom = ");
  Serial.println(analogRead(bottomPin));
  delay(500);
  
}
