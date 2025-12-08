void setup() {
  Serial.begin(9600);
  pinMode(8, INPUT_PULLUP);
}

void loop() {
  int SW = digitalRead(8);
  Serial.print("デジタルピンD8の状態: ");
  Serial.println(SW);
  delay(100);
}
