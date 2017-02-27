unsigned long lastStateChange = millis();
bool isRunning = true;
const byte pin = 12;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, isRunning ? HIGH : LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - lastStateChange >= 10 * 1000) {
    lastStateChange = millis();

    if (isRunning) {
      isRunning = false;
      digitalWrite(pin, LOW);
    } else {
      isRunning = true;
      digitalWrite(pin, HIGH);
    }
  }
  Serial.print(millis());
  Serial.print(" running: ");
  Serial.println(isRunning);
}
