#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))
int pin_mapping[] = { 3, 5, 6, 10, 11 };
int n_pins = ARRAY_SIZE(pin_mapping);
int selected = -1;

void setup() {
  Serial.begin(9600);
  Serial.println("hi");
  for (size_t i = 0; i < n_pins; i++) {
    Serial.print("connector ");
    Serial.print(i);
    Serial.print(" mapped to pin ");
    Serial.println(pin_mapping[i]);
    pinMode(pin_mapping[i], OUTPUT);
  }
}

void loop() {
  if (Serial.available() > 0) {
    int c = Serial.read();
    
    if (c >= '0' && c <= '0' + n_pins - 1) {
      selected = (c - '0');
    } else if (c == '+') {
      if (selected != -1) {
        Serial.print("output ");
        Serial.print(selected);
        Serial.println(" ON");
        digitalWrite(pin_mapping[selected], HIGH);
      }
    } else if (c == '-') {
      if (selected != -1) {
        Serial.print("output ");
        Serial.print(selected);
        Serial.println(" OFF");
        digitalWrite(pin_mapping[selected], LOW);
      }
    } else {
      Serial.print("invalid character ");
      Serial.println(c);
    }
  }
}
