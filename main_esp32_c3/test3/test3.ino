#define DIR_PIN 6
#define PWM_PIN 7

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  analogWrite(PWM_PIN, 50);
  delay(10000);
  digitalWrite(DIR_PIN, LOW);
  analogWrite(PWM_PIN, 50);
  delay(10000);
  analogWrite(PWM_PIN, 0);
}

void loop() {
  // put your main code here, to run repeatedly:

}
