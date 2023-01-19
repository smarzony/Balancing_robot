int adc_key_val[5] = { 50, 200, 400, 600, 800 };
int NUM_KEYS = 5;
int adc_key_in;
int key = -1;
int oldkey = -1;

//Standard PWM DC control
int E1 = 5;  //M1 Speed Control
int E2 = 6;  //M2 Speed Control
int M1 = 4;  //M1 Direction Control
int M2 = 7;  //M1 Direction Control

int motors_power = 0;

void setup() {
  pinMode(13, OUTPUT);  //LED13 is used for mesure the button state.
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  adc_key_in = analogRead(7);
  digitalWrite(13, LOW);
  key = get_key(adc_key_in);  //Call the button judging function.

  if (motors_power > 0) {
    advance(motors_power, motors_power);
  }

  else if (motors_power < 0) {
    back_off(-motors_power, -motors_power);
  }

  else {
    stop();
  }

  if (key != oldkey) {  // Get the button pressed
    delay(50);
    adc_key_in = analogRead(7);
    key = get_key(adc_key_in);
    if (key != oldkey) {
      oldkey = key;
      if (key >= 0) {
        digitalWrite(13, HIGH);
        switch (key) {  // Send messages accordingly.
          case 0:
            motors_power--;
            Serial.print("Power--:");
            Serial.println(motors_power);

            // advance(100, 100);
            break;
          case 1:
            Serial.println("Power 0");
            motors_power = 0;
            // Serial.println("S2 OK");
            // turn_L(100, 100);
            break;
          case 2:
            Serial.print("Power++:");
            Serial.println(motors_power);
            motors_power++;
            // Serial.println("S3 OK");
            // back_off(100, 100);
            break;
          case 3:
            turn_R(100, 100);
            Serial.println("S4 OK");
            break;
          case 4:
            Serial.println("S5 OK");
            break;
        }
      }
    }
  }
  // else
  // {
  //   // stop();
  // }
  delay(100);
}

// To know the pressed button.
int get_key(unsigned int input) {
  int k;
  for (k = 0; k < NUM_KEYS; k++) {
    if (input < adc_key_val[k]) {  // Get the button pressed
      return k;
    }
  }
  if (k >= NUM_KEYS) k = -1;  // No button is pressed.
  return k;
}

void stop(void)  //Stop
{
  digitalWrite(E1, LOW);
  digitalWrite(E2, LOW);
}
void advance(char a, char b)  //Move forward
{
  analogWrite(E1, a);  //PWM Speed Control
  digitalWrite(M1, HIGH);
  analogWrite(E2, b);
  digitalWrite(M2, HIGH);
}
void back_off(char a, char b)  //Move backward
{
  analogWrite(E1, a);
  digitalWrite(M1, LOW);
  analogWrite(E2, b);
  digitalWrite(M2, LOW);
}
void turn_L(char a, char b)  //Turn Left
{
  analogWrite(E1, a);
  digitalWrite(M1, LOW);
  analogWrite(E2, b);
  digitalWrite(M2, HIGH);
}
void turn_R(char a, char b)  //Turn Right
{
  analogWrite(E1, a);
  digitalWrite(M1, HIGH);
  analogWrite(E2, b);
  digitalWrite(M2, LOW);
}