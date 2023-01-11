//The sample code for driving one way motor encoder
#include <PID_v1.h>
const byte encoder0pinA = 2;  //A pin -> the interrupt pin 0
const byte encoder0pinB = 3;  //B pin -> the digital pin 3
int E_right = 6;              //The enabling of L298PDC motor driver board connection to the digital interface port 5
int M_right = 7;              //The enabling of L298PDC motor driver board connection to the digital interface port 4
byte encoder0PinALast;
double duration, abs_duration;  //the number of the pulses
boolean Direction;              //the rotation direction
boolean result;

double val_output;  //Power supplied to the motor PWM value.
double Setpoint;
double Kp = 0.6, Ki = 5, Kd = 0;
PID myPID(&duration, &val_output, &Setpoint, Kp, Ki, Kd, DIRECT);

int adc_key_val[5] = { 50, 200, 400, 600, 800 };
int NUM_KEYS = 5;
int adc_key_in;
int key = -1;
int oldkey = -1;

void setup() {
  Serial.begin(115200);      //Initialize the serial port
  pinMode(M_right, OUTPUT);  //L298P Control port settings DC motor driver board for the output mode
  pinMode(E_right, OUTPUT);
  Setpoint = 0;            //Set the output value of the PID
  myPID.SetMode(AUTOMATIC);  //PID is set to automatic mode
  myPID.SetSampleTime(100);  //Set PID sampling frequency is 100ms
  myPID.SetOutputLimits(-255, 255);
  EncoderInit();  //Initialize the module
}

void loop() {
  keyboard_read();
  advance();  //Motor Forward
  //abs_duration = abs(duration);
  result = myPID.Compute();  //PID conversion is complete and returns 1
  if (result) {
    Serial.print("Pulse:");
    Serial.print(duration);
    Serial.print(",");
    Serial.print("pwm:");
    Serial.print(val_output);
    Serial.print(",");
    Serial.print("setpoint:");
    Serial.println(Setpoint);
    duration = 0;  //Count clear, wait for the next count
  }
}

void EncoderInit() {
  Direction = true;  //default -> Forward
  pinMode(encoder0pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0pinA), wheelSpeed, CHANGE);
}

void wheelSpeed() {
  int Lstate = digitalRead(encoder0pinA);
  if ((encoder0PinALast == LOW) && Lstate == HIGH) {
    int val = digitalRead(encoder0pinB);
    if (val == LOW && Direction) {
      Direction = false;  //Reverse
    } else if (val == HIGH && !Direction) {
      Direction = true;  //Forward
    }
  }
  encoder0PinALast = Lstate;

  if (!Direction) duration++;
  else duration--;
}
void advance()  //Motor Forward
{
  if (val_output > 0)
  {
    digitalWrite(M_right, LOW);
    analogWrite(E_right, val_output);
  }
  else
  {
    digitalWrite(M_right, HIGH);
    analogWrite(E_right, abs(val_output));
  }


  
}
void back()  //Motor reverse
{
  digitalWrite(M_right, HIGH);
  analogWrite(E_right, val_output);
}

void Stop()  //Motor stops
{
  digitalWrite(E_right, LOW);
}

void keyboard_read() {
  adc_key_in = analogRead(7);
  key = get_key(adc_key_in);  //Call the button judging function.

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
            Serial.println("Setpoint 400");

            Setpoint = 400;
            if (Setpoint > 440)
              Setpoint = 440;
            break;
          case 1:
            Serial.println("Setpoint -400");
            Setpoint = -400;
            if (Setpoint < -440)
              Setpoint = -440;
            break;
          case 2:
            Serial.println("Setpoint 0");
            Setpoint = 0;
            break;
          case 3:
            Serial.println("S4 OK");
            break;
          case 4:
            Serial.println("S5 OK");
            break;
        }
      }
    }
  }
}

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