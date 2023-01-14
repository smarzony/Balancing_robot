//The sample code for driving one way motor encoder
#include <PID_v1.h>
#define encoder_L_pinA 2  //A pin -> the interrupt pin 0
#define encoder_L_pinB 8  //B pin -> the digital pin 3
#define encoder_R_pinA 3  //A pin -> the interrupt pin 0
#define encoder_R_pinB 9  //B pin -> the digital pin 3

#define Motor_L_DIR_pin 4
#define Motor_L_PWM_pin 5

#define Motor_R_PWM_pin 6
#define Motor_R_DIR_pin 7

#define MOTOR_SETPOINT_STEP 10
#define PID_SAMPLE_TIME 30
#define SPEED_SETPOINT_LIMIT 4*PID_SAMPLE_TIME

bool encoder_L_PinALast, encoder_R_PinALast;
double pulses_left, pulses_right, abs_pulses_left;  //the number of the pulses
boolean dir_left, dir_right;                        //the rotation dir_left
boolean motor_left_pid_ready, motor_right_pid_ready;

double motor_left_pwm;  //Power supplied to the motor PWM value.
double motor_left_setpoint_speed;

double motor_right_pwm;  //Power supplied to the motor PWM value.
double motor_right_setpoint_speed;

double Kp = 5, 
       Ki = 8, 
       Kd = 0.2;
PID motor_right_pid(&pulses_right, &motor_right_pwm, &motor_right_setpoint_speed, Kp, Ki, Kd, REVERSE);
PID motor_left_pid(&pulses_left, &motor_left_pwm, &motor_left_setpoint_speed, Kp, Ki, Kd, REVERSE);

int adc_key_val[5] = { 50, 200, 400, 600, 800 };
int NUM_KEYS = 5;
int adc_key_in;
int key = -1;
int oldkey = -1;

unsigned long long now, serial_print_time;

void setup() {
  Serial.begin(115200);  //Initialize the serial port

  pinMode(Motor_R_DIR_pin, OUTPUT);  //L298P Control port settings DC motor driver board for the output mode
  pinMode(Motor_R_PWM_pin, OUTPUT);
  pinMode(Motor_L_DIR_pin, OUTPUT);  //L298P Control port settings DC motor driver board for the output mode
  pinMode(Motor_L_PWM_pin, OUTPUT);

  motor_right_setpoint_speed = 0;
  motor_left_setpoint_speed = 0;  //Set the output value of the PID

  motor_right_pid.SetMode(AUTOMATIC);              //PID is set to automatic mode
  motor_right_pid.SetSampleTime(PID_SAMPLE_TIME);  //Set PID sampling frequency is 100ms
  motor_right_pid.SetOutputLimits(-255, 255);

  motor_left_pid.SetMode(AUTOMATIC);              //PID is set to automatic mode
  motor_left_pid.SetSampleTime(PID_SAMPLE_TIME);  //Set PID sampling frequency is 100ms
  motor_left_pid.SetOutputLimits(-255, 255);

  encoder_left_init();   //Initialize the module
  encoder_right_init();  //Initialize the module
}

void loop() {
  now = millis();
  keyboard_read();

  motor_left_go();   //Motor Forward
  motor_right_go();  //Motor Forward

  motor_right_pid_ready = motor_right_pid.Compute();  //PID conversion is complete and returns 1
  motor_left_pid_ready = motor_left_pid.Compute();    //PID conversion is complete and returns 1
  if (motor_left_pid_ready or motor_right_pid_ready) {

    Serial.print("PL:");
    Serial.print(pulses_left);
    Serial.print(",");
    Serial.print("MLP:");
    Serial.print(motor_left_pwm);
    Serial.print(",");
    Serial.print("MLS:");
    Serial.print(motor_left_setpoint_speed);

    Serial.print("PR:");
    Serial.print(pulses_right);
    Serial.print(",");
    Serial.print("MRP:");
    Serial.print(motor_right_pwm);
    Serial.print(",");
    Serial.print("MRS:");
    Serial.print(motor_right_setpoint_speed);
    Serial.println();

    if (motor_left_pid_ready)
      pulses_left = 0;  //Count clear, wait for the next count
    if (motor_right_pid_ready)
      pulses_right = 0;  //Count clear, wait for the next count
  }
}

void encoder_left_init() {
  dir_left = true;  //default -> Forward
  pinMode(encoder_L_pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_L_pinA), wheel_left_pulse_count, CHANGE);
}

void encoder_right_init() {
  dir_right = true;  //default -> Forward
  pinMode(encoder_R_pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_R_pinA), wheel_right_pulse_count, CHANGE);
}

void wheel_left_pulse_count() {
  bool Lstate = digitalRead(encoder_L_pinA);
  if ((encoder_L_PinALast == LOW) && Lstate == HIGH) {
    bool val = digitalRead(encoder_L_pinB);
    if (val == LOW && dir_left) {
      dir_left = false;  //Reverse
    } else if (val == HIGH && !dir_left) {
      dir_left = true;  //Forward
    }
  }
  encoder_L_PinALast = Lstate;

  if (!dir_left) pulses_left--;
  else pulses_left++;
}

void wheel_right_pulse_count() {
  int Rstate = digitalRead(encoder_R_pinA);
  if ((encoder_R_PinALast == LOW) && Rstate == HIGH) {
    bool val = digitalRead(encoder_R_pinB);
    if (val == LOW && dir_right) {
      dir_right = false;  //Reverse
    } else if (val == HIGH && !dir_right) {
      dir_right = true;  //Forward
    }
  }
  encoder_R_PinALast = Rstate;

  if (!dir_right) pulses_right--;
  else pulses_right++;
}

void motor_right_go()  //Motor Forward
{
  if (motor_right_pwm > 0) {
    digitalWrite(Motor_R_DIR_pin, LOW);
    analogWrite(Motor_R_PWM_pin, (uint8_t)motor_right_pwm);
  } else {
    digitalWrite(Motor_R_DIR_pin, HIGH);
    analogWrite(Motor_R_PWM_pin, (uint8_t)abs(motor_right_pwm));
  }
}

void motor_left_go()  //Motor Forward
{
  if (motor_left_pwm > 0) {
    digitalWrite(Motor_L_DIR_pin, LOW);
    analogWrite(Motor_L_PWM_pin, motor_left_pwm);
  } else {
    digitalWrite(Motor_L_DIR_pin, HIGH);
    analogWrite(Motor_L_PWM_pin, abs(motor_left_pwm));
  }
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
            // Serial.println("motors_setpoint_speed 0");
            motor_left_setpoint_speed = 0;
            motor_right_setpoint_speed = 0;

            break;
          case 1:
            // Serial.print("motor_right_setpoint_speed +");
            // Serial.println(MOTOR_SETPOINT_STEP);
            // motor_right_setpoint_speed = motor_right_setpoint_speed + MOTOR_SETPOINT_STEP;
            motor_right_setpoint_speed = SPEED_SETPOINT_LIMIT - SPEED_SETPOINT_LIMIT*0.6;
            if (motor_right_setpoint_speed > SPEED_SETPOINT_LIMIT) {
              motor_right_setpoint_speed = SPEED_SETPOINT_LIMIT;
            }
            break;
          case 2:
            // Serial.print("motor_right_setpoint_speed -");
            // Serial.println(MOTOR_SETPOINT_STEP);
            // motor_right_setpoint_speed = motor_right_setpoint_speed - MOTOR_SETPOINT_STEP;
            motor_right_setpoint_speed = -SPEED_SETPOINT_LIMIT + SPEED_SETPOINT_LIMIT*0.6;
            if (motor_right_setpoint_speed < -SPEED_SETPOINT_LIMIT) {
              motor_right_setpoint_speed = -SPEED_SETPOINT_LIMIT;
            }
            break;
          case 3:
            // Serial.print("motor_left_setpoint_speed -");
            // Serial.println(MOTOR_SETPOINT_STEP);
            motor_left_setpoint_speed = -SPEED_SETPOINT_LIMIT + SPEED_SETPOINT_LIMIT*0.6;
            // motor_left_setpoint_speed = motor_left_setpoint_speed - MOTOR_SETPOINT_STEP;
            if (motor_right_setpoint_speed < -SPEED_SETPOINT_LIMIT) {
              motor_right_setpoint_speed = -SPEED_SETPOINT_LIMIT;
            }
            break;
          case 4:
            // Serial.print("motor_left_setpoint_speed +");
            // Serial.println(MOTOR_SETPOINT_STEP);
            motor_left_setpoint_speed = SPEED_SETPOINT_LIMIT - SPEED_SETPOINT_LIMIT*0.6;
            // motor_left_setpoint_speed = motor_left_setpoint_speed + MOTOR_SETPOINT_STEP;
            if (motor_right_setpoint_speed > SPEED_SETPOINT_LIMIT) {
              motor_right_setpoint_speed = SPEED_SETPOINT_LIMIT;
            }
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