#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>
#include <PID_v1.h>

#define encoder_L_pinA 2  //A pin -> the interrupt pin 0
#define encoder_L_pinB 8  //B pin -> the digital pin 3
#define encoder_R_pinA 3  //A pin -> the interrupt pin 0
#define encoder_R_pinB 9  //B pin -> the digital pin 3

#define Motor_L_DIR_pin 4
#define Motor_L_PWM_pin 5

#define Motor_R_PWM_pin 6
#define Motor_R_DIR_pin 7

#define MOTORS_DEAD_ZONE 40

#define MOTORS_MAX_PWM (255 - MOTORS_DEAD_ZONE)

#define GYRO_INTERVAL 1
#define SERIAL_INTERVAL 20

#define ANGLE_CENTER -3.5
#define ANGLE_BALANCE_SPAN 16.0

#define PID_BALANCING_SAMPLE_TIME_MS 50

#define PID_MOTORS_SAMPLE_TIME_MS 30
#define SPEED_SETPOINT_LIMIT (4 * PID_MOTORS_SAMPLE_TIME_MS)



bool encoder_L_PinALast, encoder_R_PinALast;
double pulses_left, pulses_right, abs_pulses_left;  //the number of the pulses
bool dir_left, dir_right;                           //the rotation dir_left
bool motor_left_pid_ready, motor_right_pid_ready;

double motor_left_pwm;  //Power supplied to the motor PWM value.
double motor_left_setpoint_speed;

double motor_right_pwm;  //Power supplied to the motor PWM value.
double motor_right_setpoint_speed;

// KEYBOARD CONTROL
int adc_key_val[5] = { 50, 200, 400, 600, 800 };
int NUM_KEYS = 5;
int adc_key_in;
int key = -1;
int oldkey = -1;

bool enable_balancing = false;
bool disable_serial = false;
bool motor_test_run = false;


// PID
double Setpoint_angle, Input_angle, Output_motor_speed;


double Kp_balancing = 1,
       Ki_balancing = 100,
       Kd_balancing = 1;

double Kp_motors = 5,
       Ki_motors = 8,
       Kd_motors = 0.2;

PID motor_right_pid(&pulses_right, &motor_right_pwm, &motor_right_setpoint_speed, Kp_motors, Ki_motors, Kd_motors, REVERSE);
PID motor_left_pid(&pulses_left, &motor_left_pwm, &motor_left_setpoint_speed, Kp_motors, Ki_motors, Kd_motors, REVERSE);

PID balancePID(&Input_angle, &Output_motor_speed, &Setpoint_angle, Kp_balancing, Ki_balancing, Kd_balancing, DIRECT);

// PID speedPID()
int u;

// GYRO & KALMAN
MPU6050 mpu;

// Konfiguracja filtru Kalmana dla osi X i Y (kat, odchylka, pomiar)
KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

// Obliczone wartosci Pitch i Roll tylko z akcelerometru
float accPitch = 0;
float accRoll = 0;

// Obliczone wartosci Pitch i Roll z uwzglednieniem filtru Kalmana i zyroskopu
float kalPitch = 0;
float kalRoll = 0;


unsigned long long now, gyro_timer, serial_timer;


void setup() {
  Serial.begin(115200);

  // GYRO
  // Inicjalizacja MPU6050
  Serial.println("Inicjalizacja MPU6050");
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Nie znaleziono MPU6050!");
    delay(500);
  }

  // Kalibracja zyroskopu
  mpu.calibrateGyro();

  pinMode(Motor_R_DIR_pin, OUTPUT);  //L298P Control port settings DC motor driver board for the output mode
  pinMode(Motor_R_PWM_pin, OUTPUT);
  pinMode(Motor_L_DIR_pin, OUTPUT);  //L298P Control port settings DC motor driver board for the output mode
  pinMode(Motor_L_PWM_pin, OUTPUT);

  motor_right_setpoint_speed = 0;
  motor_left_setpoint_speed = 0;  //Set the output value of the PID

  motor_right_pid.SetMode(AUTOMATIC);                        //PID is set to automatic mode
  motor_right_pid.SetSampleTime(PID_MOTORS_SAMPLE_TIME_MS);  //Set PID sampling frequency is 100ms
  motor_right_pid.SetOutputLimits(-MOTORS_MAX_PWM, MOTORS_MAX_PWM);

  motor_left_pid.SetMode(AUTOMATIC);                        //PID is set to automatic mode
  motor_left_pid.SetSampleTime(PID_MOTORS_SAMPLE_TIME_MS);  //Set PID sampling frequency is 100ms
  motor_left_pid.SetOutputLimits(-MOTORS_MAX_PWM, MOTORS_MAX_PWM);

  encoder_left_init();   //Initialize the module
  encoder_right_init();  //Initialize the module

  balancePID.SetMode(AUTOMATIC);                           //PID is set to automatic mode
  balancePID.SetSampleTime(PID_BALANCING_SAMPLE_TIME_MS);  //Set PID sampling frequency is 100ms
  balancePID.SetOutputLimits(-150, 150);
}

void loop() {
  now = millis();
  keyboard_read();
  if (now - gyro_timer > GYRO_INTERVAL) {
    gyro_timer = now;
    read_gyro_kalman();
  }

  if (motor_right_pid.Compute()) {
    pulses_right = 0;
  }  //PID conversion is complete and returns 1
  if (motor_left_pid.Compute()) {
    pulses_left = 0;
  }  //PID conversion is complete and returns 1



  if (kalPitch < ANGLE_CENTER - ANGLE_BALANCE_SPAN || kalPitch > ANGLE_CENTER + ANGLE_BALANCE_SPAN) {
    enable_balancing = false;
  }

  if (enable_balancing) {
    Setpoint_angle = ANGLE_CENTER;
    Input_angle = kalPitch;
    motor_right_setpoint_speed = Output_motor_speed;
    motor_left_setpoint_speed = Output_motor_speed;
    balancePID.Compute();
    motor_left_go();   //Motor Forward
    motor_right_go();  //Motor Forward
  } else {
    motors_stop();
  }

  if (now - serial_timer > SERIAL_INTERVAL) {
    serial_timer = now;
    serial_data();
  }
}

void serial_data() {
  Serial.print("E:");
  Serial.print(kalPitch - ANGLE_CENTER);
  Serial.print(",");
  Serial.print("Out_spd:");
  Serial.print(motor_right_setpoint_speed);
  Serial.print(",");
  Serial.println();
}

void read_gyro_kalman() {
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  // Kalukacja Pitch &amp; Roll z akcelerometru
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis * acc.YAxis + acc.ZAxis * acc.ZAxis)) * 180.0) / M_PI;
  accRoll = (atan2(acc.YAxis, acc.ZAxis) * 180.0) / M_PI;

  // Kalman - dane z akcelerometru i zyroskopu
  kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  kalRoll = kalmanX.update(accRoll, gyr.XAxis);
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
    analogWrite(Motor_R_PWM_pin, (uint8_t)motor_right_pwm + MOTORS_DEAD_ZONE);
  } else {
    digitalWrite(Motor_R_DIR_pin, HIGH);
    analogWrite(Motor_R_PWM_pin, (uint8_t)abs(motor_right_pwm) + MOTORS_DEAD_ZONE);
  }
}

void motor_left_go()  //Motor Forward
{
  if (motor_left_pwm > 0) {
    digitalWrite(Motor_L_DIR_pin, LOW);
    analogWrite(Motor_L_PWM_pin, motor_left_pwm + MOTORS_DEAD_ZONE);
  } else {
    digitalWrite(Motor_L_DIR_pin, HIGH);
    analogWrite(Motor_L_PWM_pin, abs(motor_left_pwm) + MOTORS_DEAD_ZONE);
  }
}

void motors_stop() {
  analogWrite(Motor_L_PWM_pin, 0);
  analogWrite(Motor_R_PWM_pin, 0);
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
            break;
          case 1:
            enable_balancing = !enable_balancing;
            break;
          case 2:
            break;
          case 3:
            break;
          case 4:
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