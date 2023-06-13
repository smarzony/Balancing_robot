#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>
#include <PID_v1.h>
#include <EEPROM.h>

#define DOUBLE_SIZEOF 4
#define UINT8_T_SIZEOF 2

#define ADDR_P 0
#define ADDR_I (DOUBLE_SIZEOF * 1)
#define ADDR_D (DOUBLE_SIZEOF * 2)
#define ADDR_ANGLE (DOUBLE_SIZEOF * 3)
#define ADDR_VELOCITY_LIMIT_MIN (ADDR_ANGLE + UINT8_T_SIZEOF)
#define ADDR_VELOCITY_LIMIT_MAX (ADDR_VELOCITY_LIMIT_MIN + UINT8_T_SIZEOF)

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
#define SERIAL_INTERVAL 50

#define ANGLE_BALANCE_SPAN 16.0

#define PID_BALANCING_SAMPLE_TIME_MS 50

#define PID_MOTORS_SAMPLE_TIME_MS 30
#define SPEED_SETPOINT_LIMIT (4 * PID_MOTORS_SAMPLE_TIME_MS)


bool encoder_L_PinALast, encoder_R_PinALast;
double pulses_left, pulses_right;  //the number of the pulses
bool dir_left, dir_right;                           //the rotation dir_left
bool motor_left_pid_ready, motor_right_pid_ready;

double motor_left_pwm;  //Power supplied to the motor PWM value.
double motor_left_setpoint_speed;

double motor_right_pwm;  //Power supplied to the motor PWM value.
double motor_right_setpoint_speed;

// KEYBOARD CONTROL
uint16_t adc_key_val[5] = { 50, 200, 400, 600, 800 };
int8_t NUM_KEYS = 5;
int8_t adc_key_in;
int8_t key = -1;
int8_t oldkey = -1;

bool enable_balancing = false;
bool disable_serial = true;
bool motor_test_run = false;

// PID
double Setpoint_angle, Input_angle, Output_motor_speed;
uint8_t Velocity_limit_min, Velocity_limit_max;

double Kp_balancing = 1,
       Ki_balancing = 100,
       Kd_balancing = 1;

double Kp_motors = 5,
       Ki_motors = 8,
       Kd_motors = 0.2;

PID motor_right_pid(&pulses_right, &motor_right_pwm, &motor_right_setpoint_speed, Kp_motors, Ki_motors, Kd_motors, REVERSE);
PID motor_left_pid(&pulses_left, &motor_left_pwm, &motor_left_setpoint_speed, Kp_motors, Ki_motors, Kd_motors, REVERSE);

PID balancePID(&Input_angle, &Output_motor_speed, &Setpoint_angle, Kp_balancing, Ki_balancing, Kd_balancing, DIRECT);

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

unsigned long now, gyro_timer, serial_timer;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  EEPROM.get(ADDR_P, Kp_balancing);
  EEPROM.get(ADDR_I, Ki_balancing);
  EEPROM.get(ADDR_D, Kd_balancing);
  EEPROM.get(ADDR_ANGLE, Setpoint_angle);
  EEPROM.get(ADDR_VELOCITY_LIMIT_MIN, Velocity_limit_min);
  EEPROM.get(ADDR_VELOCITY_LIMIT_MAX, Velocity_limit_max);

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
  balancePID.SetOutputLimits(-Velocity_limit_min, Velocity_limit_max);
}

void loop() {
  now = millis();
  keyboard_read();
  commandEngine();
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

  if (kalPitch < Setpoint_angle - ANGLE_BALANCE_SPAN || kalPitch > Setpoint_angle + ANGLE_BALANCE_SPAN) {
    enable_balancing = false;
  }

  if (enable_balancing) {
    Input_angle = kalPitch;
    motor_right_setpoint_speed = Output_motor_speed;
    motor_left_setpoint_speed = Output_motor_speed;
    balancePID.Compute();
    motor_left_go();   //Motor Forward
    motor_right_go();  //Motor Forward
  } else {
    motors_stop();
  }

  if ((now - serial_timer > SERIAL_INTERVAL) and !disable_serial) {
    serial_timer = now;
    serial_data();
  }
}

void serial_data() {
  Serial.print("E:");
  Serial.print(kalPitch - Setpoint_angle);
  Serial.print(",");
  Serial.print("Out_spd:");
  Serial.print(motor_right_setpoint_speed);
  Serial.print(",");
  Serial.println();
}
