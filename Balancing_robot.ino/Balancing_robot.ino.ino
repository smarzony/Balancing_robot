#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>
#include <PID_v1.h>

#define GYRO_INTERVAL 1
#define SERIAL_INTERVAL 20

#define ANGLE_CENTER -101
#define ANGLE_BALANCE_SPAN 45.0
#define ANGLE_STANDSTILL 1.5

#define MOTOR_CORRECTION 0.89

//Standard PWM DC control
#define E1 5  //M1 Speed Control
#define E2 6  //M2 Speed Control
#define M1 4  //M1 Direction Control
#define M2 7  //M1 Direction Control



// KEYBOARD CONTROL
int adc_key_val[5] = { 50, 200, 400, 600, 800 };
int NUM_KEYS = 5;
int adc_key_in;
int key = -1;
int oldkey = -1;

bool disable_motors = true;
bool disable_serial = false;
bool motor_test_run = false;


// PID
double Setpoint, Input, Output;


// double Kp = 15,
//        Ki = 2,
//        Kd = 1;

double Kp = 18,
       Ki = 2000,
       Kd = 0.4;
      

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);
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

  // MOTORS
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  // GYRO
  // Inicjalizacja MPU6050
  Serial.println("Inicjalizacja MPU6050");
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Nie znaleziono MPU6050!");
    delay(500);
  }

  // Kalibracja zyroskopu
  mpu.calibrateGyro();  

  //PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-190, 190);
  myPID.SetSampleTime(25);
}

void loop() {
  now = millis();

  adc_key_in = analogRead(7);
  digitalWrite(13, LOW);
  key = get_key(adc_key_in);

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
            disable_motors = !disable_motors;
            if (disable_motors) {
              Serial.println("PID disabled");
              stop();
            } else
              Serial.println("PID enabled");
            break;
          case 1:
            disable_serial = !disable_serial;
            if (disable_motors)
              Serial.println("Serial disabled");
            else
              Serial.println("Serial enabled");
            break;
          case 2:
            motor_test_run = !motor_test_run;
            if (motor_test_run) {
              Serial.println("Test run enabled");
            } else {
              Serial.println("Test run disabled");
              stop();
            }
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

  if (now - gyro_timer > GYRO_INTERVAL) {
    gyro_timer = now;
    read_gyro_kalman();
  }

  if (now - serial_timer > SERIAL_INTERVAL) {
    serial_timer = now;
    serial_data();
  }

  Input = kalRoll;
  Setpoint = ANGLE_CENTER;
  if (!motor_test_run) {
    if (Input > ANGLE_CENTER - ANGLE_BALANCE_SPAN && Input < ANGLE_CENTER + ANGLE_BALANCE_SPAN) {
      balance_PID2();
    } else {
      stop();
    }
  } else {
    int man_speed = 100;
    fwd(man_speed  * MOTOR_CORRECTION, man_speed);
  }
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

void balance() {
  if (Input < ANGLE_CENTER - ANGLE_STANDSTILL) {
    bwd(100, 100);
  } else if (Input > ANGLE_CENTER + ANGLE_STANDSTILL) {
    fwd(100, 100);
  } else {
    if (!motor_test_run)
      stop();
  }
}

void balance_PID2() {
  myPID.Compute();
  u = Output;

  if (!disable_motors) {
    if (u > 0) {
      if (u > 255) {
        u = 255;
      }
      fwd(u * MOTOR_CORRECTION, u);
    } else {
      if (u < -255) {
        u = -255;
      }
      bwd(-u * MOTOR_CORRECTION, -u);
    }
  }
}


void serial_data() {
  if (!disable_serial) {
    Serial.print("Input:");
    Serial.print(Input);
    Serial.print(",");
    Serial.print("error:");
    Serial.print(Setpoint - Input);
    Serial.print(",");
    Serial.print("Output:");
    Serial.println(Output);
  }
}

void stop(void)  //Stop
{
  digitalWrite(E1, LOW);
  digitalWrite(E2, LOW);
}
void fwd(char a, char b)  //Move forward
{
  analogWrite(E1, a);  //PWM Speed Control
  digitalWrite(M1, HIGH);
  analogWrite(E2, b);
  digitalWrite(M2, HIGH);
}
void bwd(char a, char b)  //Move backward
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

void read_gyro_kalman(){
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();
 
  // Kalukacja Pitch &amp; Roll z akcelerometru
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI;

  // Kalman - dane z akcelerometru i zyroskopu
  kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  kalRoll = kalmanX.update(accRoll, gyr.XAxis);
}