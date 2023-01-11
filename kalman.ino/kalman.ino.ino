#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>
 
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
 
void setup()
{
  Serial.begin(115200);
 
  // Inicjalizacja MPU6050
  Serial.println("Inicjalizacja MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Nie znaleziono MPU6050!");
    delay(500);
  }
 
  // Kalibracja zyroskopu
  mpu.calibrateGyro();
}
 
void loop()
{
  // Odczytanie wektorow z czujnikow
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();
 
  // Kalukacja Pitch &amp; Roll z akcelerometru
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI;
 
  // Kalman - dane z akcelerometru i zyroskopu
  kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  kalRoll = kalmanX.update(accRoll, gyr.XAxis);
 
  Serial.print("Pitch:");
  Serial.print(accPitch);
  Serial.print(",");
  Serial.print("Roll:");
  Serial.print(accRoll);
  Serial.print(",");
 
  Serial.print("(K)Pitch:");
  Serial.print(kalPitch);
  Serial.print(",");
  Serial.print("(K)Roll:");
  Serial.print(kalRoll);
 
  Serial.println();
  delay(10);
}