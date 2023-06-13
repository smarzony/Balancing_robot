#define MAX_ARGUMENTS 3

void commandEngine() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Odczytaj dane z portu szeregowego do momentu napotkania znaku nowej linii

    Serial.print("Otrzymano komendę: ");
    Serial.println(command);

    if (command.startsWith("PRINT")) 
    {
      if (command.length() == 5 || command.length() == 6) {
        functionPrint();
      } else {
        Serial.println("Nieznana komenda!");
      }
    } 
    else if (command.startsWith("BAL")) 
    {
      if (command.length() == 3 || command.length() == 4) {
        functionBalanceEnable();
      } 
      else {
        Serial.println("Nieznana komenda!");
      }
    } 
    else if (command.startsWith("SERIAL")) 
    {
      if (command.length() == 6 || command.length() == 7) {
        functionSerialEnable();
      } 
      else {
        Serial.println("Nieznana komenda!");
      }
    } 
    else if (command.startsWith("P ")) 
    {
      float arg = command.substring(2).toFloat();
      functionP(arg);
    } 
    else if (command.startsWith("I ")) 
    {
      float arg = command.substring(2).toFloat();
      functionI(arg);
    }
    else if (command.startsWith("D ")) 
    {
      float arg = command.substring(2).toFloat();
      functionD(arg);
    }
    else if (command.startsWith("ANGLE ")) 
    {
      float arg = command.substring(6).toFloat();
      functionAngle(arg);
    }
    else 
    {
      Serial.println("Nieznana komenda!");
    }
  }
}

void functionP(double arg) {
  Kp_balancing = arg;
  EEPROM.put(ADDR_P, arg);
  Serial.print("P set to: ");
  Serial.println(arg);
}

void functionI(double arg) {
  Ki_balancing = arg;
  EEPROM.put(ADDR_I, arg);
  Serial.print("I set to: ");
  Serial.println(arg);
}

void functionD(double arg) {
  Kd_balancing = arg;
  EEPROM.put(ADDR_D, arg);
  Serial.print("D set to: ");
  Serial.println(arg);
}

void functionAngle(float arg) {
  Setpoint_angle = arg;
  EEPROM.put(ADDR_ANGLE, arg);
  // Dodaj kod obsługujący zapis parametru ANGLE
  Serial.print("ANGLE set to: ");
  Serial.println(arg);
}

void functionSerialEnable() {
  disable_serial = !disable_serial;
  Serial.print("disable_serial set to: ");
  Serial.println(disable_serial);
}

void functionBalanceEnable() {
  enable_balancing = !enable_balancing;
  Serial.print("enable_balancing set to: ");
  Serial.println(enable_balancing);
}

void functionPrint() {
  // Wyświetl wszystkie parametry
  Serial.print("PID value: ");
  Serial.print(Kp_balancing);
  Serial.print(" ");
  Serial.print(Ki_balancing);
  Serial.print(" ");
  Serial.println(Kd_balancing);
  Serial.print("ANGLE value: ");
  Serial.println(Setpoint_angle);

  // Dodaj inne parametry, jeśli są potrzebne
}

