#define MAX_ARGUMENTS 3

void commandEngine() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Odczytaj dane z portu szeregowego do momentu napotkania znaku nowej linii

    // Serial.print("Otrzymano komendę: ");
    // Serial.println(command);

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
    else if (command.startsWith("ANG_SET")) 
    {
      if (command.length() == 7 || command.length() == 8) {
        functionAngleCurrent();
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
    else if (command.startsWith("V_LIM ")) {
      String arg1 = getValue(command, ' ', 1);
      String arg2 = getValue(command, ' ', 2);
      if (arg1 != "" && arg2 != "") {
        int val1 = arg1.toInt();
        int val2 = arg2.toInt();
        functionVLim(val1, val2);
      } 
      else {
        Serial.println("Nieprawidłowe argumenty!");
      }
    }
    else 
    {
      Serial.println("Nieznana komenda!");
    }
  }
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
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

void functionAngleCurrent() {
  EEPROM.put(ADDR_ANGLE, kalPitch);
  Setpoint_angle = kalPitch;
  // Dodaj kod obsługujący zapis parametru ANGLE
  Serial.print("ANGLE set to: ");
  Serial.println(kalPitch);
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
  Serial.print("Limits value: -");
  Serial.print(Velocity_limit_min);
  Serial.print(" ");
  Serial.println(Velocity_limit_max);

  // Dodaj inne parametry, jeśli są potrzebne
}

void functionVLim(int16_t arg1, int16_t arg2) {
  arg1 = abs(arg1);
  arg2 = abs(arg2);
  EEPROM.put(ADDR_VELOCITY_LIMIT_MIN, arg1);
  EEPROM.put(ADDR_VELOCITY_LIMIT_MAX, arg2);
  Velocity_limit_min = arg1;
  Velocity_limit_max = arg2;
  

  // Dodaj kod obsługujący zapis parametrów V_LIM
  balancePID.SetOutputLimits(-arg1, arg2);
  Serial.print("Wartość parametru V_LIMIT_MIN ustawiona na: ");
  Serial.println(arg1);
  Serial.print("Wartość parametru V_LIMIT_MAX ustawiona na: ");
  Serial.println(arg2);
}

