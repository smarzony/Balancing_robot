void cmd_P(MyCommandParser::Argument *args, char *response) {
  // Serial.print("double: "); Serial.println(args[0].asDouble);
  Kp_balancing = args[0].asDouble;
  EEPROM.put(ADDR_P, Kp_balancing);
  strlcpy(response, "P saved", MyCommandParser::MAX_RESPONSE_SIZE);
}

void cmd_I(MyCommandParser::Argument *args, char *response) {
  // Serial.print("double: "); Serial.println(args[0].asDouble);
  Ki_balancing = args[0].asDouble;
  EEPROM.put(ADDR_I, Ki_balancing);
  strlcpy(response, "I saved", MyCommandParser::MAX_RESPONSE_SIZE);
}

void cmd_D(MyCommandParser::Argument *args, char *response) {
  // Serial.print("double: "); Serial.println(args[0].asDouble);
  Kd_balancing = args[0].asDouble;
  EEPROM.put(ADDR_D, Kd_balancing);
  strlcpy(response, "D saved", MyCommandParser::MAX_RESPONSE_SIZE);
}

void cmd_ANGLE(MyCommandParser::Argument *args, char *response) {
  // Serial.print("double: "); Serial.println(args[0].asDouble);
  Setpoint_angle = args[0].asDouble;
  EEPROM.put(ADDR_ANGLE, Setpoint_angle);
  strlcpy(response, "Angle saved", MyCommandParser::MAX_RESPONSE_SIZE);
}

void cmd_print(MyCommandParser::Argument *args, char *response) {
  Serial.print("Kp_b: ");
  Serial.println(Kp_balancing);
  Serial.print("Ki_b: ");
  Serial.println(Ki_balancing);
  Serial.print("Kd_b: ");
  Serial.println(Kd_balancing);
  Serial.print("Setp: ");
  Serial.println(Setpoint_angle);
  strlcpy(response, "PID values", MyCommandParser::MAX_RESPONSE_SIZE);
}

void cmd_switch_serial(MyCommandParser::Argument *args, char *response) {
  disable_serial = !disable_serial;
  strlcpy(response, "Serial switched", MyCommandParser::MAX_RESPONSE_SIZE);
}

void CommandsSerial()
{
  if (Serial.available()) {
    char line[128];
    size_t lineLength = Serial.readBytesUntil('\n', line, 127);
    line[lineLength] = '\0';

    char response[MyCommandParser::MAX_RESPONSE_SIZE];
    uint8_t success = parser.processCommand(line, response);
    Serial.println(response);
  }
}