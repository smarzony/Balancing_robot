#include <EEPROM.h>
#include <CommandParser.h>  // https://github.com/Uberi/Arduino-CommandParser

#define DOUBLE_SIZEOF 4

#define ADDR_P 0
#define ADDR_I (DOUBLE_SIZEOF * 1)
#define ADDR_D (DOUBLE_SIZEOF * 2)

typedef CommandParser<> MyCommandParser;

MyCommandParser parser;

double P, I, D;

void cmd_P(MyCommandParser::Argument *args, char *response) {
  Serial.print("double: "); Serial.println(args[0].asDouble);
  P = args[0].asDouble;
  EEPROM.put(ADDR_P, P);
  strlcpy(response, "P saved", MyCommandParser::MAX_RESPONSE_SIZE);
}

void cmd_I(MyCommandParser::Argument *args, char *response) {
  Serial.print("double: "); Serial.println(args[0].asDouble);
  I = args[0].asDouble;
  EEPROM.put(ADDR_I, I);
  strlcpy(response, "I saved", MyCommandParser::MAX_RESPONSE_SIZE);
}

void cmd_D(MyCommandParser::Argument *args, char *response) {
  Serial.print("double: "); Serial.println(args[0].asDouble);
  D = args[0].asDouble;
  EEPROM.put(ADDR_D, D);
  strlcpy(response, "D saved", MyCommandParser::MAX_RESPONSE_SIZE);
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

void cmd_print(MyCommandParser::Argument *args, char *response) {
  Serial.print("P: ");
  Serial.println(P);
  Serial.print("I: ");
  Serial.println(I);
  Serial.print("D: ");
  Serial.println(D);
  strlcpy(response, "PID values", MyCommandParser::MAX_RESPONSE_SIZE);
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  parser.registerCommand("P", "d", &cmd_P);
  parser.registerCommand("I", "d", &cmd_I);  
  parser.registerCommand("D", "d", &cmd_D);
  parser.registerCommand("PRINT", "", &cmd_print);

  EEPROM.get(ADDR_P, P);
  EEPROM.get(ADDR_I, I);
  EEPROM.get(ADDR_D, D);
}

void loop() {
CommandsSerial();
delay(500);
}