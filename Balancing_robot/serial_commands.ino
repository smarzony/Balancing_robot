#define MAX_ARGUMENTS 3

void commandEngine()
{
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Odczytaj dane z portu szeregowego do momentu napotkania znaku nowej linii

    // Podziel komendę na poszczególne argumenty
    String arguments[MAX_ARGUMENTS];
    int numArguments = splitCommand(command, arguments);

    if (numArguments > 0) {
      char functionCode = arguments[0].charAt(0); // Pierwszy znak komendy

      // Wykonaj odpowiednią funkcję na podstawie kodu funkcji i przekazanych argumentów
      switch (functionCode) {
        case 'P':
          if (numArguments >= 2) {
            float arg1 = arguments[1].toFloat();
            functionP(arg1);
          } else {
            Serial.println("Za mało argumentów dla funkcji P!");
          }
          break;
        case 'I':
          if (numArguments >= 2) {
            float arg1 = arguments[1].toFloat();
            functionI(arg1);
          } else {
            Serial.println("Za mało argumentów dla funkcji I!");
          }
          break;
        // Dodaj inne przypadki dla innych funkcji

        default:
          // Nieznana komenda, obsłuż błąd
          Serial.println("Nieznana komenda!");
          break;
      }
    }
  }
}

// Funkcja do podziału komendy na poszczególne argumenty
int splitCommand(String command, String arguments[]) {
  int index = 0;
  int startIndex = 0;
  int endIndex = command.indexOf(' ');
  while (endIndex != -1 && index < MAX_ARGUMENTS) {
    arguments[index] = command.substring(startIndex, endIndex);
    startIndex = endIndex + 1;
    endIndex = command.indexOf(' ', startIndex);
    index++;
  }
  if (index < MAX_ARGUMENTS) {
    arguments[index] = command.substring(startIndex);
    index++;
  }
  return index;
}

// Przykładowe funkcje P i I
void functionP(float arg1) {
  // Zaimplementuj funkcję P
  Serial.print("Wywołano funkcję P z argumentem 1: ");
  Serial.println(arg1);
}

void functionI(float arg1) {
  // Zaimplementuj funkcję I
  Serial.print("Wywołano funkcję I z argumentem 1: ");
  Serial.println(arg1);
}