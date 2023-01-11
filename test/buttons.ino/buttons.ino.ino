int adc_key_val[5] ={50, 200, 400, 600, 800 };
int NUM_KEYS = 5;
int adc_key_in;
int key=-1;
int oldkey=-1;

void setup(){
  pinMode(13, OUTPUT);  //LED13 is used for mesure the button state.
  Serial.begin(115200);
}

void loop(){
  adc_key_in = analogRead(7);
  digitalWrite(13,LOW);
  key = get_key(adc_key_in);  //Call the button judging function.

  if (key != oldkey){   // Get the button pressed
      delay(50);
      adc_key_in = analogRead(7);
      key = get_key(adc_key_in);
      if (key != oldkey)    {
        oldkey = key;
        if (key >=0){
          digitalWrite(13,HIGH);
          switch(key){          // Send messages accordingly.
             case 0:Serial.println("S1 OK");
                    break;
             case 1:Serial.println("S2 OK");
                    break;
             case 2:Serial.println("S3 OK");
                    break;
             case 3:Serial.println("S4 OK");
                    break;
             case 4:Serial.println("S5 OK");
                    break;
          }
        }
      }
  }
  delay(100);
}

// To know the pressed button.
int get_key(unsigned int input){
    int k;
    for (k = 0; k < NUM_KEYS; k++){
      if (input < adc_key_val[k]){     // Get the button pressed
            return k;
      }
   }
   if (k >= NUM_KEYS)k = -1;  // No button is pressed.
   return k;
}