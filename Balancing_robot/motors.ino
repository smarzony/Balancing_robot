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
  int val = abs(motor_right_pwm) + MOTORS_DEAD_ZONE;
  if (val > 255)
    val = 255;

  if (motor_right_pwm > 0) {
    digitalWrite(Motor_R_DIR_pin, LOW);    
    analogWrite(Motor_R_PWM_pin, val);
  } else {
    digitalWrite(Motor_R_DIR_pin, HIGH);
    analogWrite(Motor_R_PWM_pin, val);
  }
}

void motor_left_go()  //Motor Forward
{
  int val = abs(motor_left_pwm) + MOTORS_DEAD_ZONE;
  if (val > 255)
    val = 255;
  
  if (val < MOTORS_DEAD_ZONE)
    val = MOTORS_DEAD_ZONE;

  if (motor_left_pwm > 0) {
    digitalWrite(Motor_L_DIR_pin, LOW);
    analogWrite(Motor_L_PWM_pin, val);
  } else {
    digitalWrite(Motor_L_DIR_pin, HIGH);
    analogWrite(Motor_L_PWM_pin, val);
  }
}

void motors_stop() {
  analogWrite(Motor_L_PWM_pin, 0);
  analogWrite(Motor_R_PWM_pin, 0);
}
