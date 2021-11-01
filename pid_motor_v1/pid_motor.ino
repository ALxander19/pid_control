#include "pid_motor.h"

ros::NodeHandle nh;

void setup() {

  nh.initNode();

  nh.subscribe(w_right);
  nh.advertise(vel_motor_right);
  nh.advertise(vel_fil_right);

  pinMode(inaR, OUTPUT);
  pinMode(inbR, OUTPUT);
  
  digitalWrite(inaR, LOW);       // MOTOR RIGHT
  digitalWrite(inbR, LOW);
  
  pinMode(encaR, INPUT);           // MOTOR RIGHT
  digitalWrite(encaR, HIGH);       // turn on pullup resistor
  pinMode(encbR, INPUT);           // MOTOR RIGHT
  digitalWrite(encbR, HIGH);       // turn on pullup resistor

  attachInterrupt(digitalPinToInterrupt(encaR), doEncoderA, CHANGE);  // MOTOR ENCODER
  attachInterrupt(digitalPinToInterrupt(encbR), doEncoderB, CHANGE);  // EXTERNAL ENCODER
}

void loop() {

  unsigned long t = micros();

  if ((t - tTime[0]) >= (1000000 / ENCODER_PUBLISH_FREQUENCY))
  {
    newpositionR = encoderRPos;                      // MOTOR RIGHT
    newtime_encoder = micros();
    // 420 counts per revolution
    // La velocidad es de 75 rpm - 7.85 rad/s
    // Se da 4 pulsos por cuenta
    wR = (float)(newpositionR - oldpositionR)*(2*3.1416/(420*4))*1000000/(newtime_encoder - oldtime_encoder);

    oldpositionR = newpositionR;
    oldtime_encoder = newtime_encoder;

    vel_right_msg.data = wR;
    vel_motor_right.publish( &vel_right_msg );
    tTime[0] = t;
  }

  if ((t - tTime[1]) >= (1000000 / FILTER_PUBLISH_FREQUENCY))
  {
    velfil = wR * 0.0609 + velfil * 0.9391;
    vel_fil_msg.data = velfil;
    vel_fil_right.publish( &vel_fil_msg );
    tTime[1] = t;
  }

  if ((t - tTime[2]) >= (1000000 / MOTOR_FREQUENCY))
  {
    newtime_motor = micros();

    error_R = input - velfil;
    derivative_R = (error_R - prev_errorR)*1000000/(newtime_motor - oldtime_motor);
    integral_R = integral_R + error_R*(newtime_motor - newtime_motor)/1000000;
    wR_control = (int)(error_R*kp_R + derivative_R*kd_R + integral_R*ki_R);
    //wR_control = input;

    prev_errorR = error_R;
  
    if (wR_control == 0) {
      digitalWrite(inaR, LOW);
      digitalWrite(inbR, LOW);
    }
    else if (wR_control > 0 and wR_control <= 255) {
      analogWrite(inaR, wR_control);
      digitalWrite(inbR, LOW);
    }
    else if (wR_control > 255) {
      digitalWrite(inaR, HIGH);
      digitalWrite(inbR, LOW);
    }
    else if (wR_control < 0 and wR_control >= -255) {
      digitalWrite(inaR, LOW);
      analogWrite(inbR, wR_control*(-1));
    }
    else if (wR_control < -255) {
      digitalWrite(inaR, LOW);
      digitalWrite(inbR, HIGH);
    }
    
    oldtime_motor = newtime_motor;
    tTime[2] = t;
  }

  nh.spinOnce();
}


void w_right_cb(const std_msgs::Float32& w_right_msg) {

  input = w_right_msg.data;
}

void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(encaR) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encbR) == LOW) {
      encoderRPos = encoderRPos + 1;         // CW
    }
    else {
      encoderRPos = encoderRPos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encbR) == HIGH) {
      encoderRPos = encoderRPos + 1;          // CW
    }
    else {
      encoderRPos = encoderRPos - 1;          // CCW
    }
  }
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encbR) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encaR) == HIGH) {
      encoderRPos = encoderRPos + 1;         // CW
    }
    else {
      encoderRPos = encoderRPos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encaR) == LOW) {
      encoderRPos = encoderRPos + 1;          // CW
    }
    else {
      encoderRPos = encoderRPos - 1;          // CCW
    }
  }
}
