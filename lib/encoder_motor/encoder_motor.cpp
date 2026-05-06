#include "encoder_motor.h"
#include "variables.h" 


volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

void initEncoderMotor() {
    // Motor Pins - Khớp với variables.h
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    // Encoder Pins - Khớp với c1l, c2l, c1r, c2r
    pinMode(c1l, INPUT_PULLUP); pinMode(c2l, INPUT_PULLUP);
    pinMode(c1r, INPUT_PULLUP); pinMode(c2r, INPUT_PULLUP);

    // Ngắt Encoder (Dùng chân pha A của mỗi bên)
    attachInterrupt(digitalPinToInterrupt(c1l), onEncoderLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(c1r), onEncoderRight, CHANGE);
}

// Logic đọc Encoder[cite: 11]
void onEncoderLeft() {
    if (digitalRead(c1l) == digitalRead(c2l)) encoderCountLeft--;
    else encoderCountLeft++;
}

void onEncoderRight() {
    if (digitalRead(c1r) == digitalRead(c2r)) encoderCountRight++;
    else encoderCountRight--;
}

void resetEncoder() {
    encoderCountLeft = 0;
    encoderCountRight = 0;
}

// Điều khiển Motor Trái[cite: 11]
void controlMotor_left(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
        analogWrite(PWMA, speed);
    } else if (speed < 0) {
        digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
        analogWrite(PWMA, -speed);
    } else {
        digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
        analogWrite(PWMA, 0);
    }
}

// Điều khiển Motor Phải[cite: 11]
void controlMotor_right(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
        analogWrite(PWMB, speed);
    } else if (speed < 0) {
        digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
        analogWrite(PWMB, -speed);
    } else {
        digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
        analogWrite(PWMB, 0);
    }
}

void controlMotor(int speed_left, int speed_right) {
    controlMotor_left(speed_left);
    controlMotor_right(speed_right); 
}

void stopMotor() {
    controlMotor(0, 0);
}

// Các hàm di chuyển dùng biến từ variables.h[cite: 11]
void forward(long dic) {
    resetEncoder();
    controlMotor(speed_mode, speed_mode + denta_speed); 
    while (abs(encoderCountLeft) + abs(encoderCountRight) < dic * 2) {
        delay(1); 
    }
    stopMotor();
}

void rotate(long dic) {
    resetEncoder();
    if (dic > 0) controlMotor(-speed_mode, speed_mode); 
    else controlMotor(speed_mode, -speed_mode);         
    
    while (abs(encoderCountLeft) + abs(encoderCountRight) < abs(dic) * 2) {
        delay(1);
    }
    stopMotor();
}

void turn_left() { rotate(450); }
void turn_right() { rotate(-450); }
void go_one_cell() { forward(one_cell_distance); }