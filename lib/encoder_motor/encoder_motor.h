#ifndef ENCODER_MOTOR_H
#define ENCODER_MOTOR_H

#include <Arduino.h>
#include "variables.h"

// Biến Encoder (Dùng extern để main.cpp có thể đọc)
extern volatile long encoderCountLeft;
extern volatile long encoderCountRight;

// Biến điều khiển tốc độ
extern uint8_t speed_mode;
extern uint8_t denta_speed;

// Khởi tạo và điều khiển cơ bản
void initEncoderMotor();
void onEncoderLeft();
void onEncoderRight();
void resetEncoder();

// Điều khiển Motor
void controlMotor_left(int speed);
void controlMotor_right(int speed);
void controlMotor(int speed_left, int speed_right);
void stopMotor();

// Các hàm di chuyển logic
void forward(long dic);
void rotate(long dic);
void turn_left();
void turn_right();
void turn_back();
void go_one_cell();
void run_path(String path);

#endif