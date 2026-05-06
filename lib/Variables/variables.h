#ifndef VARIABLES_H
#define VARIABLES_H

#include <Arduino.h>

// --- Định nghĩa chân cắm (Pins) ---
// Cảm biến QTR
#define qtr1 36 
#define qtr2 39
#define qtr3 34
#define qtr4 35
#define qtr5 33
#define qtr6 25
#define qtr7 26
#define qtr8 27

// Encoder
#define c1l 18
#define c2l 19
#define c1r 21
#define c2r 22

// Điều khiển Motor A
#define AIN1 5
#define AIN2 16
#define PWMA 4  

// Điều khiển Motor B
#define BIN1 2
#define BIN2 15
#define PWMB 17

#define STBY 32

// servo
#define servo_pin 14

//vl503
#define sda_pin 23
#define scl_pin 13

// --- Khai báo biến toàn cục (Extern) ---
// Dùng extern để các file khác không bị báo lỗi "multiple definition"
extern uint8_t robot_state;
extern uint8_t speed_mode;
extern uint8_t denta_speed;
extern long one_cell_distance;

#endif