#ifndef MY_SERVO_CONTROL_H
#define MY_SERVO_CONTROL_H

#include <Arduino.h>
#include <ESP32Servo.h>


class MyServoControl {
private:
    Servo myServo;
    int servoPin;
    int btnPin;
    int closeAngle;
    int openAngle;
    bool isOpen;
    bool lastButtonState;
    int currentAngle;

public:
    // Khởi tạo: chân servo, chân nút bấm, góc đóng, góc mở
    MyServoControl(int servoPin, int btnPin, int cAngle = 90, int oAngle = 0);
    
    void begin();    // Khởi tạo trong setup()
    void update();   // Kiểm tra nút bấm trong loop()
    void open();     // Lệnh nhả
    void close();    // Lệnh kẹp
    void setAngle(int angle); // Đặt góc servo tùy ý
    int getAngle();
    bool getState(); // Lấy trạng thái hiện tại (đang mở hay đóng)
    void forceDetach();
};

#endif