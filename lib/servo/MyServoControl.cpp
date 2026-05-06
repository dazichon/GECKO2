#include "MyServoControl.h"

MyServoControl::MyServoControl(int sPin, int bPin, int cAngle, int oAngle) {
    servoPin = sPin;
    btnPin = bPin;
    closeAngle = cAngle;
    openAngle = oAngle;
    isOpen = false;
    lastButtonState = HIGH;
}

void MyServoControl::begin() {
    pinMode(btnPin, INPUT_PULLUP); // Nút BOOT thường là chân 0[cite: 10]
    ESP32PWM::allocateTimer(2);
    myServo.attach(servoPin, 600, 2200); // Dùng chân 'servo' từ variables.h[cite: 10]
    myServo.setPeriodHertz(50);
    
    // Khởi động ở trạng thái đóng (góc kẹp)[cite: 5]
    currentAngle = closeAngle;
    myServo.write(closeAngle);
    delay(300);
    isOpen = false;
}

void MyServoControl::forceDetach() {
    myServo.detach();
}

void MyServoControl::open() {
    myServo.attach(servoPin, 600, 2200); // Đảm bảo servo đã được attach trước khi điều khiển
    Serial.println("Nhả...");
    myServo.write(openAngle);
    delay(200); 
    currentAngle = openAngle;
    isOpen = true;
}

void MyServoControl::close() {
    Serial.println("Kẹp...");
    myServo.write(closeAngle);
    delay(200); 
    currentAngle = closeAngle;
    myServo.detach(); // Giải phóng servo để tránh tiêu thụ điện năng khi không cần thiết[cite: 5]
    isOpen = false;
}

void MyServoControl::setAngle(int angle) {
    angle = constrain(angle, 0, 180);
    myServo.attach(servoPin, 600, 2200);
    Serial.print("Servo angle set to: ");
    Serial.println(angle);
    myServo.write(angle);
    delay(200);
    currentAngle = angle;
    isOpen = (angle != closeAngle);
}

int MyServoControl::getAngle() {
    return currentAngle;
}

void MyServoControl::update() {
    bool currentButtonState = digitalRead(btnPin);

    // Logic xử lý nhấn nút BOOT[cite: 5]
    if (lastButtonState == HIGH && currentButtonState == LOW) {
        delay(50); // Debounce chống nhiễu
        if (digitalRead(btnPin) == LOW) {
            if (isOpen) close();
            else open();
        }
    }
    lastButtonState = currentButtonState;
}

bool MyServoControl::getState() {
    return isOpen;
}