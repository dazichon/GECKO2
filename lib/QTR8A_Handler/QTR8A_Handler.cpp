#include "QTR8A_Handler.h"

QTR8A_Handler::QTR8A_Handler() {}

void QTR8A_Handler::begin(const uint8_t pins[], uint8_t sensorCount) {
    _sensorCount = sensorCount;
    _qtr.setTypeAnalog();
    _qtr.setSensorPins(pins, _sensorCount);
}

void QTR8A_Handler::calibrate(uint8_t ledPin) {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);
    Serial.println("--- DANG CAN CHINH (QUET QTR QUA VACH DEN) ---");
    
    for (uint16_t i = 0; i < 400; i++) {
        _qtr.calibrate();
    }
    
    digitalWrite(ledPin, LOW);
    Serial.println("Can chinh xong!");
}

uint16_t QTR8A_Handler::getPosition() {
    // Đổi sang readLineBlack cho vạch đen trên nền trắng
    return _qtr.readLineBlack(_sensorValues);
}

uint16_t QTR8A_Handler::getSensorValue(uint8_t index) {
    if (index >= _sensorCount) return 0;
    // Không đảo ngược, giá trị cao khi gặp vạch trắng
    return _sensorValues[index];
}