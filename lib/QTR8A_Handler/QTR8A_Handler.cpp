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
    
    for (uint16_t i = 0; i < 200; i++) {
        _qtr.calibrate();
    }
    
    digitalWrite(ledPin, LOW);
    Serial.println("Can chinh xong!");
}
uint16_t QTR8A_Handler::getPosition(bool inverted) {
    _qtr.read(_sensorValues); 
    uint32_t weightedSum = 0;
    uint16_t total = 0;
    static uint16_t lastPosition = 2500; // Biến tĩnh để nhớ vị trí cuối cùng (center cho 6 sensors)

    for (uint8_t i = 0; i < _sensorCount; i++) {
        uint16_t val = getSensorValue(i); 
        if (inverted) val = 1000 - val; // Nếu là mode line trắng, đảo logic: trắng=1000, đen=0
        weightedSum += (uint32_t)val * i * 1000;
        total += val;
    }

    // Nếu không mắt nào thấy vạch đen (total rất nhỏ)
    if (total < 100) { 
        if (lastPosition < 2000) return 0;       // Nếu trước đó lệch trái, trả về 0 để bẻ phải gắt
        if (lastPosition > 3000) return 5000;    // Nếu trước đó lệch phải, trả về 5000 để bẻ trái gắt
        return lastPosition; // Nếu đang ở giữa mà mất thì giữ nguyên hướng cũ
    }

    // Normalize position phù hợp với 6 cảm biến (phạm vi 0-5000)
    uint16_t maxIndex = (_sensorCount > 0) ? (_sensorCount - 1) : 1;
    lastPosition = (weightedSum / total) * 5000 / (maxIndex * 1000);
    return lastPosition;
}

uint16_t QTR8A_Handler::getSensorValue(uint8_t index) {
    if (index >= _sensorCount) return 0;
    // Map giá trị raw sang 0-1000: trắng=0, đen=1000
    int mappedValue = map(_sensorValues[index], VAL_WHITE, VAL_BLACK, 0, 1000);
    // Giới hạn để không bị quá 1000 hoặc nhỏ hơn 0
    mappedValue = constrain(mappedValue, 0, 1000);
    // Làm tròn xuống 0 nếu giá trị nhỏ (dưới 300)
    if (mappedValue < 400) {
        mappedValue = 0;
    }
    return (uint16_t)mappedValue;
}