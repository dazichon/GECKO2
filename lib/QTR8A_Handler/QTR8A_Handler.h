#ifndef QTR8A_HANDLER_H
#define QTR8A_HANDLER_H

#include <Arduino.h>
#include <QTRSensors.h>

class QTR8A_Handler {
  public:
    QTR8A_Handler();
    // Khởi tạo các chân và kiểu cảm biến
    void begin(const uint8_t pins[], uint8_t sensorCount);
    // Hàm thực hiện cân chỉnh
    void calibrate(uint8_t ledPin = 2);
    // Đọc giá trị vị trí và mảng giá trị cảm biến (Cho vạch trắng trên nền đen)
    uint16_t getPosition();
    // Lấy giá trị từng mắt đã lọc nhiễu
    uint16_t getSensorValue(uint8_t index);

  private:
    QTRSensors _qtr;
    static const uint8_t MAX_SENSORS = 8;
    uint16_t _sensorValues[MAX_SENSORS];
    uint8_t _sensorCount;
};

#endif