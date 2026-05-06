
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "encoder_motor.h"
#include "MyServoControl.h"
#include "variables.h"
#include "QTR8A_Handler.h" // Thư viện mình vừa tạo
#include "PID.h" // Thêm PID
#include "esp_wifi.h"

// Cấu hình WiFi
const char* ssid = "da";
const char* password = "ducanh1110";

WebServer server(80);
MyServoControl gripper(servo_pin, 0, 0, 45);

// Biến điều khiển servo
int servoAngle = 0; // Giá trị mặc định của slider

// Biến điều khiển tốc độ
int current_speed = 40;
const int MIN_SPEED = 10;
const int MAX_SPEED = 100;

// Khởi tạo QTR8A
QTR8A_Handler myQTR;
const uint8_t qtrPins[] = {36, 39, 34, 35, 33, 25, 26, 27};
const uint8_t sensorCount = 8;

// Khởi tạo PID cho dò line
PID linePID(0.3, 0.0, 0.1, -50, 50); // Giảm Kp để giảm độ cua mạnh

// Giao diện Web (HTML + CSS + JS cập nhật cảm biến)
String getHTML() {
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>";
    html += "html, body { margin:0; padding:0; font-family: Arial, sans-serif; background:#111; color:#eee; user-select:none; -webkit-user-select:none; -ms-user-select:none; }";
    html += "body { min-height:100vh; display:flex; align-items:center; justify-content:center; }";
    html += "button { width: 100px; height: 45px; margin: 8px; font-size: 18px; border-radius: 12px; border: none; background: #3498db; color: white; cursor: pointer; outline: none; user-select:none; -webkit-user-select:none; touch-action: manipulation; -webkit-tap-highlight-color: transparent; }";
    html += "button:active { transform: scale(0.98); }";
    html += ".stop { background: #e74c3c; }";
    html += ".gripper { background: #2ecc71; width: 220px; }";
    html += ".control-row { display:flex; justify-content:center; align-items:center; gap: 10px; flex-wrap: wrap; margin-top: 12px; }";
    html += ".panel { text-align:center; width: 100%; max-width: 420px; padding: 20px; box-sizing:border-box; }";
    html += "</style></head><body>";
    html += "<div class='panel'><center><h1>Robocon Control</h1>";

    // Hiển thị và điều khiển tốc độ
    html += "<div class='control-row'>";
    html += "<div style='color: #f1c40f; font-weight: bold; margin: 10px 0;'>Tốc độ: <span id='speed'>30</span>/100</div>";
    html += "</div>";
    html += "<div class='control-row'>";
    html += "<button type='button' onclick='sendCommand(\"/speed_down\")'><b>-</b></button>";
    html += "<button type='button' onclick='sendCommand(\"/speed_up\")'><b>+</b></button>";
    html += "</div>";

    // Giao diện điều khiển kiểu nhấn giữ
    html += "<div class='control-row'>";
    html += "<button type='button' onpointerdown='handlePress(\"/forward\", event)' onpointerup='handleRelease(event)' onpointercancel='handleRelease(event)' onpointerleave='handleRelease(event)'>Tien</button>";
    html += "</div>";
    html += "<div class='control-row'>";
    html += "<button type='button' onpointerdown='handlePress(\"/left\", event)' onpointerup='handleRelease(event)' onpointercancel='handleRelease(event)' onpointerleave='handleRelease(event)'>Trai</button>";
    html += "<button type='button' class='stop' onclick='sendCommand(\"/stop\")'>STOP</button>";
    html += "<button type='button' onpointerdown='handlePress(\"/right\", event)' onpointerup='handleRelease(event)' onpointercancel='handleRelease(event)' onpointerleave='handleRelease(event)'>Phai</button>";
    html += "</div>";
    html += "<div class='control-row'>";
    html += "<button type='button' onpointerdown='handlePress(\"/backward\", event)' onpointerup='handleRelease(event)' onpointercancel='handleRelease(event)' onpointerleave='handleRelease(event)'>Lui</button>";
    html += "</div>";

    // Nút gắp vẫn dùng onclick để bấm một lần là đổi trạng thái
    html += "<div class='control-row'><button type='button' class='gripper' onclick='sendCommand(\"/toggle_gripper\")'>GAP / NHA (SERVO)</button></div>";
    html += "<div class='control-row' style='flex-direction:column; align-items:center; width:100%;'>";
    html += "  <label style='width:100%; color:#f1c40f; font-weight:bold; margin-bottom:8px;'>Servo góc: <span id='servoAngle'>" + String(servoAngle) + "</span>°</label>";
    html += "  <input id='servoSlider' type='range' min='0' max='80' step='1' value='" + String(servoAngle) + "' style='width:100%; margin-bottom:12px;' oninput='updateServo(this.value)' />";
    html += "</div>";
    html += "<div class='control-row'><button type='button' class='stop' onclick='sendCommand(\"/turn_off_wifi\")'>TAT WIFI</button></div>";
    // Script JS để lấy dữ liệu cảm biến mỗi 100ms
    html += "<script>";
    html += "function sendCommand(path) { fetch(path).catch(() => {}); }";
    html += "function handlePress(path, event) { event.preventDefault(); sendCommand(path); }";
    html += "function handleRelease(event) { event.preventDefault(); sendCommand('/stop'); }";
    html += "function updateServo(value) { document.getElementById('servoAngle').innerText = value; fetch('/servo_angle?value=' + value).catch(() => {}); }";
    html += "setInterval(function() {";
    html += "  fetch('/data').then(response => response.json()).then(data => {";
    html += "    document.getElementById('speed').innerText = data.speed;";
    html += "  }).catch(() => {});";
    html += "}, 200);"; // Cập nhật mỗi 200ms
    html += "</script>";

    html += "</center></div></body></html>";
    return html;
}
void disableWiFi() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    esp_wifi_stop();     // đảm bảo radio dừng hẳn
    delay(200);          // chờ ADC2 ổn định
}

// Thuật toán dò line sử dụng PID
void followLine() {
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // dt tính bằng giây
    if (dt < 0.005) dt = 0.005; // Tránh dt quá nhỏ
    lastTime = currentTime;

    // Đọc vị trí line từ QTR8A (0-7000)
    uint16_t position = myQTR.getPosition();
    
    // Hiển thị thông số QTR lên Serial Monitor
    Serial.print("QTR Position: ");
    Serial.print(position);
    Serial.print(" | Sensors: ");
    for (int i = 0; i < sensorCount; i++) {
        Serial.print(myQTR.getSensorValue(i));
        Serial.print(" ");
    }
    Serial.println();
    
    // Tính correction từ PID (setpoint=3500 đã được set trong setup)
    // Position từ readLineBlack(): 0=trái, 3500=giữa, 7000=phải
    float correction = linePID.compute(position, dt);
    
    // Tính tốc độ motor dựa trên setup bánh xe của bạn
    // Bánh phải lag hơn bánh trái ~10.35%, nên cần cân bằng compensation
    int baseSpeed = current_speed;
    const float wheelCompensation = 1.1035; // Compensation cho bánh phải
    
    // Công thức: 
    // - Khi error > 0 (line lệch phải): giảm bánh trái hoặc tăng bánh phải
    // - Khi error < 0 (line lệch trái): tăng bánh trái hoặc giảm bánh phải
    int leftSpeed = baseSpeed - correction;  
    int rightSpeed = (baseSpeed * wheelCompensation) + correction;
    
    // Giới hạn tốc độ
    leftSpeed = constrain(leftSpeed, -60, 60);
    rightSpeed = constrain(rightSpeed, -60, 60);

    Serial.print(correction);

    Serial.print(leftSpeed);

    Serial.println(rightSpeed);
    
    // Điều khiển motor
    controlMotor(leftSpeed, rightSpeed);

}
void setup() {
    Serial.begin(115200);

    // Khởi tạo motor và servo
    initEncoderMotor();
    gripper.begin();

    // Khởi tạo QTR8A và cân chỉnh
    myQTR.begin(qtrPins, sensorCount);
    myQTR.calibrate(2); // Dùng LED chân 2 báo hiệu khi calibrate

    // SET SETPOINT CHO PID
    linePID.setSetpoint(3500); // Line ở giữa = 3500
    linePID.setDeadzone(20);   // Tăng deadzone để giảm phản ứng với lỗi nhỏ

    // Phát WiFi
    WiFi.softAP(ssid, password);
    Serial.println("WiFi Started!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());

    // --- CÁC ĐƯỜNG DẪN WEB ---
    server.on("/", []() { server.send(200, "text/html", getHTML()); });
    
    // Endpoint trả về dữ liệu JSON
        server.on("/data", []() {

        String json = "{\"speed\":" + String(current_speed) + "}";

        server.send(200, "application/json", json);
    });

    server.on("/left", []() {
        controlMotor(-current_speed, current_speed+8);
        server.send(200, "text/plain", "OK");
    });

    server.on("/right", []() {
        controlMotor(current_speed, -current_speed-8);
        server.send(200, "text/plain", "OK");
    });

    server.on("/forward", []() {
        controlMotor(current_speed, current_speed+8);
        server.send(200, "text/plain", "OK");
    });

    server.on("/backward", []() {
        controlMotor(-current_speed, -current_speed-8);
        server.send(200, "text/plain", "OK");
    });

    server.on("/stop", []() {
        stopMotor();
        server.send(200, "text/plain", "OK");
    });

    server.on("/toggle_gripper", []() {
        if (gripper.getState()) gripper.close();
        else gripper.open();

        delay(300); // 🟢 cực quan trọng: cho servo chạy xong
    });

    server.on("/servo_angle", []() {
        if (server.hasArg("value")) {
            int angle = server.arg("value").toInt();
            angle = constrain(angle, 0, 80);
            gripper.setAngle(angle);
            servoAngle = angle;
        }
        server.send(200, "text/plain", "OK");
    });

    server.on("/speed_up", []() {
        if (current_speed < MAX_SPEED) {
            current_speed += 5;
            if (current_speed > MAX_SPEED) current_speed = MAX_SPEED;
        }
        server.send(200, "text/plain", String(current_speed));
    });

    server.on("/speed_down", []() {
        if (current_speed > MIN_SPEED) {
            current_speed -= 5;
            if (current_speed < MIN_SPEED) current_speed = MIN_SPEED;
        }
        server.send(200, "text/plain", String(current_speed));
    });

    server.on("/turn_off_wifi", []() {
        disableWiFi();
        server.send(200, "text/plain", "WiFi turned off");
    });

    server.begin();
}

void loop() {
    server.handleClient(); // Lắng nghe yêu cầu từ web
    
    // Nếu WiFi đã tắt, chuyển sang chế độ dò line
    if (WiFi.getMode() == WIFI_OFF) {
        followLine();
    }
}