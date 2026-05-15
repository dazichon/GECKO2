
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "encoder_motor.h"
#include "MyServoControl.h"
#include "variables.h"
#include "QTR8A_Handler.h" // Thư viện mình vừa tạo
#include "PID.h"           // Thêm PID
#include "esp_wifi.h"

// Cấu hình WiFi
const char *ssid = "da";
const char *password = "ducanh1110";

WebServer server(80);
MyServoControl gripper(servo_pin, 0, 0, 60);

// Biến điều khiển servo
int servoAngle = 0; // Giá trị mặc định của slider

// Biến điều khiển tốc độ
int current_speed = 80;
int current_speed_left = 80;
int current_speed_right = 96;
const int MIN_SPEED = 10;
const int MAX_SPEED = 200;

// Biến điều khiển tốc độ dò line
int lineFollowSpeed = 60;  // Tốc độ riêng cho dò line
unsigned long lineFollowStartTime = 0;  // Thời điểm bắt đầu dò line
bool speedIncreased = false;  // Đã tăng tốc độ chưa

void updateAverageSpeed()
{
    current_speed = (current_speed_left + current_speed_right) / 2;
}

// Khởi tạo QTR8A
QTR8A_Handler myQTR;
const uint8_t qtrPins[] = {39, 34, 35, 33, 36, 32}; // 6 sensors D2-D7
const uint8_t sensorCount = 6;

// Khởi tạo PID cho dò line
float pidKp = 0.5;
float pidKi = 0.003;
float pidKd = 0.095;
PID linePID(pidKp, pidKi, pidKd, -150, 150); // Kp, Ki, Kd
bool followLineMode = false;
bool invertedLine = false;
int lostLineCount = 0;
bool firstLeftTurnDone = false;
bool specialForwardDone = false;
bool wasLostLine = false;
bool afterLeftTurn = false;
unsigned long leftTurnFinishTime = 0;
uint16_t lastValidPosition = 2500;
const int LINE_THRESHOLD = 600;

// Giao diện Web (HTML + CSS + JS cập nhật cảm biến)
String getHTML()
{
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>";
    html += "html, body { margin:0; padding:0; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: linear-gradient(135deg, #0f1419 0%, #1a1f2e 100%); color:#e0e0e0; user-select:none; -webkit-user-select:none; -ms-user-select:none; }";
    html += "body { min-height:100vh; display:flex; align-items:flex-start; justify-content:center; padding-top:20px; }";
    html += "button { width: 100px; height: 45px; margin: 6px; font-size: 14px; font-weight:600; border-radius: 8px; border: none; background: linear-gradient(135deg, #3498db, #2980b9); color: white; cursor: pointer; outline: none; user-select:none; -webkit-user-select:none; touch-action: manipulation; -webkit-tap-highlight-color: transparent; box-shadow: 0 2px 8px rgba(0,0,0,0.3); transition: all 0.2s; }";
    html += "button:hover { transform: translateY(-2px); box-shadow: 0 4px 12px rgba(52, 152, 219, 0.4); }";
    html += "button:active { transform: scale(0.98); }";
    html += ".stop { background: linear-gradient(135deg, #e74c3c, #c0392b); }";
    html += ".stop:hover { box-shadow: 0 4px 12px rgba(231, 76, 60, 0.4); }";
    html += ".gripper { background: linear-gradient(135deg, #2ecc71, #27ae60); width: 100%; }";
    html += ".gripper:hover { box-shadow: 0 4px 12px rgba(46, 204, 113, 0.4); }";
    html += ".control-row { display:flex; justify-content:center; align-items:center; gap: 8px; flex-wrap: wrap; margin-top: 12px; }";
    html += ".panel { text-align:center; width: 100%; max-width: 500px; padding: 25px; box-sizing:border-box; background: rgba(30, 35, 50, 0.8); border-radius: 12px; box-shadow: 0 8px 32px rgba(0,0,0,0.5); border: 1px solid rgba(255,255,255,0.1); backdrop-filter: blur(10px); }";
    html += "h1 { margin: 0 0 20px 0; color: #3498db; font-size: 28px; font-weight: 700; text-shadow: 0 2px 4px rgba(0,0,0,0.3); }";
    html += ".section-title { color: #2ecc71; font-weight: 700; margin: 16px 0 12px 0; font-size: 14px; text-transform: uppercase; letter-spacing: 0.5px; border-bottom: 2px solid rgba(46, 204, 113, 0.3); padding-bottom: 8px; }";
    html += ".info-display { background: rgba(0,0,0,0.3); padding: 12px; border-radius: 8px; margin: 8px 0; border-left: 3px solid #3498db; }";
    html += ".info-label { color: #bdc3c7; font-size: 12px; text-transform: uppercase; letter-spacing: 0.5px; margin-bottom: 4px; }";
    html += ".info-value { color: #ecf0f1; font-size: 16px; font-weight: 700; font-family: 'Courier New', monospace; }";
    html += ".pid-control { background: rgba(0,0,0,0.2); padding: 14px; border-radius: 8px; margin: 10px 0; border: 1px solid rgba(46, 204, 113, 0.2); }";
    html += ".slider-label { display: flex; justify-content: space-between; align-items: center; margin: 10px 0 6px 0; font-size: 13px; font-weight: 600; }";
    html += ".slider-label-name { color: #ecf0f1; }";
    html += ".slider-value { color: #2ecc71; font-family: 'Courier New', monospace; font-size: 14px; }";
    html += "input[type='range'] { width: 100%; height: 5px; border-radius: 3px; background: linear-gradient(to right, #34495e, #7f8c8d); outline: none; -webkit-appearance: none; appearance: none; }";
    html += "input[type='range']::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 18px; height: 18px; border-radius: 50%; background: linear-gradient(135deg, #3498db, #2980b9); cursor: pointer; box-shadow: 0 2px 6px rgba(52, 152, 219, 0.5); transition: all 0.2s; }";
    html += "input[type='range']::-webkit-slider-thumb:hover { width: 22px; height: 22px; box-shadow: 0 2px 12px rgba(52, 152, 219, 0.8); }";
    html += "input[type='range']::-moz-range-thumb { width: 18px; height: 18px; border-radius: 50%; background: linear-gradient(135deg, #3498db, #2980b9); cursor: pointer; border: none; box-shadow: 0 2px 6px rgba(52, 152, 219, 0.5); transition: all 0.2s; }";
    html += "</style></head><body>";
    html += "<div class='panel'>";
    html += "<h1>🤖 ROBOCON CONTROL</h1>";

    // MOVEMENT CONTROL
    html += "<div class='section-title'>Movement Control</div>";
    html += "<div class='control-row'>";
    html += "<button type='button' onpointerdown='handlePress(\"/forward\", event)' onpointerup='handleRelease(event)' onpointercancel='handleRelease(event)' onpointerleave='handleRelease(event)'>⬆ UP</button>";
    html += "</div>";
    html += "<div class='control-row'>";
    html += "<button type='button' onpointerdown='handlePress(\"/left\", event)' onpointerup='handleRelease(event)' onpointercancel='handleRelease(event)' onpointerleave='handleRelease(event)'>⬅ LEFT</button>";
    html += "<button type='button' class='stop' onclick='sendCommand(\"/stop\")'>⏹ STOP</button>";
    html += "<button type='button' onpointerdown='handlePress(\"/right\", event)' onpointerup='handleRelease(event)' onpointercancel='handleRelease(event)' onpointerleave='handleRelease(event)'>RIGHT ➡</button>";
    html += "</div>";
    html += "<div class='control-row'>";
    html += "<button type='button' onpointerdown='handlePress(\"/backward\", event)' onpointerup='handleRelease(event)' onpointercancel='handleRelease(event)' onpointerleave='handleRelease(event)'>⬇ DOWN</button>";
    html += "</div>";

    // SPEED CONTROL
    html += "<div class='section-title'>Speed Control</div>";
    html += "<div class='info-display'>";
    html += "<div class='info-label'>Left Motor</div>";
    html += "<div class='info-value'><span id='speedLeft'>" + String(current_speed_left) + "</span>/" + String(MAX_SPEED) + " RPM</div>";
    html += "</div>";
    html += "<div class='control-row'>";
    html += "<button onclick='sendCommand(\"/speed_down_left\")'>−</button>";
    html += "<button onclick='sendCommand(\"/speed_up_left\")'>+</button>";
    html += "</div>";
    
    html += "<div class='info-display'>";
    html += "<div class='info-label'>Right Motor</div>";
    html += "<div class='info-value'><span id='speedRight'>" + String(current_speed_right) + "</span>/" + String(MAX_SPEED) + " RPM</div>";
    html += "</div>";
    html += "<div class='control-row'>";
    html += "<button onclick='sendCommand(\"/speed_down_right\")'>−</button>";
    html += "<button onclick='sendCommand(\"/speed_up_right\")'>+</button>";
    html += "</div>";

    // BOTH MOTORS SPEED CONTROL
    html += "<div class='info-display'>";
    html += "<div class='info-label'>Both Motors</div>";
    html += "<div class='info-value'><span id='speedAvg'>" + String(current_speed) + "</span>/" + String(MAX_SPEED) + " RPM</div>";
    html += "</div>";
    html += "<div class='control-row'>";
    html += "<button style='width: 140px;' onclick='sendCommand(\"/speed_down\")'>−− BOTH</button>";
    html += "<button style='width: 140px;' onclick='sendCommand(\"/speed_up\")'>++ BOTH</button>";
    html += "</div>";

    // QTR SENSORS
    html += "<div class='section-title'>Line Sensors (QTR)</div>";
    html += "<div class='info-display'>";
    html += "<div id='qtrBox' style='font-family:monospace; color:#2ecc71; font-size:13px; line-height:1.6;'></div>";
    html += "</div>";
    html += "<div class='info-display'>";
    html += "<div class='info-label'>Position</div>";
    html += "<div class='info-value'><span id='posVal'>0</span>/5000</div>";
    html += "</div>";

    // GRIPPER
    html += "<div class='section-title'>Gripper</div>";
    html += "<div class='control-row'><button type='button' class='gripper' onclick='sendCommand(\"/toggle_gripper\")'>🤖 OPEN/CLOSE</button></div>";

    // SERVO
    html += "<div class='section-title'>Servo Control</div>";
    html += "<div class='pid-control'>";
    html += "<div class='slider-label'><span class='slider-label-name'>Angle</span><span class='slider-value'><span id='servoAngle'>" + String(servoAngle) + "</span>°</span></div>";
    html += "<input id='servoSlider' type='range' min='0' max='80' step='1' value='" + String(servoAngle) + "' oninput='updateServo(this.value)' />";
    html += "</div>";

    // PID TUNING
    html += "<div class='section-title'>PID Line Follower</div>";
    html += "<div class='pid-control'>";
    html += "<div class='slider-label'><span class='slider-label-name'>Kp (Proportional)</span><span class='slider-value'><span id='kpValue'>" + String(pidKp, 3) + "</span></span></div>";
    html += "<input id='kpSlider' type='range' min='0' max='1' step='0.001' value='" + String(pidKp, 3) + "' oninput='updatePid(\"kp\", this.value)' />";
    html += "</div>";
    html += "<div class='pid-control'>";
    html += "<div class='slider-label'><span class='slider-label-name'>Ki (Integral)</span><span class='slider-value'><span id='kiValue'>" + String(pidKi, 4) + "</span></span></div>";
    html += "<input id='kiSlider' type='range' min='0' max='0.2' step='0.001' value='" + String(pidKi, 4) + "' oninput='updatePid(\"ki\", this.value)' />";
    html += "</div>";
    html += "<div class='pid-control'>";
    html += "<div class='slider-label'><span class='slider-label-name'>Kd (Derivative)</span><span class='slider-value'><span id='kdValue'>" + String(pidKd, 3) + "</span></span></div>";
    html += "<input id='kdSlider' type='range' min='0' max='0.2' step='0.001' value='" + String(pidKd, 3) + "' oninput='updatePid(\"kd\", this.value)' />";
    html += "</div>";
    html += "<div class='info-display'>";
    html += "<div class='info-label'>Line Mode Status</div>";
    html += "<div class='info-value'><span id='lineMode'>OFF</span></div>";
    html += "</div>";
    html += "<div class='control-row'><button type='button' style='width:100%;' onclick='toggleLineMode()'>⚙ TOGGLE LINE MODE</button></div>";

    // WIFI
    html += "<div class='section-title'>System</div>";
    html += "<div class='control-row'><button type='button' class='stop' style='width:100%;' onclick='sendCommand(\"/turn_off_wifi\")'>⊘ TURN OFF WiFi</button></div>";

    // JAVASCRIPT
    html += "<script>";
    html += "function sendCommand(path) { fetch(path).catch(() => {}); }";
    html += "function updatePid(name, value) { value = parseFloat(value); if (isNaN(value)) return; fetch('/set_pid?' + name + '=' + value).catch(() => {});";
    html += " if (name === 'kp') document.getElementById('kpValue').innerText = value.toFixed(5);";
    html += " if (name === 'ki') document.getElementById('kiValue').innerText = value.toFixed(2);";
    html += " if (name === 'kd') document.getElementById('kdValue').innerText = value.toFixed(5); }";
    html += "function toggleLineMode() { fetch('/toggle_line').catch(() => {}); }";
    html += "function handlePress(path, event) { event.preventDefault(); sendCommand(path); }";
    html += "function handleRelease(event) { event.preventDefault(); sendCommand('/stop'); }";
    html += "function updateServo(value) { document.getElementById('servoAngle').innerText = value; fetch('/servo_angle?value=' + value).catch(() => {}); }";
    html += "setInterval(function() {";
    html += "  fetch('/data').then(response => response.json()).then(data => {";
    html += "    document.getElementById('speedLeft').innerText = data.speed_left;";
    html += "    document.getElementById('speedRight').innerText = data.speed_right;";
    html += "    document.getElementById('lineMode').innerText = data.line_mode ? 'ON' : 'OFF';";
    html += "    document.getElementById('posVal').innerText = data.position;";
    html += "    let qtrHtml = '';";
    html += "    data.qtr.forEach((v, i) => { qtrHtml += 'S' + i + ': ' + v + (i === 2 ? '<br>' : (i === 5 ? '' : ' | ')); });";
    html += "    document.getElementById('qtrBox').innerHTML = qtrHtml;";
    html += "  }).catch(() => {});";
    html += "}, 200);";
    html += "</script>";

    html += "</div></body></html>";
    return html;
}
void disableWiFi()
{
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    esp_wifi_stop(); // đảm bảo radio dừng hẳn
    delay(200);      // chờ ADC2 ổn định
}
void updateLineColorMode()
{
    // Lấy giá trị cảm biến hiện tại (Black = 1000, White = 0)
    int s0 = myQTR.getSensorValue(0);
    int s5 = myQTR.getSensorValue(5);
    int s2 = myQTR.getSensorValue(2);
    int s3 = myQTR.getSensorValue(3);

    // 1. Nếu 2 mắt ngoài cùng (S0 và S5) cùng thấy đen -> Chuyển sang follow line trắng
    if (s0 > LINE_THRESHOLD && s5 > LINE_THRESHOLD)
    {
        if (!invertedLine)
        {
            invertedLine = true;
            Serial.println("DETECTED OUTER BLACK: SWITCH TO WHITE LINE MODE");
        }
    }
    // 2. Nếu thấy line đen ở các mắt giữa (S2 hoặc S3) -> Quay lại follow line đen
    else if (s2 > LINE_THRESHOLD || s3 > LINE_THRESHOLD)
    {
        if (invertedLine)
        {
            invertedLine = false;
            Serial.println("DETECTED CENTER BLACK: SWITCH TO BLACK LINE MODE");
        }
    }
}

// Kiểm tra xem tất cả cảm biến có đều là 1000 không
bool allSensorsAreMax()
{
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        if (myQTR.getSensorValue(i) != 1000)
        {
            return false;
        }
    }
    return true;
}
void followLine()
{
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();

    // 1. Cố định thời gian lấy mẫu 10ms để PID không bị nhiễu
    if (currentTime - lastTime < 10)
        return;

    float dt = (currentTime - lastTime) / 1000.0;

    lastTime = currentTime;

    // Kiểm tra xem đã 11 giây chưa, nếu có thì tăng tốc độ
    if (!speedIncreased && (currentTime - lineFollowStartTime) >= 15500)
    {
        lineFollowSpeed = 80;
        speedIncreased = true;
        Serial.println("11 seconds elapsed: Line follow speed increased to 80");
    }

    // 2. Lấy vị trí từ 6 cảm biến (phạm vi 0-5000, center = 2500)
    updateLineColorMode(); // Cập nhật trạng thái invertedLine trước
    uint16_t position = myQTR.getPosition(invertedLine); // Sử dụng trạng thái invertedLine mới nhất

    bool lostLine = true;

    // kiểm tra còn thấy line không
    for (uint8_t i = 0; i < sensorCount; i++)
    {

        int sensorValue = myQTR.getSensorValue(i);

        bool detectLine;

        if (!invertedLine)
        {
            detectLine = sensorValue > 300;
        }
        else
        {
            detectLine = sensorValue < 300;
        }

        if (detectLine)
        {
            lostLine = false;
            break;
        }
    }

    // mất line hoàn toàn
    if (lostLine)
    {
        Serial.println("LOST LINE");

        if (allSensorsAreMax())
        {
            Serial.println("ALL SENSORS MAX - TURN LEFT");
            controlMotor(30, 130);
            return;
        }

        if (lastValidPosition < 2500)
        {
            Serial.println("Searching LEFT");
            controlMotor(-70, 120);
        }
        else
        {
            Serial.println("Searching RIGHT");
            controlMotor(120, -70);
        }

        return;
    }
    else
    {
        wasLostLine = false;
        lastValidPosition = position;
    }

    // 3. Tính toán PID thẳng đều, không có xử lý cua gắt
    float correctionRaw = linePID.compute(position, dt);
    static float correctionFiltered = 0;
    correctionFiltered = correctionFiltered * 0.70 + correctionRaw * 0.3;
    int correction = (int)correctionFiltered;

    // Thêm offset để cân bằng motor (lệch trái thì motor phải yếu hơn)
    const int motorBalance = 3; // Tăng tốc độ motor phải thêm 3

    int leftSpeed = lineFollowSpeed + correction;
    int rightSpeed = lineFollowSpeed - correction + motorBalance;

    // Không quay lùi; chỉ cho phép điều chỉnh tốc độ hai bánh để giữ thẳng
    leftSpeed = constrain(leftSpeed, 0, 150);
    rightSpeed = constrain(rightSpeed, 0, 150);

    controlMotor(leftSpeed, rightSpeed);

    // In giá trị cảm biến
    Serial.print(position);

    for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print("S");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(myQTR.getSensorValue(i));

        // Nếu chưa đủ 6 sensor và chưa phải sensor cuối
        if ((i + 1) % 6 != 0 && i < sensorCount - 1)
        {
            Serial.print(" | ");
        }

        // Xuống dòng sau mỗi 6 sensor hoặc sensor cuối cùng
        if ((i + 1) % 6 == 0 || i == sensorCount - 1)
        {
            Serial.println();
        }
    }
}
void setup()
{
    Serial.begin(115200);

    server.on("/set_pid", []()
              {

    if (server.hasArg("kp")) {
        pidKp = server.arg("kp").toFloat();
    }

    if (server.hasArg("ki")) {
        pidKi = server.arg("ki").toFloat();
    }

    if (server.hasArg("kd")) {
        pidKd = server.arg("kd").toFloat();
    }

    // cập nhật PID runtime
    linePID.setTunings(pidKp, pidKi, pidKd);

    Serial.print("New PID -> ");
    Serial.print("Kp: ");
    Serial.print(pidKp);

    Serial.print(" Ki: ");
    Serial.print(pidKi);

    Serial.print(" Kd: ");
    Serial.println(pidKd);

    server.send(200, "text/plain", "PID Updated"); });
    server.on("/toggle_line", []()
              {

    followLineMode = !followLineMode;
    
    if (followLineMode) {
        // Bắt đầu dò line, reset timer
        lineFollowStartTime = 0;
        speedIncreased = false;
    }

    if (!followLineMode) {
        stopMotor();
        lostLineCount = 0;
        wasLostLine = false;
        firstLeftTurnDone = false; // Reset lại khi tắt/bật mode để có thể test lại
        specialForwardDone = false;
        afterLeftTurn = false;
        lastValidPosition = 2500;
        
        // Reset biến tốc độ và timer
        lineFollowSpeed = 50;  // Reset tốc độ về 50
        speedIncreased = false;  // Reset biến tăng tốc độ
        lineFollowStartTime = 0;  // Reset thời điểm bắt đầu

        linePID.reset(); // Reset PID khi tắt chế độ dò line để tránh tích tụ lỗi cũ khi bật lại
    }
    else if (lineFollowStartTime == 0)
    {
        // Khi bất đầu dò line, lưu thời điểm bắt đầu
        lineFollowStartTime = millis();
        Serial.println("Line follow mode started - timer begins");
    }

    Serial.print("Follow Line: ");
    Serial.println(followLineMode ? "ON" : "OFF");

    server.send(200, "text/plain", followLineMode ? "ON" : "OFF"); });
    // Khởi tạo motor và servo
    initEncoderMotor();
    gripper.begin();

    // Khởi tạo QTR8A và cân chỉnh
    myQTR.begin(qtrPins, sensorCount);
    myQTR.calibrate(2); // Dùng LED chân 2 báo hiệu khi calibrate

    // SET SETPOINT CHO PID
    linePID.setSetpoint(2500); // Line ở giữa = 2500 cho 6 mắt
    linePID.setDeadzone(25);   // Deadzone 25 để lọc nhiễu nhưng vẫn phản hồi tốt

    // Phát WiFi
    WiFi.mode(WIFI_AP);        // Đảm bảo ESP32 ở chế độ phát Access Point
    WiFi.softAP(ssid, password);
    Serial.println("WiFi Started!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());

    // --- CÁC ĐƯỜNG DẪN WEB ---
    server.on("/", []()
              { server.send(200, "text/html", getHTML()); });

    // Endpoint trả về dữ liệu JSON
    server.on("/data", []()
              {
    uint16_t pos = myQTR.getPosition(invertedLine); // Cập nhật dữ liệu cảm biến mới nhất
    String json = "{\"speed_left\":" + String(current_speed_left)
        + ",\"speed_right\":" + String(current_speed_right)
        + ",\"speed_avg\":" + String(current_speed)
        + ",\"line_mode\":" + String(followLineMode ? "true" : "false")
        + ",\"position\":" + String(pos)
        + ",\"qtr\":[";

    for (uint8_t i = 0; i < sensorCount; i++)
    {
        json += String(myQTR.getSensorValue(i));
        if (i < sensorCount - 1) json += ",";
    }

    json += "]}";

    server.send(200, "application/json", json); });

    server.on("/left", []()
              {
        controlMotor(-current_speed_left, current_speed_right);
        server.send(200, "text/plain", "OK"); });

    server.on("/right", []()
              {
        controlMotor(current_speed_left, -current_speed_right);
        server.send(200, "text/plain", "OK"); });

    server.on("/forward", []()
              {
        controlMotor(current_speed_left, current_speed_right);
        server.send(200, "text/plain", "OK"); });

    server.on("/backward", []()
              {
        controlMotor(-current_speed_left, -current_speed_right);
        server.send(200, "text/plain", "OK"); });

    server.on("/stop", []()
              {
        stopMotor();
        server.send(200, "text/plain", "OK"); });

    server.on("/toggle_gripper", []()
              {
                  if (gripper.getState())
                      gripper.close();
                  else
                      gripper.open();

                  delay(300); // 🟢 cực quan trọng: cho servo chạy xong
              });

    server.on("/servo_angle", []()
              {
        if (server.hasArg("value")) {
            int angle = server.arg("value").toInt();
            angle = constrain(angle, 0, 80);
            gripper.setAngle(angle);
            servoAngle = angle;
        }
        server.send(200, "text/plain", "OK"); });

    server.on("/speed_up_left", []()
              {
        if (current_speed_left < MAX_SPEED) {
            current_speed_left += 1;
            if (current_speed_left > MAX_SPEED) current_speed_left = MAX_SPEED;
            updateAverageSpeed();
        }
        server.send(200, "text/plain", String(current_speed_left)); });

    server.on("/speed_down_left", []()
              {
        if (current_speed_left > MIN_SPEED) {
            current_speed_left -= 1;
            if (current_speed_left < MIN_SPEED) current_speed_left = MIN_SPEED;
            updateAverageSpeed();
        }
        server.send(200, "text/plain", String(current_speed_left)); });

    server.on("/speed_up_right", []()
              {
        if (current_speed_right < MAX_SPEED) {
            current_speed_right += 1;
            if (current_speed_right > MAX_SPEED) current_speed_right = MAX_SPEED;
            updateAverageSpeed();
        }
        server.send(200, "text/plain", String(current_speed_right)); });

    server.on("/speed_down_right", []()
              {
        if (current_speed_right > MIN_SPEED) {
            current_speed_right -= 1;
            if (current_speed_right < MIN_SPEED) current_speed_right = MIN_SPEED;
            updateAverageSpeed();
        }
        server.send(200, "text/plain", String(current_speed_right)); });

    // Backward-compatible endpoints that adjust both motors together
    server.on("/speed_up", []()
              {
        if (current_speed_left < MAX_SPEED) {
            current_speed_left += 5;
            if (current_speed_left > MAX_SPEED) current_speed_left = MAX_SPEED;
        }
        if (current_speed_right < MAX_SPEED) {
            current_speed_right += 5;
            if (current_speed_right > MAX_SPEED) current_speed_right = MAX_SPEED;
        }
        updateAverageSpeed();
        server.send(200, "text/plain", String(current_speed)); });

    server.on("/speed_down", []()
              {
        if (current_speed_left > MIN_SPEED) {
            current_speed_left -= 5;
            if (current_speed_left < MIN_SPEED) current_speed_left = MIN_SPEED;
        }
        if (current_speed_right > MIN_SPEED) {
            current_speed_right -= 5;
            if (current_speed_right < MIN_SPEED) current_speed_right = MIN_SPEED;
        }
        updateAverageSpeed();
        server.send(200, "text/plain", String(current_speed)); });

    server.on("/turn_off_wifi", []()
              {
        disableWiFi();
        server.send(200, "text/plain", "WiFi turned off"); });

    server.begin();
    // disableWiFi(); // Tắt WiFi ngay sau khi khởi động để vào chế độ dò line luôn, không cần phải đợi lệnh từ web. Vì nếu đã tắt WiFi thì cũng không còn cách nào khác để điều khiển nữa, chỉ có thể dò line tự động thôi.
}

void loop()
{
    server.handleClient(); // Lắng nghe yêu cầu từ web

    // Nếu WiFi đã tắt, chuyển sang chế độ dò line
    if (followLineMode)
    {
        followLine();
    }

    // followLine(); // Luôn dò line, dù có WiFi hay không. Nếu WiFi tắt thì vẫn dò line bình thường, chỉ là không nhận lệnh từ web nữa.
}