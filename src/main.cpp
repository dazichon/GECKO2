
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

void updateAverageSpeed()
{
    current_speed = (current_speed_left + current_speed_right) / 2;
}

// Khởi tạo QTR8A
QTR8A_Handler myQTR;
const uint8_t qtrPins[] = {39, 34, 35, 33, 36, 32}; // 6 sensors D2-D7
const uint8_t sensorCount = 6;

// Khởi tạo PID cho dò line
float pidKp = 0.13;
float pidKi = 0.0;
float pidKd = 0.095;
PID linePID(pidKp, pidKi, pidKd, -150, 150); // Kp, Ki, Kd
bool followLineMode = false;
bool invertedLine = false;
int lostLineCount = 0;
bool firstLeftTurnDone = false;
bool specialForwardDone = false;
bool wasLostLine = false;
uint16_t lastValidPosition = 2500;
const int LINE_THRESHOLD = 600;

// Giao diện Web (HTML + CSS + JS cập nhật cảm biến)
String getHTML()
{
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

    // Hiển thị và điều khiển tốc độ từng bánh
    html += "<div class='control-row' style='flex-direction:column; align-items:center;'>";
    html += "<div style='color: #f1c40f; font-weight: bold; margin: 10px 0;'>Tốc độ trái: <span id='speedLeft'>" + String(current_speed_left) + "</span>/" + String(MAX_SPEED) + "</div>";
    html += "<div style='color: #f1c40f; font-weight: bold; margin: 10px 0;'>Tốc độ phải: <span id='speedRight'>" + String(current_speed_right) + "</span>/" + String(MAX_SPEED) + "</div>";
    html += "</div>";
    html += "<div class='control-row'>";
    html += "<button type='button' onclick='sendCommand(\"/speed_down_left\")'>L-</button>";
    html += "<button type='button' onclick='sendCommand(\"/speed_up_left\")'>L+</button>";
    html += "<button type='button' onclick='sendCommand(\"/speed_down_right\")'>R-</button>";
    html += "<button type='button' onclick='sendCommand(\"/speed_up_right\")'>R+</button>";
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
    html += "<div class='control-row' style='flex-direction:column; align-items:center; width:100%;'>";
    html += "<div style='color:#1abc9c; font-weight:bold;'>QTR Sensors</div>";
    html += "<div id='qtrBox' style='font-family:monospace; color:#fff;'></div>";
    html += "<div style='color:#f1c40f;'>Position: <span id='posVal'>0</span></div>";
    html += "</div>";
    // Nút gắp vẫn dùng onclick để bấm một lần là đổi trạng thái
    html += "<div class='control-row'><button type='button' class='gripper' onclick='sendCommand(\"/toggle_gripper\")'>GAP / NHA (SERVO)</button></div>";
    html += "<div class='control-row' style='flex-direction:column; align-items:center; width:100%;'>";
    html += "  <label style='width:100%; color:#f1c40f; font-weight:bold; margin-bottom:8px;'>Servo góc: <span id='servoAngle'>" + String(servoAngle) + "</span>°</label>";
    html += "  <input id='servoSlider' type='range' min='0' max='80' step='1' value='" + String(servoAngle) + "' style='width:100%; margin-bottom:12px;' oninput='updateServo(this.value)' />";
    html += "</div>";
    html += "<div class='control-row' style='flex-direction:column; align-items:center; width:100%; max-width:420px; margin-top:14px;'>";
    html += "<div style='color:#1abc9c; font-weight:bold; margin-bottom:10px;'>Tune PID dò line</div>";
    html += "<label style='width:100%; text-align:left;'><span style='float:left;'>Kp:</span><span id='kpValue' style='float:right;'>" + String(pidKp, 2) + "</span></label>";
    html += "<input id='kpSlider' type='range' min='0' max='2' step='0.01' value='" + String(pidKp, 2) + "' style='width:100%;' oninput='updatePid(\"kp\", this.value)' />";
    html += "<label style='width:100%; text-align:left; margin-top:8px;'><span style='float:left;'>Ki:</span><span id='kiValue' style='float:right;'>" + String(pidKi, 2) + "</span></label>";
    html += "<input id='kiSlider' type='range' min='0' max='1' step='0.001' value='" + String(pidKi, 3) + "' style='width:100%;' oninput='updatePid(\"ki\", this.value)' />";
    html += "<label style='width:100%; text-align:left; margin-top:8px;'><span style='float:left;'>Kd:</span><span id='kdValue' style='float:right;'>" + String(pidKd, 2) + "</span></label>";
    html += "<input id='kdSlider' type='range' min='0' max='1' step='0.001' value='" + String(pidKd, 3) + "' style='width:100%;' oninput='updatePid(\"kd\", this.value)' />";
    html += "<button type='button' style='margin-top:14px; width:100%;' onclick='toggleLineMode()'>BẬT/TẮT dò line</button>";
    html += "<div style='margin-top:10px; color:#f1c40f;'>Trạng thái dò line: <span id=\"lineMode\">OFF</span></div>";
    html += "</div>";
    html += "<div class='control-row'><button type='button' class='stop' onclick='sendCommand(\"/turn_off_wifi\")'>TAT WIFI</button></div>";
    // Script JS để lấy dữ liệu cảm biến mỗi 100ms
    html += "<script>";
    html += "function sendCommand(path) { fetch(path).catch(() => {}); }";
    html += "function updatePid(name, value) { value = parseFloat(value); if (isNaN(value)) return; fetch('/set_pid?' + name + '=' + value).catch(() => {});";
    html += " if (name === 'kp') document.getElementById('kpValue').innerText = value.toFixed(2);";
    html += " if (name === 'ki') document.getElementById('kiValue').innerText = value.toFixed(3);";
    html += " if (name === 'kd') document.getElementById('kdValue').innerText = value.toFixed(3); }";
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
    html += "}, 200);"; // Cập nhật mỗi 200ms
    html += "</script>";

    html += "</center></div></body></html>";
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
void followLine()
{
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();

    // 1. Cố định thời gian lấy mẫu 10ms để PID không bị nhiễu
    if (currentTime - lastTime < 10)
        return;

    float dt = (currentTime - lastTime) / 1000.0;

    lastTime = currentTime;
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

        // chỉ đếm 1 lần cho mỗi lần mất line
        if (!wasLostLine)
        {
            lostLineCount++;
            wasLostLine = true;

            Serial.print("Lost Line Count: ");
            Serial.println(lostLineCount);
        }

        // tới lần thứ 3 mới rẽ trái
        if (lostLineCount >= 1)
        {

            controlMotor(-60, 90);
            delay(40);

            // reset bộ đếm
            lostLineCount = 0;
            firstLeftTurnDone = true; // Đánh dấu đã xong đoạn cua trái đầu tiên
        }

        position = lastValidPosition;
    }
    else
    {

        wasLostLine = false;

        lastValidPosition = position;

        // Logic đặc biệt: Sau cua trái đầu tiên, nếu thấy 3 mắt trái (0,1,2) cùng đen thì tiến lên 1 tí
        if (firstLeftTurnDone && !specialForwardDone)
        {
            if (myQTR.getSensorValue(0) > LINE_THRESHOLD && myQTR.getSensorValue(1) > LINE_THRESHOLD && myQTR.getSensorValue(2) > LINE_THRESHOLD)
            {
                controlMotor(current_speed_left, current_speed_right); // Tiến thẳng
                delay(150); // Tiến lên 1 khoảng nhỏ (150ms)
                specialForwardDone = true; // Đánh dấu chỉ thực hiện 1 lần duy nhất
                Serial.println(">>> SPECIAL FORWARD AFTER FIRST TURN EXECUTED <<<");
            }
        }
    }

    // 3. Tính toán PID thẳng đều, không có xử lý cua gắt
    float correctionRaw = linePID.compute(position, dt);
    static float correctionFiltered = 0;
    correctionFiltered = correctionFiltered * 0.65 + correctionRaw * 0.35;
    int correction = (int)correctionFiltered;

    // Thêm offset để cân bằng motor (lệch trái thì motor phải yếu hơn)
    const int motorBalance = 3; // Tăng tốc độ motor phải thêm 3

    int leftSpeed = current_speed + correction;
    int rightSpeed = current_speed - correction + motorBalance;

    // Không quay lùi; chỉ cho phép điều chỉnh tốc độ hai bánh để giữ thẳng
    leftSpeed = constrain(leftSpeed, -80, 150);
    rightSpeed = constrain(rightSpeed, -80, 150);

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

    if (!followLineMode) {
        stopMotor();
        lostLineCount = 0;
        wasLostLine = false;
        firstLeftTurnDone = false; // Reset lại khi tắt/bật mode để có thể test lại
        specialForwardDone = false;
        lastValidPosition = 2500;

        linePID.reset(); // Reset PID khi tắt chế độ dò line để tránh tích tụ lỗi cũ khi bật lại
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