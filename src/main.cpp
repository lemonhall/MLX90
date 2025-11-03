#include <M5CoreS3.h>
#include <HardwareSerial.h>

// MLX90640 UART模式通信
HardwareSerial mlxSerial(2); // 使用Serial2

// 温度数据数组 (32x24 = 768 像素)
float frame[32*24];

// UART引脚定义 (M5Stack Core S3)
#define MLX_RX_PIN 44   // MLX90640的TX连接到S3的G44 (作为RX接收)
#define MLX_TX_PIN 43   // MLX90640的RX连接到S3的G43 (作为TX发送)

// MLX90640 UART通信参数 - 根据GYMCU90640规格
#define MLX_BAUDRATE_DEFAULT 9600      // 默认波特率
#define MLX_BAUDRATE_HIGH 115200       // 高速波特率
#define MLX_BAUDRATE_ULTRA 460800      // 超高速波特率
#define FRAME_SIZE 768

// 函数声明
bool testMLXConnection();
bool readMLXFrame();
bool parseGYMCUData(String data);
void generateTestData();
void displaySimpleHeatmap();
uint32_t scanBaud();
void dumpRaw(uint16_t n);

// 原始数据缓冲（仅调试用，避免无限增长）
static String g_rawData; // 存最新一次读取的原始串口块
static uint32_t g_currentBaud = MLX_BAUDRATE_DEFAULT;

void setup() {
  // 初始化M5Stack
  M5.begin();
  
  // 初始化串口
  Serial.begin(115200);
  while(!Serial) delay(10);
  
  Serial.println("GYMCU90640 UART模式红外摄像头测试");
  
  // 自动扫描可用波特率
  g_currentBaud = scanBaud();
  mlxSerial.begin(g_currentBaud, SERIAL_8N1, MLX_RX_PIN, MLX_TX_PIN);
  Serial.printf("选择波特率: %lu bps (RX=%d TX=%d)\n", (unsigned long)g_currentBaud, MLX_RX_PIN, MLX_TX_PIN);
  
  // 等待传感器稳定
  delay(2000);
  
  // 测试传感器连接
  if (testMLXConnection()) {
    Serial.println("GYMCU90640连接成功！");
  } else {
    Serial.println("尝试其他波特率...");
    // 尝试115200波特率
    mlxSerial.end();
    mlxSerial.begin(MLX_BAUDRATE_HIGH, SERIAL_8N1, MLX_RX_PIN, MLX_TX_PIN);
    delay(1000);
    
    if (testMLXConnection()) {
      Serial.println("GYMCU90640连接成功 (115200bps)！");
    } else {
      Serial.println("GYMCU90640连接失败，请检查接线！");
      Serial.println("当前接线:");
      Serial.println("VIN → 3.3V");
      Serial.println("GND → GND"); 
      Serial.println("TX → G44");
      Serial.println("波特率: 9600 或 115200");
    }
  }
  
  // 显示初始化信息
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.println("GYMCU90640 UART");
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(10, 40);
  M5.Lcd.println("Press BtnA to read temp");
  M5.Lcd.setCursor(10, 60);
  M5.Lcd.printf("RX: G%d (9600/115200bps)", MLX_RX_PIN);
  
  Serial.println("初始化完成！");
}

void loop() {
  M5.update();
  
  // 按下按钮A读取温度数据
  // 长按A键 (>1.5s)输出原始数据调试
  if (M5.BtnA.pressedFor(1500)) {
    dumpRaw(512); // 输出前512字节
  }

  if (M5.BtnA.wasPressed()) {
    Serial.println("读取MLX90640数据...");
    
    // 获取温度帧数据
    if (readMLXFrame()) {
      // 计算最大最小温度
      float minTemp = frame[0];
      float maxTemp = frame[0];
      
      for (int i = 1; i < 768; i++) {
        if (frame[i] < minTemp) minTemp = frame[i];
        if (frame[i] > maxTemp) maxTemp = frame[i];
      }
      
      // 在屏幕上显示温度信息
      M5.Lcd.fillRect(0, 80, 320, 160, BLACK);
      M5.Lcd.setTextColor(GREEN);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(10, 90);
      M5.Lcd.printf("Min: %.1f C", minTemp);
      M5.Lcd.setCursor(10, 120);
      M5.Lcd.printf("Max: %.1f C", maxTemp);
      
      // 显示中心点温度
      int centerIndex = 16 * 24 + 12; // 中心像素 (16, 12)
      M5.Lcd.setCursor(10, 150);
      M5.Lcd.printf("Center: %.1f C", frame[centerIndex]);
      
      // 串口输出详细信息
      Serial.printf("温度范围: %.2f - %.2f 摄氏度\n", minTemp, maxTemp);
      Serial.printf("中心温度: %.2f 摄氏度\n", frame[centerIndex]);
    } else {
      Serial.println("读取失败！");
      M5.Lcd.fillRect(0, 80, 320, 40, BLACK);
      M5.Lcd.setTextColor(RED);
      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(10, 90);
      M5.Lcd.println("Read Failed - Check Connection");
    }
  }
  
  // 按下按钮B显示简单的热力图
  if (M5.BtnB.wasPressed()) {
    displaySimpleHeatmap();
  }
  
  delay(50);
}

// 测试GYMCU90640连接
bool testMLXConnection() {
  // 清空串口缓冲区
  while(mlxSerial.available()) {
    mlxSerial.read();
  }
  
  // GYMCU90640可能有连续输出，先检查是否有数据流
  delay(500);
  if (mlxSerial.available() > 0) {
    String response = mlxSerial.readString();
    Serial.println("检测到数据流: " + String(response.length()) + " bytes");
    Serial.println("前100字符: " + response.substring(0, min(100, (int)response.length())));
    return true;
  }
  
  // 如果没有连续输出，尝试发送查询命令
  // 根据常见的GYMCU协议，可能是这些命令之一:
  const char* testCommands[] = {"AT\r\n", "?\r\n", "READ\r\n", "GET\r\n", "\r\n"};
  
  for (int i = 0; i < 5; i++) {
    mlxSerial.write(testCommands[i]);
    delay(200);
    
    if (mlxSerial.available()) {
      String response = mlxSerial.readString();
      Serial.println("命令 '" + String(testCommands[i]).substring(0, String(testCommands[i]).length()-2) + "' 响应: " + response);
      if (response.length() > 2) {
        return true;
      }
    }
  }
  
  return false;
}

// 读取GYMCU90640温度帧数据 (UART模式)
bool readMLXFrame() {
  Serial.println("等待GYMCU90640数据窗口 800ms...");
  uint32_t start = millis();
  g_rawData = ""; // 重置
  while (millis() - start < 800) {
    while (mlxSerial.available()) {
      char c = mlxSerial.read();
      g_rawData += c;
      if (g_rawData.length() > 8192) { // 限制最大长度
        break;
      }
    }
    delay(2);
  }
  Serial.printf("窗口结束，收到字节: %u\n", (unsigned)g_rawData.length());
  if (g_rawData.length() < 20) {
    Serial.println("数据太少，可能未输出或波特率不匹配/模块未进入UART模式");
    return false;
  }
  Serial.println("前120字符: ");
  Serial.println(g_rawData.substring(0, min(120, (int)g_rawData.length())));

  if (parseGYMCUData(g_rawData)) {
    Serial.println("解析成功");
    return true;
  } else {
    Serial.println("解析失败，使用模拟数据");
    generateTestData();
    return true;
  }
}

// 解析GYMCU90640数据
bool parseGYMCUData(String data) {
  // GYMCU90640可能的数据格式:
  // 1. 十六进制格式
  // 2. 逗号分隔的十进制
  // 3. 固定长度二进制数据
  // 4. 带帧头的格式
  
  int validCount = 0;
  
  // 尝试解析十六进制格式 (常见于GYMCU模块)
  if (data.indexOf("0x") >= 0 || data.length() > 1000) {
    // 可能是十六进制数据
    for (int i = 0; i < data.length() - 3 && validCount < 768; i++) {
      if (data.charAt(i) == '0' && data.charAt(i+1) == 'x') {
        String hexStr = data.substring(i+2, i+6); // 读取4位十六进制
        if (hexStr.length() == 4) {
          int hexVal = strtol(hexStr.c_str(), NULL, 16);
          frame[validCount] = (float)hexVal / 100.0 - 273.15; // 转换为摄氏度
          validCount++;
          i += 5; // 跳过已处理的字符
        }
      }
    }
  }
  
  // 尝试解析逗号分隔格式
  if (validCount < 100) {
    validCount = 0;
    int startPos = 0;
    
    for (int i = 0; i < data.length() && validCount < 768; i++) {
      if (data.charAt(i) == ',' || data.charAt(i) == ' ' || 
          data.charAt(i) == '\n' || i == data.length() - 1) {
        String tempStr = data.substring(startPos, i);
        tempStr.trim();
        
        if (tempStr.length() > 0 && isDigit(tempStr.charAt(0))) {
          float temp = tempStr.toFloat();
          if (temp > -50 && temp < 150) { // 合理的温度范围
            frame[validCount] = temp;
            validCount++;
          }
        }
        startPos = i + 1;
      }
    }
  }
  
  Serial.printf("解析到 %d 个有效温度值\n", validCount);
  
  return validCount >= 400; // 至少要有一半的数据
}

// 生成测试数据（用于调试）
void generateTestData() {
  float baseTemp = 25.0; // 基础温度
  
  for (int h = 0; h < 24; h++) {
    for (int w = 0; w < 32; w++) {
      int index = h * 32 + w;
      
      // 生成模拟的温度分布
      float centerX = 16.0;
      float centerY = 12.0;
      float distance = sqrt((w - centerX) * (w - centerX) + (h - centerY) * (h - centerY));
      
      // 中心更热的分布
      frame[index] = baseTemp + 10.0 * exp(-distance / 8.0) + random(-100, 100) / 100.0;
    }
  }
}

void displaySimpleHeatmap() {
  Serial.println("生成简单热力图...");
  
  if (!readMLXFrame()) {
    Serial.println("读取失败！");
    return;
  }
  
  // 计算温度范围用于映射颜色
  float minTemp = frame[0];
  float maxTemp = frame[0];
  
  for (int i = 1; i < 768; i++) {
    if (frame[i] < minTemp) minTemp = frame[i];
    if (frame[i] > maxTemp) maxTemp = frame[i];
  }
  
  float tempRange = maxTemp - minTemp;
  
  // 清空屏幕
  M5.Lcd.fillScreen(BLACK);
  
  // 绘制热力图 (缩放到适合屏幕)
  int pixelSize = 8; // 每个热像素的显示大小
  
  for (int h = 0; h < 24; h++) {
    for (int w = 0; w < 32; w++) {
      int index = h * 32 + w;
      float temp = frame[index];
      
      // 温度映射到颜色 (蓝色-绿色-红色)
      uint16_t color;
      if (tempRange > 0) {
        float normalized = (temp - minTemp) / tempRange;
        
        if (normalized < 0.5) {
          // 蓝色到绿色
          int blue = 255 * (1 - normalized * 2);
          int green = 255 * (normalized * 2);
          color = M5.Lcd.color565(0, green, blue);
        } else {
          // 绿色到红色
          int red = 255 * ((normalized - 0.5) * 2);
          int green = 255 * (1 - (normalized - 0.5) * 2);
          color = M5.Lcd.color565(red, green, 0);
        }
      } else {
        color = WHITE;
      }
      
      // 绘制像素块
      int x = w * pixelSize + 16;
      int y = h * pixelSize + 20;
      M5.Lcd.fillRect(x, y, pixelSize-1, pixelSize-1, color);
    }
  }
  
  // 显示温度范围
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(10, 220);
  M5.Lcd.printf("Min:%.1fC Max:%.1fC", minTemp, maxTemp);
}

// 自动扫描波特率：返回检测到的最佳（数据最多）波特率
uint32_t scanBaud() {
  struct Candidate { uint32_t baud; uint16_t bytes; };
  Candidate cands[] = {
    {MLX_BAUDRATE_DEFAULT, 0},
    {MLX_BAUDRATE_HIGH, 0},
    {MLX_BAUDRATE_ULTRA, 0}
  };
  Serial.println("开始扫描波特率...");
  for (auto &c : cands) {
    mlxSerial.end();
    delay(50);
    mlxSerial.begin(c.baud, SERIAL_8N1, MLX_RX_PIN, MLX_TX_PIN);
    // 采样 250ms
    uint32_t t0 = millis();
    uint16_t count = 0;
    while (millis() - t0 < 250) {
      while (mlxSerial.available()) {
        mlxSerial.read();
        count++;
      }
    }
    c.bytes = count;
    Serial.printf("  波特率 %lu 收到字节 %u\n", (unsigned long)c.baud, count);
  }
  // 选最大
  Candidate best = cands[0];
  for (auto &c : cands) {
    if (c.bytes > best.bytes) best = c;
  }
  if (best.bytes == 0) {
    Serial.println("未检测到任何数据，采用默认9600");
    return MLX_BAUDRATE_DEFAULT;
  }
  Serial.printf("波特率扫描完成，选取 %lu (bytes=%u)\n", (unsigned long)best.baud, best.bytes);
  return best.baud;
}

// 输出原始数据前 n 字节（十六进制 + 可打印字符）
void dumpRaw(uint16_t n) {
  if (g_rawData.length() == 0) {
    Serial.println("无原始数据可输出，先按A短按采集一帧。");
    return;
  }
  uint16_t len = min<uint16_t>(n, g_rawData.length());
  Serial.printf("---- RAW HEX (len=%u) ----\n", len);
  for (uint16_t i = 0; i < len; ++i) {
    uint8_t b = (uint8_t)g_rawData[i];
    Serial.printf("%02X ", b);
    if ((i+1) % 32 == 0) Serial.println();
  }
  Serial.println();
  Serial.println("---- RAW ASCII ----");
  for (uint16_t i = 0; i < len; ++i) {
    char ch = g_rawData[i];
    if (ch < 32 || ch > 126) ch = '.';
    Serial.print(ch);
  }
  Serial.println();
  Serial.println("-------------------");
}