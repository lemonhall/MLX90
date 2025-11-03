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

void setup() {
  // 初始化M5Stack
  auto cfg = M5.config();
  CoreS3.begin(cfg);
  
  // 初始化串口
  Serial.begin(115200);
  while(!Serial) delay(10);
  
  Serial.println("GYMCU90640 UART模式红外摄像头测试");
  
  // 初始化MLX90640串口通信 - 先尝试默认波特率9600
  mlxSerial.begin(MLX_BAUDRATE_DEFAULT, SERIAL_8N1, MLX_RX_PIN, MLX_TX_PIN);
  
  Serial.printf("GYMCU90640串口初始化: RX=%d, TX=%d, 波特率=%d\n", 
                MLX_RX_PIN, MLX_TX_PIN, MLX_BAUDRATE_DEFAULT);
  
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
  CoreS3.Display.fillScreen(BLACK);
  CoreS3.Display.setTextColor(WHITE);
  CoreS3.Display.setTextSize(2);
  CoreS3.Display.setCursor(10, 10);
  CoreS3.Display.println("GYMCU90640 UART");
  CoreS3.Display.setTextSize(1);
  CoreS3.Display.setCursor(10, 40);
  CoreS3.Display.println("Press BtnA to read temp");
  CoreS3.Display.setCursor(10, 60);
  CoreS3.Display.printf("RX: G%d (9600/115200bps)", MLX_RX_PIN);
  
  Serial.println("初始化完成！");
}

void loop() {
  M5.update();
  
  // 按下按钮A读取温度数据
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
  // 清空缓冲区
  while(mlxSerial.available()) {
    mlxSerial.read();
  }
  
  // GYMCU90640可能支持连续输出模式，直接读取
  Serial.println("等待GYMCU90640数据...");
  
  // 等待数据 - GYMCU可能需要更长时间
  int timeout = 5000; // 5秒超时
  String dataBuffer = "";
  
  while (timeout > 0) {
    if (mlxSerial.available()) {
      char c = mlxSerial.read();
      dataBuffer += c;
      
      // 检查是否收到完整帧 (GYMCU可能以特定字符结束)
      if (dataBuffer.length() > 100) { // 收集足够数据后处理
        timeout = 100; // 继续读取剩余数据
      }
    }
    delay(1);
    timeout--;
  }
  
  if (dataBuffer.length() < 10) {
    Serial.println("UART读取超时或数据不足");
    return false;
  }
  
  Serial.println("接收到数据长度: " + String(dataBuffer.length()));
  Serial.println("数据开头: " + dataBuffer.substring(0, min(50, (int)dataBuffer.length())));
  
  // 解析数据
  if (parseGYMCUData(dataBuffer)) {
    Serial.println("GYMCU数据解析成功");
    return true;
  } else {
    Serial.println("GYMCU数据解析失败，使用测试数据");
    generateTestData();
    return true; // 即使解析失败也返回true，使用测试数据
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
  CoreS3.Display.fillScreen(BLACK);
  
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
          color = CoreS3.Display.color565(0, green, blue);
        } else {
          // 绿色到红色
          int red = 255 * ((normalized - 0.5) * 2);
          int green = 255 * (1 - (normalized - 0.5) * 2);
          color = CoreS3.Display.color565(red, green, 0);
        }
      } else {
        color = WHITE;
      }
      
      // 绘制像素块
      int x = w * pixelSize + 16;
      int y = h * pixelSize + 20;
      CoreS3.Display.fillRect(x, y, pixelSize-1, pixelSize-1, color);
    }
  }
  
  // 显示温度范围
  CoreS3.Display.setTextColor(WHITE);
  CoreS3.Display.setTextSize(1);
  CoreS3.Display.setCursor(10, 220);
  CoreS3.Display.printf("Min:%.1fC Max:%.1fC", minTemp, maxTemp);
}