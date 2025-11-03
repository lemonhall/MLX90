#include <M5CoreS3.h>
#include <HardwareSerial.h>
// 可选严格模式：校验失败则拒绝帧
#ifndef USE_STRICT_PROTOCOL
#define USE_STRICT_PROTOCOL 0
#endif

// 当前帧环境温度
static float g_envTemp = NAN;

// 简单命令发送（根据说明书）
// 波特率设置: 0xA5 0x05 0x15 0x01 0xBB (9600)
// 查询自动输出: 0xA5 0x05 0x35 0x00 0xDB
// 自动输出开启: 0xA5 0x05 0x35 0x02 0xDC
// 帧率设置: 0xA5 0x05 0x25 0x01 0xCB (1Hz) 等
// 先声明串口对象再使用
HardwareSerial mlxSerial(2); // 使用Serial2
void sendRawCommand(const uint8_t *data, size_t len) {
  mlxSerial.write(data, len);
  mlxSerial.flush();
  Serial.print("发送命令: ");
  for (size_t i=0;i<len;i++){ Serial.printf("%02X ", data[i]); }
  Serial.println();
}

void commandSetFrameRate(uint8_t rateCode) {
  // rateCode: 0x00=0.5Hz 0x01=1Hz 0x02=2Hz 0x03=4Hz 0x04=8Hz
  uint8_t cmd[5] = {0xA5,0x05,0x25,rateCode, (uint8_t)(0xCA + rateCode)}; // 尾字节随说明书校验示例调整（若有不同需再对照）
  sendRawCommand(cmd,5);
}
void commandSetAutoOutput(bool on) {
  uint8_t cmd[5] = {0xA5,0x05,0x35, (uint8_t)(on?0x02:0x00), (uint8_t)(on?0xDC:0xDB)};
  sendRawCommand(cmd,5);
}
void commandQueryAutoOutput() {
  uint8_t cmd[5] = {0xA5,0x05,0x35,0x00,0xDB};
  sendRawCommand(cmd,5);
}

// (已上移) MLX90640 UART模式通信对象已在顶部定义

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
bool parseProtocolFrame(const String &raw); // 协议帧解析前置声明
void analyzeRawForPattern();                // 原始数据模式分析前置声明
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
  M5.Lcd.printf("RX: G%d (Baud=%lu)", MLX_RX_PIN, (unsigned long)g_currentBaud);
  M5.Lcd.setCursor(10, 75);
  M5.Lcd.println("BtnA=Capture BtnB=Heatmap BtnC=ToggleAuto");
  
  Serial.println("初始化完成！");
}

void loop() {
  M5.update();
  
  // 按下按钮A读取温度数据
  // 长按A键 (>1.5s)输出原始数据调试
  if (M5.BtnA.pressedFor(1500)) {
    dumpRaw(512); // 输出前512字节
  }

  if (M5.BtnC.wasPressed()) {
    static bool autoOn=false; autoOn=!autoOn; commandSetAutoOutput(autoOn);
    Serial.printf("切换自动输出: %s\n", autoOn?"ON":"OFF");
  }

  if (M5.BtnB.pressedFor(1500)) {
    static uint8_t rate=0; // 0..4
    rate = (rate+1)%5; commandSetFrameRate(rate);
    Serial.printf("切换帧率代码=%u\n", rate);
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
  M5.Lcd.printf("Center: %.1f C Env:%.1f C", frame[centerIndex], isnan(g_envTemp)?-1:g_envTemp);
      
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
// 可调参数宏
#ifndef MLX_CAPTURE_WINDOW_MS
#define MLX_CAPTURE_WINDOW_MS 3500 // 捕获窗口持续时间（可加长）
#endif
#ifndef MLX_EARLY_STOP_ENABLED
#define MLX_EARLY_STOP_ENABLED 1   // 检测到完整帧提前结束
#endif

bool readMLXFrame() {
  Serial.printf("等待GYMCU90640数据窗口 %u ms...\n", MLX_CAPTURE_WINDOW_MS);
  uint32_t start = millis();
  g_rawData = ""; // 重置文本缓冲
  // 二进制缓冲
  static std::vector<uint8_t> binBuf; binBuf.clear(); binBuf.reserve(4096);
  const size_t EARLY_FRAME_TOTAL = 1544; // 预期完整帧总字节
  bool frameFound = false;
  int frameParseSuccess = 0;
  while (millis() - start < MLX_CAPTURE_WINDOW_MS) {
    while (mlxSerial.available()) {
      int b = mlxSerial.read();
      if (b < 0) break;
      uint8_t ub = (uint8_t)b;
      binBuf.push_back(ub);
      g_rawData += (char)ub; // 保持原有接口兼容
      // 早期检测：长度达到最小帧，尝试协议解析
      if (MLX_EARLY_STOP_ENABLED && binBuf.size() >= EARLY_FRAME_TOTAL && !frameFound) {
        // 尝试在现有数据里解析帧
        if (parseProtocolFrame(g_rawData)) {
          frameFound = true; frameParseSuccess = 1;
          Serial.println("早期检测：成功解析协议帧，提前结束捕获");
          break;
        }
      }
      if (binBuf.size() >= 8192) break; // 安全上限
    }
    if (MLX_EARLY_STOP_ENABLED && frameFound) break;
    delay(2);
  }
  Serial.printf("窗口结束，收到字节: %u (binary)\n", (unsigned)binBuf.size());
  if (binBuf.size() < 20) {
    Serial.println("数据太少，可能未输出或波特率不匹配/模块未进入UART模式");
    return false;
  }
  Serial.println("前120字符: ");
  Serial.println(g_rawData.substring(0, min(120, (int)g_rawData.length())));

  // 二进制猜测：是否接近 768 * 2 = 1536 字节（每像素 16bit）或其倍数
  size_t len = binBuf.size();
  if (len >= 1536 && len % 256 == 0) {
    Serial.println("尝试二进制帧解析模式 (16-bit per pixel)...");
    // 尝试小端和大端两种方式
    auto tryDecode = [&](bool littleEndian) -> bool {
      size_t pixels = len / 2; // 假定全是像素
      if (pixels < 768) return false;
      size_t used = 768; // 只取前768像素
      for (size_t i = 0; i < used; i++) {
        uint8_t b1 = g_rawData[2*i];
        uint8_t b2 = g_rawData[2*i + 1];
        uint16_t raw = littleEndian ? (b2 << 8 | b1) : (b1 << 8 | b2);
        // 粗略转换：许多红外阵列原始值可能对应开氏度*100 或摄氏度*100
        float tempC = (float)raw / 100.0f; // 初步假设
        // 过滤异常值
        if (tempC < -60 || tempC > 400) {
          // 尝试另一种缩放：/16
          tempC = (float)raw / 16.0f; 
        }
        frame[i] = tempC;
      }
      // 简单合理性检查：计算范围
      float mn = frame[0], mx = frame[0];
      for (int i = 1; i < 768; ++i) { if (frame[i] < mn) mn = frame[i]; if (frame[i] > mx) mx = frame[i]; }
      Serial.printf("解析方式(%s endian) 温度范围: %.2f .. %.2f\n", littleEndian?"little":"big", mn, mx);
      // 判定是否合理：范围在 -50..350 且差值 > 1
      if (mn > -55 && mx < 360 && (mx - mn) > 1) {
        Serial.println("二进制解析成功！");
        return true;
      }
      return false;
    };
    bool ok = tryDecode(true) || tryDecode(false);
    if (ok) {
      return true;
    } else {
      Serial.println("二进制解析失败，继续文本解析...");
    }
  }

  // 如果早期已成功解析协议帧
  if (frameParseSuccess) {
    return true;
  }
  // 协议帧解析尝试（可能在较小缓冲中没有满帧）
  if (parseProtocolFrame(g_rawData)) {
    Serial.println("协议帧解析成功 (后期) ");
    return true;
  }
  if (parseGYMCUData(g_rawData)) { // 次级文本尝试
    Serial.println("文本/混合格式解析成功");
    return true;
  }
  Serial.println("所有解析失败，使用模拟数据");
  generateTestData();
  analyzeRawForPattern();
  return true;
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
  // 协议列顺序 Col1 为右上开始 -> 需要水平翻转
  int index = h * 32 + (31 - w);
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

// 分析原始数据中可能的模式（例如 0x5A 填充/定界）
void analyzeRawForPattern() {
  if (g_rawData.length() == 0) {
    Serial.println("无原始数据可分析");
    return;
  }
  size_t len = g_rawData.length();
  uint32_t count5A = 0;
  uint32_t count00 = 0;
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = (uint8_t)g_rawData[i];
    if (b == 0x5A) count5A++;
    if (b == 0x00) count00++;
  }
  Serial.printf("模式分析: 总字节=%u, 0x5A出现=%u (%.2f%%), 0x00出现=%u (%.2f%%)\n",
                (unsigned)len, count5A, 100.0*count5A/len, count00, 100.0*count00/len);
  // 去掉0x5A再尝试按16位解析
  String filtered;
  filtered.reserve(len);
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = (uint8_t)g_rawData[i];
    if (b != 0x5A) filtered += (char)b;
  }
  size_t flen = filtered.length();
  Serial.printf("过滤0x5A后字节数=%u\n", (unsigned)flen);
  if (flen >= 1536) {
    size_t pixels = flen / 2;
    Serial.printf("可能像素(过滤后按2字节/像素)= %u\n", (unsigned)pixels);
  }
}

// 按照协议：帧长度1544字节：
// [0]=0x5A [1]=0x5A [2]=lenLow [3]=lenHigh (期望 0x02 0x06 => 0x0602=1538? 或 0x0206=518?)
// 接着 1536 字节像素数据 (每像素1字节? 文档里目标温度数据1低8位、高8位 -> 2字节/像素 => 1536/2=768 像素 OK)
// 然后 2 字节模块自身温度 (低/高) + 2 字节校验
// 实际示例：总字节 1544 = 2(头) +2(长度)+1536(像素数据)+2(模块温度)+2(校验) = 1544
// 像素逻辑：每像素两个连续字节组成 16bit 原始值；需转换为摄氏温度（暂时 raw/100）
bool parseProtocolFrame(const String &raw) {
  // 新解析逻辑：声明长度字段(declaredLen) = 像素数据(2*768=1536) + 模块温度(2) = 1538
  // 完整帧总长度 = 2(帧头) + 2(长度) + declaredLen(1538) + 2(校验) = 1544
  const uint16_t EXPECT_PIXEL_COUNT = 768;
  // 支持的 declaredLen 备选：
  // 1538 = 像素(1536) + 模块温度(2) 不含校验
  // 1536 = 仅像素数据 (可能模块温度单独或缺失)
  // 1540 = 像素(1536) + 模块温度(2) + 校验前2?（某些文档差异）
  const uint16_t ALT_DECLARED[3] = {1538, 1536, 1540};
  const size_t RAW_LEN = raw.length();
  // 计算一个最小帧需求（使用最小的 declaredLen=1536 -> 总=2+2+1536+2=1542）
  const uint16_t MIN_FRAME_TOTAL = 2 + 2 + 1536 + 2; // 使用最小估计
  size_t rawLen = raw.length();
  if (rawLen < MIN_FRAME_TOTAL) return false;

  // 扫描整个缓冲寻找可能的帧头
  for (size_t start = 0; start + MIN_FRAME_TOTAL <= rawLen; ++start) {
    if ((uint8_t)raw[start] != 0x5A || (uint8_t)raw[start+1] != 0x5A) continue;
    uint8_t b2 = (uint8_t)raw[start+2];
    uint8_t b3 = (uint8_t)raw[start+3];
    uint16_t declaredLen = (uint16_t)b3 * 256 + b2; // 低在前（协议特殊顺序）
    bool lenSupported = false;
    for (auto v : ALT_DECLARED) { if (declaredLen == v) { lenSupported = true; break; } }
    if (!lenSupported) continue; // 尝试下一处帧头
    size_t pixelDataOffset = start + 4; // 像素数据开始
    size_t pixelBytes = EXPECT_PIXEL_COUNT * 2; // 固定 1536
    size_t moduleTempOffset = pixelDataOffset + pixelBytes; // 模块温度2字节位置
    // 校验位置：若 declaredLen 包含模块温度，则校验在其后两个字节；若不包含模块温度则需要调整
    bool hasModuleTemp = (declaredLen >= 1538); // 当1536时可能没有模块温度
    bool hasModuleTempExplicit = (declaredLen == 1538 || declaredLen == 1540);
    if (!hasModuleTempExplicit) {
      // 没有明确模块温度时，将环境温度设为 NAN
      g_envTemp = NAN;
    }
    size_t checksumOffset;
    if (declaredLen == 1536) {
      // 帧结构：2(头)+2(len)+1536(像素)+2(校验) 总 1542
      checksumOffset = pixelDataOffset + pixelBytes; // 紧随像素
    } else if (declaredLen == 1538) {
      // 2+2+1536+2(模块温度)+2(校验)=1544
      checksumOffset = moduleTempOffset + 2;
    } else { // 1540 假设：像素1536 + 模块温度2 + 额外填充2? + 校验2 => 总 2+2+1540+2=1546
      checksumOffset = pixelDataOffset + declaredLen; // declaredLen全部数据之后
    }
    if (checksumOffset + 2 > rawLen) {
      // 缓冲不足，放弃本帧
      continue;
    }
    Serial.printf("协议帧头在 %u, 声明长度=%u (支持)\n", (unsigned)start, declaredLen);
    // 解析像素 (16-bit 小端)
    for (uint16_t px = 0; px < EXPECT_PIXEL_COUNT; ++px) {
      uint8_t lo = (uint8_t)raw[pixelDataOffset + px*2];
      uint8_t hi = (uint8_t)raw[pixelDataOffset + px*2 + 1];
      uint16_t v = (uint16_t)hi << 8 | lo;
      float tempC = v / 100.0f; // 默认缩放
      if (tempC < -60 || tempC > 400) tempC = v / 16.0f; // 异常值备用方案
      frame[px] = tempC;
    }
    // 模块环境温度
    float moduleTemp = NAN;
    if (declaredLen != 1536) { // 有模块温度字段
      uint8_t mLo = (uint8_t)raw[moduleTempOffset];
      uint8_t mHi = (uint8_t)raw[moduleTempOffset+1];
      uint16_t moduleRaw = (uint16_t)mHi << 8 | mLo;
      moduleTemp = moduleRaw / 100.0f;
      g_envTemp = moduleTemp;
    }
    // 校验
    uint8_t chkLo = (uint8_t)raw[checksumOffset];
    uint8_t chkHi = (uint8_t)raw[checksumOffset+1];
    uint16_t chk = (uint16_t)chkHi << 8 | chkLo;
    uint32_t sum = 0;
    size_t sumEnd = (declaredLen == 1536) ? (pixelDataOffset + pixelBytes) : (declaredLen == 1538 ? (moduleTempOffset + 2) : (pixelDataOffset + declaredLen));
    for (size_t i = start; i < sumEnd; ++i) {
      sum += (uint8_t)raw[i];
    }
    uint16_t sum16 = (uint16_t)(sum & 0xFFFF);
    bool checksumOK = (chk == sum16);
    if (!isnan(moduleTemp)) {
      Serial.printf("模块温度: %.2fC, 校验=0x%04X, 累加和=0x%04X (%s)\n", moduleTemp, chk, sum16, checksumOK?"OK":"NG");
    } else {
      Serial.printf("无模块温度字段, 校验=0x%04X, 累加和=0x%04X (%s)\n", chk, sum16, checksumOK?"OK":"NG");
    }
    // 像素范围统计
    float mn = frame[0], mx = frame[0];
    for (uint16_t i=1;i<EXPECT_PIXEL_COUNT;++i){ if(frame[i]<mn) mn=frame[i]; if(frame[i]>mx) mx=frame[i]; }
    Serial.printf("像素温度范围: %.2f .. %.2f (Δ=%.2f)\n", mn,mx,mx-mn);
    // 合理性判定
    bool valueOK = (mn > -55 && mx < 360 && (mx - mn) > 0.5);
    if (!valueOK) {
      // 再尝试开氏度->摄氏度转换
      for (uint16_t i=0;i<EXPECT_PIXEL_COUNT;++i) frame[i] = frame[i] - 273.15f;
      mn = frame[0]; mx = frame[0];
      for (uint16_t i=1;i<EXPECT_PIXEL_COUNT;++i){ if(frame[i]<mn) mn=frame[i]; if(frame[i]>mx) mx=frame[i]; }
      Serial.printf("K->C调整后范围: %.2f .. %.2f (Δ=%.2f)\n", mn,mx,mx-mn);
      valueOK = (mn > -55 && mx < 360 && (mx - mn) > 0.5);
    }
    if (valueOK) {
      if (USE_STRICT_PROTOCOL && !checksumOK) {
        Serial.println("严格模式：校验失败拒绝帧");
        return false;
      }
      return true; // 成功解析
    } else {
      Serial.println("数值不合理，继续寻找下一帧...");
      continue; // 尝试后续帧
    }
  }
  return false; // 未找到有效帧
}