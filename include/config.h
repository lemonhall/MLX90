// MLX90640 红外摄像头项目配置文件

#ifndef CONFIG_H
#define CONFIG_H

// MLX90640 I2C地址 (通常是0x33)
#define MLX90640_I2C_ADDR 0x33

// 温度显示单位
#define TEMP_UNIT_CELSIUS 1
#define TEMP_UNIT_FAHRENHEIT 0

// 显示配置
#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240

// 热力图配置
#define HEATMAP_PIXEL_SIZE 8
#define HEATMAP_OFFSET_X 16
#define HEATMAP_OFFSET_Y 20

// 调试选项
#define DEBUG_SERIAL_OUTPUT 1
#define DEBUG_FULL_MATRIX 0  // 设置为1输出完整温度矩阵

#endif