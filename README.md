# esp8266/32 炫彩LED氛围灯

- 作者：pengqiangsheng
- 说明：基于太极创客智能LED改进而来，适应 esp8266/32 系列。
- 技术栈：MQTT/HTTP + FastLed + WebApp
- 硬件需要：esp8266/esp32开发板 + ws2812灯带 + 5v开关电源
- 注意事项：供电一定要足够, 灯带需要单独供电。电量计算：一颗灯珠0.3w，30颗灯珠需要 30 * 0.3w = 9w，因此带动30颗灯珠的灯带需要一个5v 2A的开关电源。
- 推荐网站：太极创客 http://www.taichi-maker.com/
- 创建时间：05/10/2021

# 版本说明

- `main`分支存放的是基于mqtt的控制界面
- `httpControl`分支存放的是基于http的控制界面

推荐第一次使用下载`httpControl`分支，Tag界面[Tag1.0.2](https://github.com/pengqiangsheng/esp8266_DazzleLed_ws2812/releases/tag/1.0.2)，直接下载链接[点我下载](https://github.com/pengqiangsheng/esp8266_DazzleLed_ws2812/archive/refs/tags/1.0.2.zip)

# 固件包

目前只有esp32的智能配网固件，下载地址：[esp32_DazzleLED.ino.esp32.bin](https://github.com/pengqiangsheng/esp8266_DazzleLed_ws2812/releases/download/1.0.2/esp32_DazzleLED.ino.esp32.bin)

# 介绍

 - esp8266/32智能LED是一款可用APP/PC/Web页面等多种方式通过MQTT/HTTP协议进行无线控制的智能光带。
 - 光带的颜色，亮度，开关，动态色彩，工作模式调整等均可以使用手机应用通过WIFI进行无线遥控。
 - 光带配有多种工作模式，可以静态单色点亮您所喜爱的色彩，也可以通过动态彩色的模式为您的生活增添欢乐气氛！

# 初次使用必看

## 2.1 基于http协议的控制界面

> 克隆`httpControl`分支或者直接下载`Tag1.0.2`, 修改wifi信息直接烧写到设备

在浏览输入串口中打印的`ip地址`

![1](https://cdn.jsdelivr.net/gh/pengqiangsheng/esp8266_DazzleLed_ws2812/img/6.png)
![2](https://cdn.jsdelivr.net/gh/pengqiangsheng/esp8266_DazzleLed_ws2812/img/7.png)
![3](https://cdn.jsdelivr.net/gh/pengqiangsheng/esp8266_DazzleLed_ws2812/img/8.png)
![4](https://cdn.jsdelivr.net/gh/pengqiangsheng/esp8266_DazzleLed_ws2812/img/9.png)


## 2.2 基于mqtt协议的控制界面

`main`分支克隆后需要做三件事：
- 1.定义板子系列（32/8266）
- 2.修改wifi信息和mqtt相关配置
- 3.在arduino IDE中安装`PubSubClient`库

在`esp8266_DazzleLed_ws2812.ino`文件54行先定义板子的系列：
```c++
#include "FastLED.h"                                          
#include "EEPROM.h"                                        
#ifndef BOARDVERSION
#define BOARDVERSION 8266 // 定义板子的系列 32/8266
#endif
```


在`esp8266_DazzleLed_ws2812.ino`文件163行开始：
```c++
// 设置wifi接入信息(请根据您的WiFi信息进行修改)
const char* ssid = "";
const char* password = "";
// MQTT服务器地址
const char* mqttServer = "192.168.1.3";
// Topic前缀
const char* mqttPubPrefix = "esp32-Pub-";
const char* mqttSubPrefix = "esp32-Sub-";
// clientId前缀
const char* mqttClientIdPrefix = "esp32";
// 日志Topic
char* logTopicName = "ws2812log";

// mqttServer 为MQTT服务器地址。
// 可以使用公用MQTT服务器，也可以使用自建的局域网MQTT服务器或者自建的公网MQTT服务器
// 关于搭建MQTT服务器的内容，敬请期待。
// 公用MQTT服务器列表如下：
// http://www.taichi-maker.com/public-mqtt-broker/
```

改好烧写到esp32/8266后需要搭配mqtt使用：

![1](https://cdn.jsdelivr.net/gh/pengqiangsheng/esp8266_DazzleLed_ws2812/img/1.png)
![2](https://cdn.jsdelivr.net/gh/pengqiangsheng/esp8266_DazzleLed_ws2812/img/2.png)
![3](https://cdn.jsdelivr.net/gh/pengqiangsheng/esp8266_DazzleLed_ws2812/img/3.png)
![4](https://cdn.jsdelivr.net/gh/pengqiangsheng/esp8266_DazzleLed_ws2812/img/4.png)
![5](https://cdn.jsdelivr.net/gh/pengqiangsheng/esp8266_DazzleLed_ws2812/img/5.png)

> 关于mqtt web客户端的内容，请移步[esp8266网站快速开发脚手架](https://github.com/pengqiangsheng/esp8266_web_generator)

# 指令说明：
|字符|  说明                               |参数范围 |                说明                                |
|---| ----------------------------------- | ------- |---------------------------------------------------|
| a |    点亮所有LED为统一颜色色调           |0-255|                    设置色调|
| b |    设置亮度                           |0-255|                                                      |
| c |    关闭光带                           |     |                                                      |
| d |    设置延迟参数                       |0-255 |                   常用数值10                          |
| e |    上调/下调动态模式                   |0/1  |                    上调 = 0, 下调 = 1.                |
| f |    设置色盘                           |0-255 |                   只对部分动态特效有效果               |
| g |    光点闪烁                           |      |                        打开/关闭光点闪烁              |
| h |    显示可用指令列表                     |     |                                                     |
| i |    色盘色调设置                       |0-255 |                   将色盘色彩设置为靠近色调数值的颜色    |
| l |    设置光带灯珠数量并存储于EEPROM      |1-255|                                                       |
| m |    设置显示模式                       |0-255 |                   设置动态色彩模式：实际只有38种：1-38, 详见strobe_mode函数|
| n |    设置光亮方向                       |       |                       有些动态模式如 Matrix 和 one_sin可以设置光亮方向|
| p |    工作模式                           | 0-2  |                    0:固定模式 1：顺序模式 2：随机模式  |
| q |    获取版本号                         |     |                                                       |
| r |    报告当前系统主要参数状态            |     |                                                       |
| t |    设置色盘模式                       | 0-3  |                   调整色盘模式  0=固定, 1=相似, 2=随机 |
| u |    每种动态色彩播放时间                | 1-255 |                   1秒 - 255秒                       |
| w |    将当前动态模式写入EEPROM            |       |                                                    |
* 
# 指令示例：
- m 5  - 显示第5种动态效果（此程序定义了多种动态效果可固定显示也可以顺序轮流显示）
- a 80 - 将所有光带设置为相同颜色并且显示， 色调为80。（饱和度为255，亮度有用户自定义亮度决定。）
- p 1 -  使用第1种工作模式。智能光带设有3种工作模式。
*        模式1：固定模式，即固定显示用户设定的LED颜色或者动态效果。
*        模式2：顺序播放模式，在这种工作模式下LED将顺序播放动态效果。
*        模式3：随机播放模式，在这种工作模式下LED将随机播放动态效果。 
# 开始使用：
- 1.输入 l 32  (指令含义：l 32 == 初始化灯珠数量为32颗)
- 2.输入 m 34  (指令含义：m 34 == 显示彩虹动态效果)
- 3.如果运行正常的话, 您的ws2812灯带将会以彩虹的色彩动态的呈现在您的眼前。

# 版权说明

本项目是基于太极创客智能LED改进而来，适应 esp8266/32 系列，仅供学习交流使用。如果有任何问题请留下issue。