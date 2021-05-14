# esp8266/32 炫彩LED氛围灯

- 作者：pengqiangsheng
- 说明：基于太极创客智能LED改进而来，适应 esp8266/32 系列。
- 技术栈：MQTT + FastLed
- 硬件需要：esp8266/esp32开发板 + ws2812灯带 + 5v开关电源
- 注意事项：供电一定要足够, 灯带需要单独供电。电量计算：一颗灯珠0.3w，30颗灯珠需要 30 * 0.3w = 9w，因此带动30颗灯珠的灯带需要一个5v 2A的开关电源。
- 推荐网站：太极创客 http://www.taichi-maker.com/
- 创建时间：05/10/2021

# 介绍

 - esp8266/32智能LED是一款可用APP/PC/Web页面等多种方式通过MQTT协议进行无线控制的智能光带。
 - 光带的颜色，亮度，开关，动态色彩，工作模式调整等均可以使用手机应用通过WIFI进行无线遥控。
 - 光带配有多种工作模式，可以静态单色点亮您所喜爱的色彩，也可以通过动态彩色的模式为您的生活增添欢乐气氛！

# 指令说明：
字符  说明                               参数范围                 说明 
---  -----------                        ---------                -----------------
- a    点亮所有LED为统一颜色色调           0-255                    设置色调
- b    设置亮度                           0-255
- c    关闭光带               
- d    设置延迟参数                       0-255                    常用数值10
- e    上调/下调动态模式                   0/1                      上调 = 0, 下调 = 1.
- f    设置色盘                           0-255                    只对部分动态特效有效果
- g    光点闪烁                                                    打开/关闭光点闪烁
- h    显示可用指令列表 
- i    色盘色调设置                       0-255                    将色盘色彩设置为靠近色调数值的颜色
- l    设置光带灯珠数量并存储于EEPROM      1-255
- m    设置显示模式                       0-255                    设置动态色彩模式：实际只有38种：1-38, 详见strobe_mode函数
- n    设置光亮方向                                                有些动态模式如 Matrix 和 one_sin可以设置光亮方向
- p    工作模式                           0-2                      0:固定模式 1：顺序模式 2：随机模式
- q    获取版本号                         
- r    报告当前系统主要参数状态                        
- t    设置色盘模式                        0-3                     调整色盘模式  0=固定, 1=相似, 2=随机              
- u    每种动态色彩播放时间                1-255                    1秒 - 255秒
- w    将当前动态模式写入EEPROM           
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