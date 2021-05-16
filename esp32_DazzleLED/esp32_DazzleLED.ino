
/*
 * 名称：esp8266/32炫彩LED氛围灯
 * 作者：pengqiangsheng
 * 说明：基于太极创客智能LED改进而来，适应 esp8266/32 系列。
 * 技术栈：MQTT + FastLed
 * 硬件需要：esp8266/esp32开发板 + ws2812灯带 + 5v开关电源
 * 注意事项：供电一定要足够, 灯带需要单独供电。电量计算：一颗灯珠0.3w，30颗灯珠需要 30 * 0.3w = 9w，因此带动30颗灯珠的灯带需要一个5v 2A的开关电源。
 * 推荐网站：太极创客 http://www.taichi-maker.com/
 * 05/10/2021
 * 
 * 
 * esp8266/32智能LED是一款可用APP/PC/Web页面等多种方式通过MQTT协议进行无线控制的智能光带。
 * 光带的颜色，亮度，开关，动态色彩，工作模式调整等均可以使用手机应用通过WIFI进行无线遥控。
 * 光带配有多种工作模式，可以静态单色点亮您所喜爱的色彩，也可以通过动态彩色的模式为您的生活增添欢乐气氛！
 * 
 *  
 * 指令说明：
 * 字符  说明                               参数范围                 说明 
 * ---  -----------                        ---------                -----------------
 * a    点亮所有LED为统一颜色色调           0-255                    设置色调
 * b    设置亮度                           0-255
 * c    关闭光带               
 * d    设置延迟参数                       0-255                    常用数值10
 * e    上调/下调动态模式                   0/1                      上调 = 0, 下调 = 1.
 * f    设置色盘                           0-255                    只对部分动态特效有效果
 * g    光点闪烁                                                    打开/关闭光点闪烁
 * h    显示可用指令列表 
 * i    色盘色调设置                       0-255                    将色盘色彩设置为靠近色调数值的颜色
 * l    设置光带灯珠数量并存储于EEPROM      1-255
 * m    设置显示模式                       0-255                    设置动态色彩模式：实际只有38种：1-38, 详见strobe_mode函数
 * n    设置光亮方向                                                有些动态模式如 Matrix 和 one_sin可以设置光亮方向
 * p    工作模式                           0-2                      0:固定模式 1：顺序模式 2：随机模式
 * q    获取版本号                         
 * r    报告当前系统主要参数状态                        
 * t    设置色盘模式                        0-3                     调整色盘模式  0=固定, 1=相似, 2=随机              
 * u    每种动态色彩播放时间                1-255                    1秒 - 255秒
 * w    将当前动态模式写入EEPROM           
 * 
 * 指令示例：
 * m 5  - 显示第5种动态效果（此程序定义了多种动态效果可固定显示也可以顺序轮流显示）
 * a 80 - 将所有光带设置为相同颜色并且显示， 色调为80。（饱和度为255，亮度有用户自定义亮度决定。）
 * p 1 -  使用第1种工作模式。智能光带设有3种工作模式。
 *        模式1：固定模式，即固定显示用户设定的LED颜色或者动态效果。
 *        模式2：顺序播放模式，在这种工作模式下LED将顺序播放动态效果。
 *        模式3：随机播放模式，在这种工作模式下LED将随机播放动态效果。 
 * 开始使用：
 * 1.输入 l 32  (指令含义：l 32 == 初始化灯珠数量为32颗)
 * 2.输入 m 34  (指令含义：m 34 == 显示彩虹动态效果)
 * 3.如果运行正常的话, 您的ws2812灯带将会以彩虹的色彩动态的呈现在您的眼前。
 */
#include "FastLED.h"                                          
#include "EEPROM.h"                                        

// fastled 版本要求
#if FASTLED_VERSION < 3001000
#error "Requires FastLED 3.1 or later; check github for latest code."
#endif

#define SERIAL_BAUDRATE 9600                                 
#define SERIAL_TIMEOUT 5

#define qsubd(x, b)  ((x>b)?wavebright:0)                     
#define qsuba(x, b)  ((x>b)?x-b:0)                            

#define SEIRLIGHT_VERSION 103                // esp8266/32智能LED版本号
byte inbyte;                                 // 当前模式                
int thisarg;                                 // 当前指令             

// 定义ws2812配置
#define LED_DT 23                            // 控制引脚                                          
#define COLOR_ORDER GRB                      // 以GRB排列               
#define LED_TYPE WS2812                      // 灯条类型（具体支持的类型请查看FastLed库源码）              
#define MAX_LEDS 100                         // 灯珠数量上限                

uint8_t NUM_LEDS;                                             
uint8_t max_bright = 255;                                    

struct CRGB leds[MAX_LEDS];                                   

CRGBPalette16 currentPalette;                                
CRGBPalette16 targetPalette;                                  
TBlendType    currentBlending;                              

// 定义写入EEPROM的地址，用户可操作地址0 ~ 4095
#define STARTMODE 0
#define STRANDLEN 1
#define GLITTERMODE 2
#define DEMORUNMODE 3
#define DEMOTIMELEN 4

uint8_t ledMode;                                             
uint8_t demorun = 0;                                          
uint8_t maxMode = 38;                                        
uint8_t demotime;                                      

// 运行参数  ----------------------------------------------------------------------
uint8_t allfreq = 32;                                         
uint8_t bgclr = 0;                                            // Generic background colour
uint8_t bgbri = 0;                                            // Generic background brightness
bool    glitter = 0;                                          // Glitter flag
uint8_t palchg;                                               // 0=no change, 1=similar, 2=random
uint8_t startindex = 0;
uint8_t thisbeat;                                             // Standard beat
uint8_t thisbright = 0;                                       // Standard brightness
uint8_t thiscutoff = 192;                                     // You can change the cutoff value to display this wave. Lower value = longer wave.
int thisdelay = 0;                                            // Standard delay
uint8_t thisdiff = 1;                                         // Standard palette jump
bool    thisdir = 0;                                          // Standard direction
uint8_t thisfade = 224;                                       // Standard fade rate
uint8_t thishue = 0;                                          // Standard hue
uint8_t thisindex = 0;                                        // Standard palette index
uint8_t thisinc = 1;                                          // Standard incrementer
int     thisphase = 0;                                        // Standard phase change
uint8_t thisrot = 1;                                          // You can change how quickly the hue rotates for this wave. Currently 0.
uint8_t thissat = 255;                                        // Standard saturation
int8_t  thisspeed = 4;                                        // Standard speed change
uint8_t wavebright = 255;                                     // You can change the brightness of the waves/bars rolling across the screen.

uint8_t xd[MAX_LEDS];                                         // arrays for the 2d coordinates of any led
uint8_t yd[MAX_LEDS];

long summ=0;

extern const TProgmemRGBGradientPalettePtr gGradientPalettes[]; // gradient_palettes.h 色板
extern const uint8_t gGradientPaletteCount;                     // 色板数量 
uint8_t gCurrentPaletteNumber = 0;                              // 当前色板编号
uint8_t currentPatternIndex = 0;                                // 当前 pattern 编号 

// Display functions -----------------------------------------------------------------------
#include "addglitter.h"
#include "make_palettes.h"

// Display routines  -----------------------------------------------------------------------
#include "circnoise_pal_1.h"
#include "circnoise_pal_2.h"
#include "circnoise_pal_3.h"
#include "circnoise_pal_4.h"
#include "confetti_pal.h"
#include "gradient_palettes.h"
#include "juggle_pal.h"
#include "matrix_pal.h"
#include "noise16_pal.h"
#include "noise8_pal.h"
#include "one_sin_pal.h"
#include "rainbow_march.h"
#include "serendipitous_pal.h"
#include "three_sin_pal.h"
#include "two_sin.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include "http_web_client.h"
#include <uri/UriRegex.h>

// 定义板载Led灯的GPIO引脚
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// 设置wifi接入信息(请根据您的WiFi信息进行修改)
const char* ssid = "";
const char* password = "";


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // 设置板上LED引脚为输出模式
  digitalWrite(LED_BUILTIN, HIGH);  // 启动后开启板上LED
  Serial.begin(SERIAL_BAUDRATE);
  delay(10);
  // 申请操作5个字节数据
  EEPROM.begin(5);
  
  //设置ESP8266工作模式为无线终端模式
  WiFi.mode(WIFI_STA);
  
  // 连接WiFi
  connectWifi();
  
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", controlIndex);
  });
  server.on(UriRegex("^\\/cmd\\/([a-z]+)\\/arg\\/([0-9]+)$"), handleControl);
  server.begin();

  delay(1000);                                                // 稳定性等待 
  
  LEDS.setBrightness(max_bright);                                                 // 亮度控制 
  LEDS.addLeds<LED_TYPE, LED_DT, COLOR_ORDER >(leds, MAX_LEDS);                   // 光带初始化 
  
  set_max_power_in_volts_and_milliamps(5, 1000);                                  // 电源管理 5V, 1000mA

  random16_set_seed(4832);                                                        // 随机种子
  random16_add_entropy(analogRead(2));                                            // seek += analogRead(2) 
  int ranstart = random16();
  
  ledMode = EEPROM.read(STARTMODE);                                              // EEPROM 0 存储 启动模式
  NUM_LEDS = EEPROM.read(STRANDLEN); if(NUM_LEDS>MAX_LEDS) NUM_LEDS = MAX_LEDS;  // EEPROM 1 存储 灯珠数量 
  glitter = EEPROM.read(GLITTERMODE);                                            // EEPROM 2 存储 闪烁模式 
  demorun = EEPROM.read(DEMORUNMODE);                                            // EEPROM 3 存储 演示模式
  demotime = EEPROM.read(DEMOTIMELEN);  if (demotime == 0) { demotime = 10; Serial.println(F("First Run demotime = 10"));}
                                                                                  // EEPROM 4 存储 动态保持时间 
  currentPalette  = CRGBPalette16(CRGB::Black);
  targetPalette   = RainbowColors_p;
  currentBlending = LINEARBLEND;

  //  Stefan Petrick 的 Circular Noise
  for (uint8_t i = 0; i < NUM_LEDS; i++) {        // precalculate the lookup-tables:
    uint8_t angle = (i * 256) / NUM_LEDS;         // on which position on the circle is the led?
    xd[i] = cos8( angle );                         // corresponding x position in the matrix
    yd[i] = sin8( angle );                         // corresponding y position in the matrix
  }
  // 欢迎内容
  Serial.println(F("Welcome to Esp8266/32 LED. Type h for Help."));
  Serial.println(F("For more info. please visit Github:"));
  Serial.println(F("https://github.com/pengqiangsheng")); 
  // 启动灯带显示模式
  strobe_mode(ledMode, 1);                                                      
}
 
void loop() { 
  server.handleClient();

  demo_check();                                                             // demo模式下检查变更动态模式时间 

  EVERY_N_MILLISECONDS(50) {                                                 // 色板平滑过渡
    uint8_t maxChanges = 24; 
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);   
  }

  EVERY_N_SECONDS(5) {                                                        // 随机色板控制 
    if (palchg == 1) SetupSimilar4Palette();
    if (palchg == 2) SetupRandom4Palette();
    if (palchg == 3) SetupRandom16Palette();
  }

  EVERY_N_MILLIS_I(thistimer, thisdelay) {                                    // delay 时间 .
    thistimer.setPeriod(thisdelay);                                           
    strobe_mode(ledMode, 0);                                                  
  }

  if(glitter) addglitter(10);                                                 // 闪烁控制 
  
  FastLED.show(); 
}

// 字节流转整形
int byte2int(byte* bytes, unsigned int length)
{
  int temp[length];
  int num = 0;
  for (unsigned int i = 0; i < length; i++)
  {
    temp[i] = bytes[i] & 0xFF;
    if(!(temp[i] >= 48 && temp[i] <= 57)) continue;
    num += (temp[i] - 48);
    num *= 10;
  }
  num /= 10;
  return num;
}
 
// ESP8266连接wifi
void connectWifi(){
 
  WiFi.begin(ssid, password);
 
  //等待WiFi连接,成功连接后输出成功信息
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Connected!");  
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

}


void strobe_mode(uint8_t newMode, bool mc) {

  if(mc) {
    fill_solid(leds,NUM_LEDS,CRGB(0,0,0));                    // 确保光滑过渡 
    Serial.print(F("Mode: ")); 
    Serial.print(ledMode);
    Serial.print(F(" ")); 
  }

  switch (newMode)
  {
    case  0: if(mc) {fill_solid(leds,NUM_LEDS,CRGB(0,0,0)); Serial.println(F("All Off"));} break;                     // All off, not animated.
    case  1: if(mc) {fill_solid(leds, NUM_LEDS,CRGB(255,255,255));Serial.println(F("All On"));} break;              // All on, not animated.
    case  2: if(mc) {thisdelay=10; allfreq=2; thisspeed=1; thatspeed=1; thishue=0; thathue=128; thisdir=0; thisrot=1; thatrot=1; thiscutoff=128; thatcutoff=192; Serial.println(F("Two Sine-1"));} two_sin(); break;
    case  3: if(mc) {thisdelay=20; targetPalette=RainbowColors_p; allfreq=4; bgclr=0; bgbri=0; thisbright=255; startindex=64; thisinc=2; thiscutoff=224; thisphase=0; thiscutoff=224; thisrot=0; thisspeed=4; wavebright=255; Serial.println(F("One Sine-1"));} one_sin_pal(); break;
    case  4: if(mc) {thisdelay=10; targetPalette = PartyColors_p; palchg=2;Serial.println(F("Noise8-1"));} noise8_pal(); break;
    case  5: if(mc) {thisdelay=10; allfreq=4; thisspeed=-1; thatspeed=0; thishue=64; thathue=192; thisdir=0; thisrot=0; thatrot=0; thiscutoff=64; thatcutoff=192;Serial.println(F("Two Sine-2"));} two_sin(); break;
    case  6: if(mc) {thisdelay=20; targetPalette=RainbowColors_p; allfreq=10; bgclr=64; bgbri=4; thisbright=255; startindex=64; thisinc=2; thiscutoff=224; thisphase=0; thiscutoff=224; thisrot=0; thisspeed=4; wavebright=255;Serial.println(F("One Sine-2"));} one_sin_pal(); break;
    case  7: if(mc) {thisdelay=10; numdots=2; targetPalette=PartyColors_p; thisfade=16; thisbeat=8; thisbright=255; thisdiff=64;Serial.println(F("Juggle-1"));} juggle_pal(); break;
    case  8: if(mc) {thisdelay=40; targetPalette = LavaColors_p; thisindex=128; thisdir=1; thisrot=0; thisbright=255; bgclr=200; bgbri=6;Serial.println(F("Matrix Palette"));} matrix_pal(); break;
    case  9: if(mc) {thisdelay=10; allfreq=6; thisspeed=2; thatspeed=3; thishue=96; thathue=224; thisdir=1; thisrot=0; thatrot=0; thiscutoff=64; thatcutoff=64;Serial.println(F("Two Sine-2"));} two_sin(); break;
    case 10: if(mc) {thisdelay=20; targetPalette=RainbowColors_p; allfreq=16; bgclr=0; bgbri=0; thisbright=255; startindex=64; thisinc=2; thiscutoff=224; thisphase=0; thiscutoff=224; thisrot=0; thisspeed=4; wavebright=255;Serial.println(F("One Sine-3"));} one_sin_pal(); break;
    case 11: if(mc) {thisdelay=50; mul1=5; mul2=8; mul3=7;Serial.println(F("Three Sine-1"));} three_sin_pal(); break;
    case 12: if(mc) {thisdelay=10; targetPalette=ForestColors_p;Serial.println(F("Serendipitous"));} serendipitous_pal(); break;
    case 13: if(mc) {thisdelay=20; targetPalette=LavaColors_p; allfreq=8; bgclr=0; bgbri=4; thisbright=255; startindex=64; thisinc=2; thiscutoff=224; thisphase=0; thiscutoff=224; thisrot=0; thisspeed=4; wavebright=255;Serial.println(F("One Sine-4"));} one_sin_pal(); break;
    case 14: if(mc) {thisdelay=10; allfreq=20; thisspeed=2; thatspeed=-1; thishue=24; thathue=180; thisdir=1; thisrot=0; thatrot=1; thiscutoff=64; thatcutoff=128;Serial.println(F("Two Sine-3"));} two_sin(); break;
    case 15: if(mc) {thisdelay=50; targetPalette = PartyColors_p; thisindex=64; thisdir=0; thisrot=1; thisbright=255; bgclr=100; bgbri=10;Serial.println(F("Matrix-1"));} matrix_pal(); break;
    case 16: if(mc) {thisdelay=10; targetPalette = OceanColors_p; palchg=1;Serial.println(F("Noise8-2"));} noise8_pal(); break;
    case 17: if(mc) {thisdelay=10; targetPalette=PartyColors_p;Serial.println(F("Circ-Noise-1"));} circnoise_pal_2(); break;
    case 18: if(mc) {thisdelay=20; allfreq=10; thisspeed=1; thatspeed=-2; thishue=48; thathue=160; thisdir=0; thisrot=1; thatrot=-1; thiscutoff=128; thatcutoff=192;Serial.println(F("Two Sine-4"));} two_sin(); break;
    case 19: if(mc) {thisdelay=50; mul1=6; mul2=9; mul3=11;Serial.println(F("Three Sine-2"));} three_sin_pal(); break;
    case 20: if(mc) {thisdelay=10; thisdir=1; thisrot=1; thisdiff=1;Serial.println(F("Rainbow March-1"));} rainbow_march(); break;
    case 21: if(mc) {thisdelay=10; thisdir=1; thisrot=2; thisdiff=10;Serial.println(F("Rainbow March-2"));} rainbow_march(); break;
    case 22: if(mc) {thisdelay=20; hxyinc = random16(1,15); octaves=random16(1,3); hue_octaves=random16(1,5); hue_scale=random16(10, 50);  x=random16(); xscale=random16(); hxy= random16(); hue_time=random16(); hue_speed=random16(1,3); x_speed=random16(1,30);Serial.println(F("Noise16-1"));} noise16_pal(); break;
    case 23: if(mc) {thisdelay=20; targetPalette=OceanColors_p; allfreq=6; bgclr=0; bgbri=0; thisbright=255; startindex=64; thisinc=2; thiscutoff=224; thisphase=0; thiscutoff=224; thisrot=0; thisspeed=4; wavebright=255;Serial.println(F("One Sine-5"));} one_sin_pal(); break;
    case 24: if(mc) {thisdelay=10; targetPalette=OceanColors_p;Serial.println(F("Circ-Noise-2"));} circnoise_pal_4(); break;
    case 25: if(mc) {thisdelay=20; targetPalette = PartyColors_p; thisinc=1; thishue=192; thissat=255; thisfade=2; thisdiff=32; thisbright=255;Serial.println(F("Confetti-1"));} confetti_pal(); break;
    case 26: if(mc) {thisdelay=10; thisspeed=2; thatspeed=3; thishue=96; thathue=224; thisdir=1; thisrot=1; thatrot=2; thiscutoff=128; thatcutoff=64;Serial.println(F("Two Sine-5"));} two_sin(); break;
    case 27: if(mc) {thisdelay=30; targetPalette = ForestColors_p; thisindex=192; thisdir=0; thisrot=0; thisbright=255; bgclr=50; bgbri=0;Serial.println(F("Matrix-2"));} matrix_pal(); break;
    case 28: if(mc) {thisdelay=20; targetPalette=RainbowColors_p; allfreq=20; bgclr=0; bgbri=0; thisbright=255; startindex=64; thisinc=2; thiscutoff=224; thisphase=0; thiscutoff=224; thisrot=0; thisspeed=4; wavebright=255;Serial.println(F("One Sine-6"));} one_sin_pal(); break;
    case 29: if(mc) {thisdelay=20; targetPalette = LavaColors_p; thisinc=2; thishue=128; thisfade=8; thisdiff=64; thisbright=255;Serial.println(F("Confetti-2"));} confetti_pal(); break;
    case 30: if(mc) {thisdelay=10; targetPalette=PartyColors_p;Serial.println(F("Circ-Noise-3"));} circnoise_pal_3(); break;
    case 31: if(mc) {thisdelay=10; numdots=4; targetPalette=OceanColors_p; thisfade=32; thisbeat=12; thisbright=255; thisdiff=20;Serial.println(F("Juggle-2"));} juggle_pal(); break;
    case 32: if(mc) {thisdelay=30; SetupSimilar4Palette(); allfreq=4; bgclr=64; bgbri=4; thisbright=255; startindex=64; thisinc=2; thiscutoff=224; thisphase=0; thiscutoff=128; thisrot=1; thisspeed=8; wavebright=255;Serial.println(F("One Sine-7"));} one_sin_pal(); break;
    case 33: if(mc) {thisdelay=50; mul1=3; mul2=4; mul3=5;Serial.println(F("Three Sine-3"));} three_sin_pal(); break;
    case 34: if(mc) {thisdelay=10; thisdir=-1; thisrot=1; thisdiff=5;Serial.println(F("Rainbow March-3"));} rainbow_march(); break;
    case 35: if(mc) {thisdelay=10; targetPalette=PartyColors_p;Serial.println(F("Circ-Noise-4"));} circnoise_pal_1(); break;
    case 36: if(mc) {thisdelay=20; targetPalette = ForestColors_p; thisinc=1; thishue=random8(255); thisfade=1; thisbright=255;Serial.println(F("Confetti-3"));} confetti_pal(); break;
    case 37: if(mc) {thisdelay=20; octaves=1; hue_octaves=2; hxy=6000; x=5000; xscale=3000; hue_scale=50; hue_speed=15; x_speed=100;Serial.println(F("Noise16-2"));} noise16_pal(); break;
    case 38: if(mc) {thisdelay=10; targetPalette = LavaColors_p; palchg=0;Serial.println(F("Noise8-3"));} noise8_pal(); break;
  }
}

void demo_check(){
  
  if(demorun) {                                                   // 检查是否为demoI, 如果是这轮流显示各个模式 
    uint8_t secondHand = (millis() / 1000) % (maxMode*demotime);        // 根据模式数量动态调节循环时间  
    static uint8_t lastSecond = 99;                               
    if (lastSecond != secondHand) {                               
      lastSecond = secondHand;
        if(secondHand%demotime==0) {                                    
          if(demorun == 2) ledMode = random8(0,maxMode); else {
            ledMode = secondHand/demotime;
          }
          strobe_mode(ledMode,1);                            
      } 
    }
  }
}

// 解析cmd数据 
String handleCommand(char inbyte, int inputNum) {
  String logCmdStr = "";
  if (inbyte != 10) {                                       
    Serial.print("# Received Command:  ");
    Serial.print(char(inbyte));
    Serial.print(" ");
  }
  
  Serial.print("接收到的命令：");
  Serial.println(inputNum);
  switch(inbyte) {

    case 97:                                                // "a" - 设置色调 hue = 0 - 255
      demorun = 0;
      ledMode = 0;
      thisarg = inputNum;
      thisarg = constrain(thisarg,0,255);
      Serial.println(thisarg);
      fill_solid(leds, NUM_LEDS, CHSV(thisarg, 255, 255));
      Serial.print(F("# Set All LED to Hue ")); Serial.println(thisarg);
      logCmdStr = "# Set All LED to Hue " + String(thisarg);
      break;

    case 98:                                                // "b" - SET MAX BRIGHTNESS to #
      max_bright = inputNum;
      max_bright = constrain(max_bright,0,255);        
      Serial.println(max_bright);
      LEDS.setBrightness(max_bright);
      Serial.print(F("# Set Brightness to ")); Serial.println(max_bright);
      logCmdStr = "# Set Brightness to " + String(max_bright);
      break;

    case 99:                                                // "c" - 关闭光带
      Serial.println(F(" "));
      demorun = 0;
      ledMode = 0;
      strobe_mode(ledMode, 1);
      logCmdStr = "# close ws2812 all " + String(max_bright);
      break;

    case 100:                                               // "d" - 设置延迟参数 
      thisarg = inputNum;
      thisdelay = constrain(thisarg,0,255);
      Serial.println(thisdelay);
      Serial.print(F("# Set Delay to ")); Serial.println(thisdelay);  
      logCmdStr = "# Set Delay to " + String(thisdelay);      
      break;

    case 101:                                              // "e" - 上调/下调动态模式      

      thisarg = inputNum;
      Serial.println(thisarg);
      if (thisarg) {
        demorun = 0; ledMode=(ledMode+1)%(maxMode+1); Serial.println(F("# Play Next Mode"));
        logCmdStr = "# Play Next Mode "; 
      } else {
        demorun = 0; ledMode=(ledMode-1); if (ledMode==255) ledMode=maxMode; Serial.println(F("# Play Previous Mode"));
        logCmdStr = "# Play Previous Mode "; 
      }
      strobe_mode(ledMode,1);
      
      break;

    case 102:                                               // "f" - 设置色盘
      demorun = 0;
      palchg = 0;
      thisarg = inputNum;
      gCurrentPaletteNumber = thisarg % gGradientPaletteCount;
      targetPalette = gGradientPalettes[gCurrentPaletteNumber];
      Serial.println(thisarg);
      Serial.print(F("# Set Palette to ")); Serial.println(thisarg);  
      logCmdStr = "# Set Palette to " + String(thisarg);    
      break;

    case 103:                                               // "g" - 打开/关闭光点闪烁
      glitter = !glitter;
      Serial.println(F(""));
      if (glitter) {Serial.println(F("# Glitter ON"));logCmdStr = "# Glitter ON";} else {Serial.println(F("# Glitter OFF"));logCmdStr = "# Glitter OFF";}
      break;
        
    case 104:                                               // "h" - 指令列表 
      Serial.println(F(""));
      Serial.println(F(" ---------- Command List ---------- "));
      Serial.println(F(" * Key  Description"));
      Serial.println(F(" * a    Set all to one colour by hue (0-255)"));
      Serial.println(F(" * b    Set brightness (0-255)"));
      Serial.println(F(" * c    clear strip (set mode 0)"));
      Serial.println(F(" * d    Set delay variable (0-255)"));
      Serial.println(F(" * e    Set display mode previous/next. Previous = 0, Next = 1."));
      Serial.println(F(" * f    Set fixed palette mode (0 to 255)"));
      Serial.println(F(" * g    Glitter toggle"));
      Serial.println(F(" * i    Similar palette hue (0-255)"));
      Serial.println(F(" * l    Set strip length & write EEPROM (0-255)"));
      Serial.println(F(" * m    Set display mode  (0-255)"));
      Serial.println(F(" * n    Direction toggle "));
      Serial.println(F(" * p    Play mode ( 0-2 fix, seq, shuf)"));
      Serial.println(F(" * q    Return version number"));
      Serial.println(F(" * t    Select palette mode (0-3)"));
      Serial.println(F(" * u    Set sequence duration (0-255)"));
      Serial.println(F(" * w    Write current mode to EEPROM"));
      logCmdStr =  " ---------- Command List ---------- \n";
      logCmdStr +=  " * Key  Description \n";
      logCmdStr +=  " * a    Set all to one colour by hue (0-255) \n";
      logCmdStr +=  " * b    Set brightness (0-255) \n";
      logCmdStr +=  " * c    clear strip (set mode 0) \n";
      logCmdStr +=  " * d    Set delay variable (0-255) \n";
      logCmdStr +=  " * e    Set display mode previous/next. Previous = 0, Next = 1. \n";
      logCmdStr +=  " * f    Set fixed palette mode (0 to 255) \n";
      logCmdStr +=  " * g    Glitter toggle \n";
      logCmdStr +=  " * i    Similar palette hue (0-255) \n";
      logCmdStr +=  " * l    Set strip length & write EEPROM (0-255) \n";
      logCmdStr +=  " * m    Set display mode  (0-255) \n";
      logCmdStr +=  " * n    Direction toggle \n";
      logCmdStr +=  " * p    Play mode ( 0-2 fix, seq, shuf) \n";
      logCmdStr +=  " * q    Return version number \n";
      logCmdStr +=  " * t    Select palette mode (0-3) \n";
      logCmdStr +=  " * u    Set sequence duration (0-255) \n";
      logCmdStr +=  " * w    Write current mode to EEPROM \n";
      break;     
         
    case 105:                                               // "i" - 将色盘色彩设置为靠近色调数值的颜色
      palchg = 0;
      thisarg = inputNum;
      thishue = constrain(thisarg,0,255);
      Serial.println(thishue);
      SetupMySimilar4Palette();
      Serial.print(F("# Set Hue to ")); Serial.println(thishue); 
      logCmdStr = "# Set Hue to " + String(thishue);    
      break;

    case 108:                                               // "l" - 设置光带灯珠数量并存储于EEPROM
      thisarg = inputNum;
      NUM_LEDS = constrain(thisarg,1,MAX_LEDS);
      Serial.println(NUM_LEDS);
      EEPROM.write(STRANDLEN, NUM_LEDS);
      EEPROM.commit();
      delay(10);
      Serial.print(F("# Set LED Number to ")); Serial.println(NUM_LEDS); 
      logCmdStr = "# Set LED Number to " + String(NUM_LEDS);  
      Serial.println(F("# LED info Saved to EEPROM."));
      break;

    case 109:                                               // "m" - 设置显示模式 to #
      demorun = 0;
      ledMode = inputNum;
      ledMode = constrain(ledMode,0,maxMode);
      Serial.println(ledMode);
      strobe_mode(ledMode, 1);
      break;

    case 110:                                               // "n"  - 设置光亮方向
      Serial.println(F(" "));
      thisdir = !thisdir;
      Serial.println(F("# Set Direction"));
      logCmdStr = "# Set Direction";
      break;

    case 112:                                               // "p" - Play mode is either fixed, sequential or shuffle
      demorun = inputNum;
      demorun = constrain(demorun,0,2);
      Serial.println(demorun); 
      if (demorun == 0) {
        Serial.println(F("# Play Mode: Fixed"));
        logCmdStr = "# Play Mode: Fixed";
      } else if(demorun == 1) {
        Serial.println(F("# Play Mode: Sequential"));
        logCmdStr = "# Play Mode: Sequential";
      } else if(demorun == 2) {
        Serial.println(F("# Play Mode: Shuffle"));
        logCmdStr = "# Play Mode: Shuffle";
      } 
      EEPROM.write(DEMORUNMODE,demorun); 
      EEPROM.commit();
      delay(10);             
      break;      
    
    case 113:                                               // "q" - 版本号
      Serial.println(SEIRLIGHT_VERSION);
      logCmdStr = "# version number is " + String(SEIRLIGHT_VERSION);
      break;

    case 114:                                               // "r" - 输出当前状态 
      Serial.println(F("")); 
      Serial.println(F("--- Report System Status ---"));  
      Serial.print(F("LED Number:")); Serial.println(NUM_LEDS); 
      Serial.print(F("LED Mode No:")); Serial.println(ledMode);          
      Serial.print(F("Work Mode:")); Serial.println(demorun);            
      Serial.print(F("Demo Time:")); Serial.println(demotime);
      Serial.print(F("Glitter Setting:")); Serial.println(glitter);    
      Serial.print(F("Brightness:")); Serial.println(max_bright);  
      logCmdStr =  "--- Report System Status ---\n";
      logCmdStr +=  "LED Number:";
      logCmdStr += String(NUM_LEDS);
      logCmdStr += "\n";
      logCmdStr +=  "LED Mode No:";
      logCmdStr += String(ledMode);
      logCmdStr += "\n";
      logCmdStr +=  "Work Mode:";
      logCmdStr += String(demorun);
      logCmdStr += "\n";
      logCmdStr +=  "Demo Time:";
      logCmdStr += String(demotime);
      logCmdStr += "\n";
      logCmdStr +=  "Glitter Setting:";
      logCmdStr += String(glitter);
      logCmdStr += "\n";
      logCmdStr +=  "Brightness:";
      logCmdStr += String(max_bright);
      logCmdStr += "\n";
      break;
      
    case 116:                                               // "t" - 设置色盘模式
      thisarg = inputNum;
      palchg = constrain(thisarg,0,3);
      Serial.println(palchg);
      logCmdStr = "# set colorPlatte to " + String(palchg);
      break;

    case 117:                                               // "u" - 每种动态色彩播放时间 
      thisarg = inputNum;
      demotime = constrain(thisarg,1,255);
      Serial.println(demotime);
      Serial.print(F("# Set Mode Play Time to ")); Serial.println(demotime); 
      logCmdStr = "# Set Mode Play Time to " + String(demotime);
      EEPROM.write(DEMOTIMELEN,demotime);
      EEPROM.commit();
      delay(10);
      break;

    case 119:                                               // "w" - 将当前动态模式写入EEPROM            
      Serial.println(F(""));
      Serial.print(F("# Save Mode: "));
      Serial.print(ledMode);
      Serial.print(F(" and Current Status to EEPROM..."));
      logCmdStr = "# Save Mode: " + String(ledMode) + " and Current Status to EEPROM...";
      delay(100);
      EEPROM.write(STARTMODE, ledMode);
      EEPROM.write(GLITTERMODE, glitter);
      EEPROM.commit();
      delay(10);
      Serial.println(F("Done!"));
      logCmdStr += "Done!";
      break;   

    default: 
      Serial.println(F(""));
      Serial.println(F("# Unknown Command. Type h for command list."));   
      logCmdStr = "# Unknown Command. Type h for command list.";     
  }
  
  return logCmdStr;
} 

void handleControl() {
   
  String cmdStr = server.pathArg(0);
  String argStr = server.pathArg(1);

  char cmdTemp[2]; 
  strcpy(cmdTemp, cmdStr.c_str());
  int number = atoi(argStr.c_str());
  Serial.println("");
  Serial.println(cmdStr);
  Serial.println(argStr);
  Serial.println("");
  String logStr = handleCommand(cmdTemp[0], number);
  
  server.send(200, "text/plain", logStr);
}
