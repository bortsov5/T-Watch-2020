#include <Wire.h>
#include <axp20x.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

#define TFT_CS    5  // define chip select pin
#define TFT_DC    27  // define data/command pin
#define TFT_RST   -1
#define TFT_MOSI 19  // Data out
#define TFT_SCLK 18  // Clock out

uint8_t data[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t datar[1] = {0x04};
uint8_t dataw[1] = {0x17};
uint8_t datam[1] = {0x03};
byte trol = 0;

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

AXP20X_Class axp;

const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;

uint16_t writeRegister(uint8_t address, uint8_t reg, uint8_t *data, uint16_t len)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data, len);
  return (0 !=  Wire.endTransmission());
}


uint16_t readRegister(uint8_t address, uint8_t reg, uint8_t *data, uint16_t len)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)address, (uint8_t)len);
  uint8_t i = 0;
  while (Wire.available()) {
    data[i++] = (byte)Wire.read();
  }
  return 0; //Pass
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(i2c_sda, i2c_scl);
  int ret = axp.begin(Wire);
  ret = axp.setPowerOutPut(AXP202_LDO2, AXP202_ON);

  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH); //screen ON

  tft.init(240, 240, SPI_MODE2);  
  tft.fillScreen(ST77XX_BLUE);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(50, 20);
  tft.println("Hello World!");

 
}

void loop()
{
  // float temp = axp.getTemp();
  // Serial.print(temp);
  // Serial.println("*C");
 
 //BMA423
  writeRegister(0x19, 0x7D, datar, 1); 
  writeRegister(0x19, 0x40, dataw, 1); 
  writeRegister(0x19, 0x7C, datam, 1); 
  readRegister(0x19, 0x12, data, 6);
  trol = data[1];

  //Display
  digitalWrite(12, HIGH); //screen ON
  tft.setCursor(50, 40);
  tft.fillRect(50, 40, 40, 20, ST77XX_BLUE);
  tft.print(trol);
  
  //delay(10000);
  //digitalWrite(12, LOW); //screen OFF
  delay(1000);

}
