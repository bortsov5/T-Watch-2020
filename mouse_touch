#include <Wire.h>
#include <axp20x.h>
#include <FT6236.h>
#include <BleMouse.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

#define TFT_CS    5  // define chip select pin
#define TFT_DC    27  // define data/command pin
#define TFT_RST   -1
#define TFT_MOSI 19  // Data out
#define TFT_SCLK 18  // Clock out

#define TWATCH_DAC_IIS_BCK          26
#define TWATCH_DAC_IIS_WS           25
#define TWATCH_DAC_IIS_DOUT         33

uint8_t data[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t datar[1] = {0x04};
uint8_t dataw[1] = {0x17};
uint8_t datam[1] = {0x03};
byte trol = 0;
byte touch = 0;
int val = 0;

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
AXP20X_Class axp;
FT6236 ts = FT6236();
BleMouse bleMouse;

const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;

uint16_t writeRegister(uint8_t address, uint8_t reg, uint8_t *data, uint16_t len)
{
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  Wire1.write(data, len);
  return (0 !=  Wire1.endTransmission());
}


uint16_t readRegister(uint8_t address, uint8_t reg, uint8_t *data, uint16_t len)
{
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  Wire1.endTransmission();
  Wire1.requestFrom((uint8_t)address, (uint8_t)len);
  uint8_t i = 0;
  while (Wire1.available()) {
    data[i++] = (byte)Wire1.read();
  }
  return 0; //Pass
}

void tone(uint8_t pin, unsigned int frequency, unsigned long duration, uint8_t channel)
{
    if (ledcRead(channel)) {
        log_e("Tone channel %d is already in use", ledcRead(channel));
        return;
    }
    ledcAttachPin(pin, channel);
    ledcWriteTone(channel, frequency);
    if (duration) {
        delay(duration);
        noTone(pin, channel);
    }    
}

void noTone(uint8_t pin, uint8_t channel)
{
    ledcDetachPin(pin);
    ledcWrite(channel, 0);
}

void setup()
{
  Serial.begin(115200);
  ts.begin(2, 23, 32);
  Wire1.begin(i2c_sda, i2c_scl);
  int ret = axp.begin(Wire1);
  ret = axp.setPowerOutPut(AXP202_LDO2, AXP202_ON);
  //----
   axp.enableIRQ(AXP202_ALL_IRQ, AXP202_OFF);
    axp.adc1Enable(0xFF, AXP202_OFF);
    axp.adc2Enable(0xFF, AXP202_OFF);
    axp.adc1Enable(AXP202_BATT_VOL_ADC1 | AXP202_BATT_CUR_ADC1 | AXP202_VBUS_VOL_ADC1 | AXP202_VBUS_CUR_ADC1, AXP202_ON);
    axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_CHARGING_FINISHED_IRQ, AXP202_ON);
    axp.clearIRQ();

  //----

  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH); //screen ON

  tft.init(240, 240, SPI_MODE2);  
  tft.fillScreen(ST77XX_BLUE);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(50, 20);
  tft.println("Mouse!");

  bleMouse.begin();

}

void loop()
{

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

  if (ts.touched())
    {
        // Retrieve a point
        TS_Point p = ts.getPoint();
        tft.setCursor(50, 70);
        tft.fillRect(50, 70, 40, 20, ST77XX_BLUE);
        tft.print(p.x);
        tft.setCursor(50, 90);
        tft.fillRect(50, 90, 40, 20, ST77XX_BLUE);
        tft.print(p.y);

         if(bleMouse.isConnected()) {
         

          if (((p.x>90)&&(p.x<120))&&((p.y>90)&&(p.y<120))) {
             bleMouse.click(MOUSE_LEFT);
              } else {
               bleMouse.move(-p.x,-p.y);
              }

          
        }

    }
  
  delay(10);

}
