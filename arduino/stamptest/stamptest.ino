
#if defined(ARDUINO_AVR_ITSYBITSY32U4_3V)
# define BOARDNAME "Itsy Bitsy 3.3V"
#elif defined(ARDUINO_AVR_MICRO)
# define BOARDNAME "Arduino Pro Micro"
#elif defined(ARDUINO_TRINKET_M0)
# define BOARDNAME "Trinket M0"
#elif defined(ARDUINO_ESP32C3_DEV)
# define BOARDNAME "ESP32C3"
# define I2CSDA 18
# define I2CSCL 19
#elif defined(ESP_PLATFORM)
# define BOARDNAME "STAMP-C3"
# define I2CSDA 0
# define I2CSCL 1
#else
# error "Unknown platform."
#endif

#if defined(ARDUINO_ESP32C3_DEV)
# define OLED_CS0    10
# define OLED_CS1     3
# define OLED_CS2     2
# define OLED_CS3     2
# define OLED_DC      1
# define OLED_RESET  -1
#elif defined(ARDUINO_AVR_MICRO)
# error "Not supported"
#elif defined(ESP_PLATFORM)
# define OLED_CS0     20
# define OLED_CS1     21
# define OLED_CS2     3
# define OLED_CS3     7
# define OLED_DC      5
# define OLED_RESET  -1
#endif

#define __ASSERT_USE_STDERR
#include <assert.h>

#include <SPI.h>
#include <Wire.h>

#include <Adafruit_GFX.h>

#undef DOINA3221
#undef DOSSD1306
#define DOSH1106

#if defined(DOINA3221)
# include "Beastdevices_INA3221.h"
#endif

#if defined(DOSSD1306)
# include <Adafruit_SSD1306.h>
# define DPY Adafruit_SDD1306
#endif

#if defined(DOSH1106)
# include <Adafruit_SH110X.h>
# define DPY Adafruit_SH1106G
#endif

#define NUMD  4
static DPY* displays[NUMD];

#include "oled.h"



const uint32_t bitrate = 2000000;
#if defined(DOSH1106)
static Adafruit_SH1106G dpy0(128, 64, &SPI, OLED_DC, OLED_RESET, OLED_CS0, bitrate);
static Adafruit_SH1106G dpy1(128, 64, &SPI, OLED_DC, OLED_RESET, OLED_CS1, bitrate);
static Adafruit_SH1106G dpy2(128, 64, &SPI, OLED_DC, OLED_RESET, OLED_CS2, bitrate);
#endif

#if defined(DOSSD1306)
static Adafruit_SSD1306 dpy0(128, 64, &SPI, OLED_DC, OLED_RESET, OLED_CS0, bitrate);
static Adafruit_SSD1306 dpy1(128, 64, &SPI, OLED_DC, OLED_RESET, OLED_CS1, bitrate);
static Adafruit_SSD1306 dpy1(128, 64, &SPI, OLED_DC, OLED_RESET, OLED_CS2, bitrate);
#endif

#if defined(DOINA3221)
static Beastdevices_INA3221 ina3221(INA3221_ADDR40_GND);
#endif

static char status_lines[2][11];
static uint8_t status_line_dirty[2];

static uint32_t last_time_stamp;

static void setupSerial(void)
{
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i)
    delay(100);
  Serial.println(BOARDNAME " is now ready.");
}


static void setupDisplays(void)
{
  const int8_t cs[NUMD] = { OLED_CS0, OLED_CS1, OLED_CS2, OLED_CS3 };
  for (int d=0; d<NUMD; ++d)
  {
    displays[d] = new DPY(128, 64, &SPI, OLED_DC, OLED_RESET, cs[d]);
    
#if defined(DOSH1106)
    const int ir = displays[d]->begin(0,true);
#else
    const int ir = displays[d]->begin(SSD1306_EXTERNALVCC); // SSD1306_SWITCHCAPVCC
#endif

    if (!ir)
    {
      Serial.println(F("display->begin() failed"));
      delete displays[d];
      displays[d]=0;
    }
    else
    {
     displays[d]->clearDisplay();
     displays[d]->setTextSize(4);
     displays[d]->setTextColor(1, 0);
     displays[d]->display();
    }
  }
}


static void scan(void)
{
  uint8_t count = 0;
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Finished.");
  Serial.print ("Discovered ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
}



#if defined(DOINA3221)
static void printCurrentForChannel(ina3221_ch_t c)
{
  const float current = ina3221.getCurrent(c);
  const float voltage = ina3221.getVoltage(c);
  Serial.print( "Channel " );
  Serial.print(c);
  Serial.print(": ");
  Serial.print( current, 3 );
  Serial.print("A");
  Serial.print(" ");
  Serial.print( voltage, 3 );
  Serial.println("V");
}
#endif

void setup()
{
  setupSerial();

#if defined( LED_BUILTIN_TX)
  // No TX and RX leds, please.
  pinMode( LED_BUILTIN_TX, INPUT);
  pinMode( LED_BUILTIN_RX, INPUT);
#endif

  // Join the i2c bus as a master.
  Wire.begin(I2CSDA, I2CSCL);

  Wire.setClock(400000);

#if 1
  Serial.println("Scanning...");
  scan();
#endif

#if defined(DOINA3221)
  Serial.println("Initializing INA2113 current sensor.");
  ina3221.begin();
  ina3221.reset();
  ina3221.setShuntRes(100, 100, 100);
  printCurrentForChannel(INA3221_CH1);
  printCurrentForChannel(INA3221_CH2);
  printCurrentForChannel(INA3221_CH3);
#endif

  setupDisplays();
  
  last_time_stamp = millis();
}



void loop()
{
  const uint32_t current_time_stamp = millis();
  uint32_t elapsed;
  if ( current_time_stamp < last_time_stamp )
  {
    Serial.println("Timestamp has wrapped around.");
    elapsed = 0; // wrapped around (in 51 days.)
  }
  else
  {
    elapsed = current_time_stamp - last_time_stamp;
  }
  last_time_stamp = current_time_stamp;


  static uint64_t counter = 0;
  counter += 1;

  const uint8_t offs = (counter>>5) & 0xf;
  for (int d=0; d<NUMD; ++d)
  {
    char line[6];
    uint8_t doff = (NUMD-1-d)*3;
    line[0] = 'A' + offs + doff + 0;
    line[1] = ' ';
    line[2] = 'A' + offs + doff + 1;
    line[3] = ' ';
    line[4] = 'A' + offs + doff + 2;
    line[5] = 0;
    displays[d]->setCursor(0,0);
    displays[d]->print(line);
    displays[d]->display();
  }

  displays[0]->drawBitmap(30, 16,  fntdat+0, 24, 96, 1);
  displays[0]->display();

}


void __assert(const char* __func, const char* __file, int __lineno, const char *__sexp)
{
  // transmit diagnostic informations through serial link.
  Serial.println(__func);
  Serial.println(__file);
  Serial.println(__lineno, DEC);
  Serial.println(__sexp);
  Serial.flush();
  // abort program execution.
  abort();
}
