
#if defined(ARDUINO_AVR_ITSYBITSY32U4_3V)
# define BOARDNAME "Itsy Bitsy 3.3V"
#elif defined(ARDUINO_AVR_MICRO)
# define BOARDNAME "Arduino Pro Micro"
#elif defined(ARDUINO_TRINKET_M0)
# define BOARDNAME "Trinket M0"
#elif defined(ARDUINO_ESP32C3_DEV) || defined(ESP_PLATFORM)
# define BOARDNAME "ESP32C3"
# define I2CSDA 0
# define I2CSCL 1
#else
# error "Unknown platform."
#endif

#define __ASSERT_USE_STDERR
#include <assert.h>

#include <Wire.h>

#include "oled.h"


#define OLEDADDR0 0x3c
#define OLEDADDR1 0x3d

static char status_lines[2][11];
static uint8_t status_line_dirty[2];

static uint32_t last_time_stamp;

static void setupSerial(void)
{
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  for (int i=0; i<20 && !Serial; ++i)
     delay(100);
  Serial.println(BOARDNAME " is now ready.");
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


static uint8_t update_status_lines(void)
{
  static uint8_t dpy=0;
  if ( status_line_dirty[dpy] )
  {
    oled_write_row(OLEDADDR0+dpy, 0, 0, status_lines[dpy], 0x0);
    oled_write_row(OLEDADDR0+dpy, 1, 0, status_lines[dpy], 0x0);
    status_line_dirty[dpy] = 0;
    dpy = (dpy+1)&1;
    return 1;
  }
  dpy = (dpy+1)&1;
  return 0;
}


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

  oled_setup(OLEDADDR0);
  oled_pattern(OLEDADDR0, 0x0, 0 );
  oled_write_row(OLEDADDR0, 0, 0, "GSAS Inc  ", 0x0);
  oled_write_row(OLEDADDR0, 1, 0, "GSAS Inc  ", 0x0);

#if 0
  oled_setup(OLEDADDR1);
  oled_pattern(OLEDADDR1, 0x0, 0 );
  oled_write_row(OLEDADDR1, 0, 0, "(c)2022   ", 0x0);
  oled_write_row(OLEDADDR1, 1, 0, "(c)2022   ", 0x0);
#endif

  oled_set_contrast( OLEDADDR0, 0x60 );
  //oled_set_contrast( OLEDADDR1, 0x60 );

  //status_line_dirty[0] = 0xff;
  //status_line_dirty[1] = 0xff; 
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
