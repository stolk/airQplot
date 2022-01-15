
#if defined(ARDUINO_AVR_ITSYBITSY32U4_3V)
# define BOARDNAME "Itsy Bitsy 3.3V"
#elif defined(ARDUINO_AVR_MICRO)
# define BOARDNAME "Arduino Pro Micro"
# define PINLEDR 7
# define PINLEDY 8
# define PINLEDG 9
# define ENCA 4
# define ENCB 5
# define ENCSW 6
#elif defined(ARDUINO_TRINKET_M0)
# define BOARDNAME "Trinket M0"
#elif defined(ARDUINO_ESP32C3_DEV)
# define BOARDNAME "ESP32C3"
# define I2CSDA 0
# define I2CSCL 1
# define PINLEDR 2
# define PINLEDY 3
# define PINLEDG 4
# define ENCA 4
# define ENCB 5
# define ENCSW 6
#else
# error "Unknown platform."
#endif

#define __ASSERT_USE_STDERR
#include <assert.h>

#include <Wire.h>

// For barometric sensor
#include <Adafruit_MPL3115A2.h>

// For CO2 sensor
#include <SensirionI2CScd4x.h>


#define SAMPLEINTERVAL 300000 //300000 // In milliseconds (5 minutes.)

#define OLEDADDR0 0x3c
#define OLEDADDR1 0x3d

#define SWITCHPIN 12




Adafruit_MPL3115A2 baro;

SensirionI2CScd4x cdos;


static char status_lines[2][11];
static uint8_t status_line_dirty[2];
static uint8_t graph_row_dirty[2][6];


static int32_t  sample_delay = SAMPLEINTERVAL;
static uint32_t last_time_stamp  = 0;
static uint16_t num_measurements = 0;
static uint32_t sum_measurements = 0;

static uint8_t datalog_lo[256];
static uint8_t datalog_hi[256];
static uint8_t logidx=0;


#include "oled.h"
#include "knob.h"

static void setupSerial(void)
{
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  for (int i=0; i<18 && !Serial; ++i)
     delay(100);
  Serial.println(BOARDNAME " is now ready.");
}


static void setupBarometer(void)
{
  if (!baro.begin())
    Serial.println("Could not find bariometric sensor. Check wiring.");
  else
    baro.setSeaPressure(1012 );
}


static void setupCO2Sensor(void)
{
  uint16_t err = 0;
  char errorMessage[128];

  cdos.begin(Wire);

  // stop potentially previously started measurement
  err = cdos.stopPeriodicMeasurement();
  assert(!err);

#if 0
  err = cdos.performFactoryReset();
  assert(!err);
#endif

#if 1
  err = cdos.setAutomaticSelfCalibration(0);
  assert(!err);
#endif

  err = cdos.startPeriodicMeasurement();
  assert(!err);
}


static void correctForAmbientPressure(uint16_t pressure)
{
  const uint16_t err = cdos.setAmbientPressure(pressure);
  assert(err == 0);
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


static uint16_t updateBarometer(void)
{
  static uint16_t pre = 0;

  float pressure=0;
  uint8_t phase = baro.getPressureNonBlocking(pressure);

  if ( !phase )
  {
    pre = (uint16_t) (pressure+0.5f);
    // The CO2 measurement is more accurate if we provice ambient pressure data.
    correctForAmbientPressure(pre);
  }
  return pre;
}


static uint8_t update_graph(void)
{
  static uint8_t dpy=0;
  for ( uint8_t i=0; i<6; ++i )
  {
    if ( graph_row_dirty[dpy][i] )
    {
      graph_row_dirty[dpy][i] = 0;
      // Writes a row of 8 pixels.
      oled_write_strip(OLEDADDR0+dpy, 7-i, logidx+dpy*128, i*8, datalog_lo, datalog_hi, dpy==0);
      dpy = (dpy+1)&1;
      return 1;
    }
  }
  dpy = (dpy+1)&1;
  return 0;
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


static void write_datalog(void)
{
  // Write 6 lines of 8 pixels each, to the displays.
  for ( int i=0; i<6; ++i )
  {
    oled_write_strip(OLEDADDR0, 7-i, logidx+  0, i*8, datalog_lo, datalog_hi, 1 );
    oled_write_strip(OLEDADDR1, 7-i, logidx+128, i*8, datalog_lo, datalog_hi, 0 );
  }
}

#if 0
static void calib(void)
{
  uint16_t err = cdos.stopPeriodicMeasurement();
  assert(!err);

  delay(500);
  
  uint16_t target = 440;
  uint16_t frc = 0x5555;
  err = cdos.performForcedRecalibration(target, frc );
  assert(!err);

  Serial.print("frc:"); Serial.println(frc); // On my sensor, this returned: 31857

  err = cdos.startPeriodicMeasurement();
  assert(!err);  
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

  pinMode( PINLEDR, OUTPUT );
  pinMode( PINLEDY, OUTPUT );
  pinMode( PINLEDG, OUTPUT );
  digitalWrite( PINLEDR, 0 );
  digitalWrite( PINLEDY, 0 );
  digitalWrite( PINLEDG, 0 );

  // Join the i2c bus as a master.
  Wire.begin(I2CSDA, I2CSCL);

  Wire.setClock(400000);

#if 1
  Serial.println("Scanning...");
  scan();
#endif

  knob_setup(0, ENCA,ENCB,ENCSW);

  setupBarometer();
  Serial.println("barometer was set up.");

  setupCO2Sensor();
  Serial.println("CO2 sensor was set up.");

  memset(datalog_lo, 0xff, sizeof(datalog_lo));
  memset(datalog_hi, 0xff, sizeof(datalog_hi));

  oled_setup(OLEDADDR0);
  oled_pattern(OLEDADDR0, 0x0, 0 );
  oled_write_row(OLEDADDR0, 0, 0, "GSAS Inc  ", 0x0);
  oled_write_row(OLEDADDR0, 1, 0, "GSAS Inc  ", 0x0);

  oled_setup(OLEDADDR1);
  oled_pattern(OLEDADDR1, 0x0, 0 );
  oled_write_row(OLEDADDR1, 0, 0, "(c)2022   ", 0x0);
  oled_write_row(OLEDADDR1, 1, 0, "(c)2022   ", 0x0);

  oled_set_contrast( OLEDADDR0, 0x60 );
  oled_set_contrast( OLEDADDR1, 0x60 );

  memset(graph_row_dirty, 0xff, sizeof(graph_row_dirty));
  //write_datalog();

  last_time_stamp = millis();
  num_measurements = 0;
  sum_measurements = 0;
}


static void update_status(uint16_t pre, uint16_t co2)
{
  uint8_t k,h,d,u; // kilo,hecto,deca,uni.
  
  k = (co2/1000);
  h = (co2%1000)/100;
  d = (co2%100)/10;
  u = (co2%10);
  char* p = status_lines[0];
  if (k) *p++ = 48 + k;
  *p++ = 48 + h;
  *p++ = 48 + d;
  *p++ = 48 + u;
  *p++ = ' ';
  *p++ = 'p';
  *p++ = 'p';
  *p++ = 'm';
  *p++ = ' ';
  *p++ = 0;

  k = (pre/1000);
  h = (pre%1000)/100;
  d = (pre%100)/10;
  u = (pre%10);
  p = status_lines[1];
  if (k) *p++ = 48 + k;
  *p++ = 48 + h;
  *p++ = 48 + d;
  *p++ = 48 + u;
  *p++ = ' ';
  *p++ = 'h';
  *p++ = 'P';
  *p++ = 'a';
  *p++ = 0;

  status_line_dirty[0] = 0xff;
  status_line_dirty[1] = 0xff; 
}


static int8_t lastv[6] = {-1,-1,-1,-1,-1,-1};

static uint16_t idlecountlo=0;
static uint16_t idlecounthi=0;



void loop()
{
  const uint32_t current_time_stamp = millis();
  uint32_t elapsed;
  if ( current_time_stamp < last_time_stamp )
  {
    Serial.println("Timestamp has wrapped around.");
    elapsed = SAMPLEINTERVAL/2; // wrapped around (in 51 days.)
  }
  else
  {
    elapsed = current_time_stamp - last_time_stamp;
  }
  last_time_stamp = current_time_stamp;
  sample_delay -= elapsed;

  if ( sample_delay <= 0 )
  {
    if (num_measurements==0)
      Serial.print("ERROR: num_measurements is 0?");
    const uint16_t avg = num_measurements ? sum_measurements / num_measurements : 0;
    Serial.print("Interval: "); Serial.print(logidx);
    Serial.print(" sample_delay: "); Serial.print(sample_delay);
    Serial.print(" elapsed: "); Serial.print(elapsed);
    Serial.print(" count: "); Serial.print(num_measurements);
    Serial.print(" average: "); Serial.print(avg); Serial.println(" ppm");
    sample_delay += SAMPLEINTERVAL;
    num_measurements = 0;
    sum_measurements = 0;
    logidx += 1;
    memset(graph_row_dirty, 0xff, sizeof(graph_row_dirty));
    //write_datalog();
  }

  uint8_t idle=1;
  const int8_t delta = knob_update(0);
  if ( delta )
  {
    Serial.println(delta);
  }

  // Update the amount of time that knobs have been idle.
  if (idle && idlecounthi<0xffff)
  {
    idlecountlo++;
    if (idlecountlo==0)
      idlecounthi++;
  }

  // If we have been idle a bit, we could update the display, but just one row.
  if (idlecountlo > 240 || idlecounthi > 0)
  {
  }

#if 0
  static uint8_t sleepmode = 0;
  // Check if the display needs to be dimmed.
  uint8_t newmode = 0;
  if ( idlecounthi > 0 )
    newmode = 1;

  if ( newmode != sleepmode )
  {
    if ( newmode == 1 && sleepmode == 0 )
    {
      oled_set_contrast( OLEDADDR0, 0x30 );
      oled_set_contrast( OLEDADDR1, 0x30 );
    } 
    if ( newmode == 0 && sleepmode == 1 )
    {
      oled_set_contrast( OLEDADDR0, 0x50 );
      oled_set_contrast( OLEDADDR1, 0x50 );
    }
    sleepmode = newmode;
  }
#endif
  
  // Get the barometric pressure.
  const uint16_t pre = updateBarometer();

  uint8_t updated = update_status_lines();
  if (!updated)
    updated = update_graph();

  {
    uint16_t rdy = 0xffff;
    uint16_t err = cdos.getDataReadyStatus(rdy);
    assert(err==0);
    if ( (rdy & 0x7ff ) != 0 )
    {
      uint16_t co2;
      float temperature;
      float humidity;
      uint16_t err = cdos.readMeasurement(co2,temperature,humidity);
      assert(err==0);
      Serial.print("co2:");
      Serial.print(co2);
      Serial.print("  temperature:");
      Serial.print(temperature);
      Serial.print("  humidity:");
      Serial.print(humidity);
      Serial.print("\n");
      
      update_status( pre, co2 );
      
      int16_t v = co2 < 400 ? 0 : (co2 - 400) / 32;
      v = v > 47 ? 47 : v;
      const uint8_t s = (uint8_t)v;
      datalog_lo[logidx] = s < datalog_lo[logidx] || !num_measurements ? s : datalog_lo[logidx];
      datalog_hi[logidx] = s > datalog_hi[logidx] || !num_measurements ? s : datalog_hi[logidx];
      num_measurements += 1;
      sum_measurements += co2; 
    }
  }

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