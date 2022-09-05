
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
#elif defined(ARDUINO_ESP32C3_DEV) || defined(ESP_PLATFORM)
# define BOARDNAME "ESP32C3"
# define I2CSDA 0
# define I2CSCL 1
# define PINLEDR 7
# define PINLEDY 8
# define PINLEDG 10
# define ENCA 4
# define ENCB 5
# define ENCSW 6
#else
# error "Unknown platform."
#endif

#define RECALIBRATE_TARGET 440

#define THRESHOLD_LO  700
#define THRESHOLD_HI  1000

#define UI_MODE_GRAPH 0
#define UI_MODE_MENU  1

#define SWITCH_NONE           0
#define SWITCH_SHORT          1
#define SWITCH_LONG           2
#define SWITCH_START_PRESSING 3

#define MENU_ITEM_0_EXIT        0
#define MENU_ITEM_1_BRIGHTNESS  1
#define MENU_ITEM_2_CALIBRATE   2
#define MENU_ITEM_3_ABOUT       3
#define MENU_ITEM_COUNT 4

#define BLANK_TEXT "          "

#define __ASSERT_USE_STDERR
#include <assert.h>

#include <Wire.h>

// For barometric sensor
// Non-blocking version of Adafruit lib from:
// https://github.com/stolk/Adafruit_MPL3115A2_Library
#include "Adafruit_MPL3115A2_NB.h"

// For CO2 sensor
#include <SensirionI2CScd4x.h>


#define NUMZ  7 // Nr of zoom levels
// Zoom 64x   600.000s    6 samples/hr
// Zoom 32x   300.000s   12 samples/hr
// Zoom 16x   150.000s   24 samples/hr
// Zoom  8x    75.000s   48 samples/hr
// Zoom  4x    37.500s   96 samples/hr
// Zoom  2x    18.750s  192 samples/hr
// Zoom  1x     9.375s  384 samples/hr

static int curz=3;  // current zoom level: 8x

#define OLEDADDR0 0x3c
#define OLEDADDR1 0x3d

static Adafruit_MPL3115A2_NB baro;

static SensirionI2CScd4x cdos;

static char status_lines[2][11];
static uint8_t status_line_dirty[2];
static uint8_t graph_row_dirty[2][6];

static const int32_t periods[NUMZ] =
{
  600000,300000,150000,75000,37500,18750,9375,
};
static const uint16_t samples_per_hr[NUMZ] =
{
  6,12,24,48,96,192,384,
};
static const char *graphrng[NUMZ] =
{
  "43 Hrs  ", "21 Hrs  ", "11 Hrs  ", "5 Hrs   ", "160 Mins", "80 Mins ", "40 Mins ",
};
static uint8_t dimming_values[3] =
{
  0x60, // Little dimming.
  0x01, // Max dimming.
  0x00, // Off.
};


static uint32_t last_time_stamp  = 0;

static uint16_t co2 = 0;

static uint8_t datalog_lo[NUMZ][256];
static uint8_t datalog_hi[NUMZ][256];
static uint8_t logidx[NUMZ];
static int32_t sample_delay[NUMZ];
static int16_t num_measurements[NUMZ];

static uint8_t knob_state;
static uint32_t knob_ms_held;
static uint8_t knob_primed;

static uint8_t dimming_mode;

static int32_t notification_time;

static uint8_t ui_mode = UI_MODE_GRAPH;
static uint8_t menu_item;

#include "oled.h"
#include "knob.h"

static void setupSerial(void)
{
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  for (int i=0; i<20 && !Serial; ++i)
     delay(100);
  Serial.println(BOARDNAME " is now ready.");
}


static void setupBarometer(void)
{
  if (!baro.begin())
    Serial.println("Could not find bariometric sensor. Check wiring.");
  else
  {
    baro.setSeaPressure(1012 );
    Serial.println("Barometer set up.");
  }
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

  err = cdos.setAutomaticSelfCalibration(0);
  assert(!err);

  err = cdos.startPeriodicMeasurement();
  assert(!err);

  Serial.println("CO2 sensor was set up.");
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
      delay (1);
    } // end of good response
  } // end of for loop
  Serial.println ("Finished.");
  Serial.print ("Discovered ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
}


// Set the leds, modulated by dimming_mode
static void set_leds(uint8_t r, uint8_t y, uint8_t g )
{
  const uint8_t pulsewidths[3] =
  {
    0, 160, 250,	// 0 is full bright, 255 is no light.
  };
  const uint8_t pw = pulsewidths[ dimming_mode ];
  const uint8_t rv = r ? pw : 255;
  const uint8_t yv = y ? pw : 255;
  const uint8_t gv = g ? pw : 255;
  analogWrite(PINLEDR, rv);
  analogWrite(PINLEDY, yv);
  analogWrite(PINLEDG, gv);
}


// Set the leds, based on CO2 value.
static void update_leds(void)
{
  // Activates the red, ylw or grn LED, based on CO2.
  if ( co2 < THRESHOLD_LO )
    set_leds(0,0,1);
  else if ( co2 < THRESHOLD_HI )
    set_leds(0,1,0);
  else
    set_leds(1,0,0);
}


static uint16_t updateBarometer(void)
{
  static uint16_t pre = 0;

  float pressure=0;
  uint8_t phase = baro.getPressureNonBlocking(pressure);

  if ( !phase )
  {
    pre = (uint16_t) (pressure+0.5f);
    // The CO2 measurement is more accurate if we provide ambient pressure data.
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
      oled_write_strip
      (
        OLEDADDR0+dpy,
        7-i,
        logidx[curz]+dpy*128,
        i*8,
        datalog_lo[curz],
        datalog_hi[curz],
        samples_per_hr[curz],
        dpy==0
      );
      dpy = (dpy+1)&1;
      return 1;
    }
  }
  dpy = (dpy+1)&1;
  return 0;
}


// The top yellow section of each display is used for status text.
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


static void calib(void)
{
  uint16_t err = cdos.stopPeriodicMeasurement();
  assert(!err);

  delay(500);

  uint16_t target = RECALIBRATE_TARGET;
  uint16_t frc = 0x5555;
  err = cdos.performForcedRecalibration(target, frc );
  assert(!err);

  Serial.print("frc:"); Serial.println(frc); // On my sensor, this returned: 31857

  err = cdos.startPeriodicMeasurement();
  assert(!err);
}


static void cycle_dimming_mode(void)
{
  dimming_mode = dimming_mode+1;
  dimming_mode = dimming_mode > 2 ? 0 : dimming_mode;
  oled_set_contrast( OLEDADDR0, dimming_values[dimming_mode] );
  oled_set_contrast( OLEDADDR1, dimming_values[dimming_mode] );
  update_leds();
}


void setup()
{
  setupSerial();

#if defined( LED_BUILTIN_TX )
  // No TX and RX leds, please.
  pinMode( LED_BUILTIN_TX, INPUT);
  pinMode( LED_BUILTIN_RX, INPUT);
#endif

  pinMode( PINLEDR, OUTPUT );
  pinMode( PINLEDY, OUTPUT );
  pinMode( PINLEDG, OUTPUT );
  set_leds(0,0,0);

  // Wait 20ms before we do anything: I suspect the SCD41 needs a little time?
  delay(20);

  // Join the i2c bus as a master.
  Wire.begin(I2CSDA, I2CSCL);

  Wire.setClock(400000);

#if 1
  Serial.println("Scanning...");
  scan();
#endif

  knob_setup(0, ENCA, ENCB, ENCSW);

  oled_setup(OLEDADDR0);
  oled_pattern(OLEDADDR0, 0x0, 0 );
  oled_write_row(OLEDADDR0, 0, 0, "GSAS Inc  ", 0x0);
  oled_write_row(OLEDADDR0, 1, 0, "GSAS Inc  ", 0x0);

  oled_setup(OLEDADDR1);
  oled_pattern(OLEDADDR1, 0x0, 0 );
  oled_write_row(OLEDADDR1, 0, 0, "(c)2022   ", 0x0);
  oled_write_row(OLEDADDR1, 1, 0, "(c)2022   ", 0x0);

  oled_set_contrast(OLEDADDR0, 0x60);
  oled_set_contrast(OLEDADDR1, 0x60);

  setupBarometer();

  setupCO2Sensor();

  // Read initial knob state.
  knob_state = knob_switch_value(0);

  memset(datalog_lo, 0xff, sizeof(datalog_lo));
  memset(datalog_hi, 0xff, sizeof(datalog_hi));

  mark_graph_dirty();

  for ( int z=0; z<NUMZ; ++z )
    sample_delay[z] = periods[z];

  last_time_stamp = millis();
}


static void mark_graph_dirty()
{
  memset(graph_row_dirty, 0xff, sizeof(graph_row_dirty));
}


static void update_status(uint16_t pre, uint16_t co2)
{
  uint8_t k,h,d,u; // kilo,hecto,deca,uni.

  char* p = status_lines[1];
  k = (co2/1000);
  h = (co2%1000)/100;
  d = (co2%100)/10;
  u = (co2%10);
  *p++ = ' ';
  *p++ = ' ';
  *p++ = k ? 48 + k : ' ';
  *p++ = 48 + h;
  *p++ = 48 + d;
  *p++ = 48 + u;
  *p++ = ' ';
  *p++ = 'p';
  *p++ = 'p';
  *p++ = 'm';
  *p++ = 0;
  status_line_dirty[1] = 0xff;

  p = status_lines[0];
#if 0 // This just confuses people, thinking it is 2 graphs. Ugh!
  k = (pre/1000);
  h = (pre%1000)/100;
  d = (pre%100)/10;
  u = (pre%10);
  if (k) *p++ = 48 + k;
  *p++ = 48 + h;
  *p++ = 48 + d;
  *p++ = 48 + u;
  *p++ = ' ';
  *p++ = 'h';
  *p++ = 'P';
  *p++ = 'a';
  *p++ = 0;
#else
  *p++ = ' ';
  *p++ = ' ';
  *p++ = ' ';
  *p++ = ' ';
  *p++ = ' ';
  *p++ = ' ';
  *p++ = ' ';
  *p++ = ' ';
  *p++ = 0;
#endif
  status_line_dirty[0] = 0xff;
}


void loop()
{
  // Time keeping.
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

  for ( int z=0; z<NUMZ; ++z )
    sample_delay[z] -= elapsed;

  // Check which zoom levels need to scroll to the left at this tick.
  for ( int z=0; z<NUMZ; ++z )
  {
    if ( sample_delay[z] <= 0 )
    {
      sample_delay[z] += periods[z];
      logidx[z] += 1;
      num_measurements[z] = 0;
      if ( curz == z )
        mark_graph_dirty();
    }
  }

  handle_knob_input(elapsed);

  // Evict stale status messages.
  if ( notification_time > 0 )
  {
    notification_time -= elapsed;
    uint8_t lnr = (notification_time>>6) & 0x3;
    set_leds( lnr==0, lnr==1, lnr==2 );
    if ( notification_time <= 0 )
    {
      strcpy(status_lines[0], BLANK_TEXT);
      strcpy(status_lines[1], BLANK_TEXT);
      status_line_dirty[0] = 0xff;
      status_line_dirty[1] = 0xff;
      set_leds(0,0,0);
    }
  }

  if ( ui_mode == UI_MODE_GRAPH )
  {
    // Update status lines and graph.
    uint8_t updated = update_status_lines();
    if (!updated)
      updated = update_graph();
  }

  // Get the barometric pressure.
  const uint16_t pre = updateBarometer();

  // Read the CO2 sensor, and record sample if a new one is available.
  {
    uint16_t rdy = 0xffff;
    uint16_t err = cdos.getDataReadyStatus(rdy);
    assert(err==0);
    if ( (rdy & 0x7ff ) != 0 )
    {
      float temperature;
      float humidity;
      uint16_t err = cdos.readMeasurement(co2, temperature, humidity);
      assert(err==0);
      Serial.print("co2:");
      Serial.print(co2);
      Serial.print("  temperature:");
      Serial.print(temperature);
      Serial.print("  humidity:");
      Serial.print(humidity);
      Serial.print("\n");

      if ( notification_time <= 0 )
        update_status( pre, co2 );

      update_leds();

      int16_t v = co2 < 400 ? 0 : (co2 - 400) / 22;
      v = v > 47 ? 47 : v;
      const uint8_t s = (uint8_t)v;
      for ( int z=0; z<NUMZ; ++z )
      {
        datalog_lo[z][logidx[z]] = s < datalog_lo[z][logidx[z]] || !num_measurements[z] ? s : datalog_lo[z][logidx[z]];
        datalog_hi[z][logidx[z]] = s > datalog_hi[z][logidx[z]] || !num_measurements[z] ? s : datalog_hi[z][logidx[z]];
        num_measurements[z] += 1;
      }
    }
  }
}


void handle_knob_input(uint32_t t_elapsed)
{
  const int8_t delta = knob_update(0);

  const uint8_t s = knob_switch_value(0);

  uint8_t sw = SWITCH_NONE;

  // Did knob switch status change?
  if ( s != knob_state )
  {
    if ( s == 0 )
    {
      sw = SWITCH_START_PRESSING;
    }
    else
    {
      // Short press was just released?
      if ( knob_ms_held > 1 && knob_ms_held < 800 )
      {
        sw = SWITCH_SHORT;
      }
    }

    knob_ms_held = 0;
    knob_state = s;
    knob_primed = 1;
  }
  else if ( s == 0 )
  {
    knob_ms_held += t_elapsed;
  }

  if ( knob_state == 0 && knob_ms_held > 1500 && knob_primed )
  {
    knob_primed = 0;
    sw = SWITCH_LONG;
  }

  switch(ui_mode)
  {
    case UI_MODE_GRAPH:
      handle_knob_input_graph(sw, delta);
      break;
    case UI_MODE_MENU:
      handle_knob_input_menu(sw, delta);
      break;
  }
}


bool is_button_being_held()
{
  return knob_primed == 1 && knob_state == 0;
}


void switch_ui_mode(int8_t mode)
{
  ui_mode = mode;
  switch(mode)
  {
    case UI_MODE_GRAPH:
      status_line_dirty[0] = 0xff;
      status_line_dirty[1] = 0xff;
      mark_graph_dirty();
      break;
    case UI_MODE_MENU:
      menu_item = MENU_ITEM_0_EXIT;
      update_menu();
      break;
  }
}


void update_menu()
{
  update_menu_text(OLEDADDR0,
                   "   MENU   ",
                   menu_item == MENU_ITEM_0_EXIT ? "EXIT      " : "Exit      ",
                   menu_item == MENU_ITEM_1_BRIGHTNESS ? "BRIGHTNESS" : "Brightness",
                   menu_item == MENU_ITEM_2_CALIBRATE ? "CALIBRATE " : "Calibrate ");
                   // TODO: Implement menu scroll.
                   // menu_item == MENU_ITEM_3_ABOUT ? "ABOUT     " : "About     "

  String ppm_text;
  bool holding;
  switch(menu_item)
  {
    case MENU_ITEM_2_CALIBRATE:
      ppm_text = String("to ");
      ppm_text += RECALIBRATE_TARGET;
      ppm_text += " ppm";
      ppm_text += BLANK_TEXT;
      ppm_text = ppm_text.substring(0, strlen(BLANK_TEXT));
      holding = is_button_being_held();
      update_menu_text(OLEDADDR1,
                      holding ? "CONTINUE  " : "Long press",
                      holding ? "HOLDING TO" : "to        ",
                      "calibrate ",
                      ppm_text.c_str());
      break;
    default:
      update_menu_text(OLEDADDR1,
                      BLANK_TEXT,
                      BLANK_TEXT,
                      BLANK_TEXT,
                      BLANK_TEXT);
      break;
  }
}


void update_menu_text(int devaddr, const char* line1, const char* line2, const char* line3, const char* line4)
{
  oled_write_row(devaddr, 0, 0, line1, 0x0);
  oled_write_row(devaddr, 1, 0, line1, 0x0);

  oled_write_row(devaddr, 2, 0, line2, 0x0);
  oled_write_row(devaddr, 3, 0, line2, 0x0);

  oled_write_row(devaddr, 4, 0, line3, 0x0);
  oled_write_row(devaddr, 5, 0, line3, 0x0);

  oled_write_row(devaddr, 6, 0, line4, 0x0);
  oled_write_row(devaddr, 7, 0, line4, 0x0);
}


void handle_knob_input_menu(int8_t sw, int8_t delta)
{
  switch(menu_item)
  {
    case MENU_ITEM_0_EXIT:
      if ( sw == SWITCH_SHORT )
      {
        switch_ui_mode(UI_MODE_GRAPH);
        return;
      }
      break;
    case MENU_ITEM_1_BRIGHTNESS:
      if ( sw == SWITCH_SHORT )
      {
        cycle_dimming_mode();
        return;
      }
      break;
    case MENU_ITEM_2_CALIBRATE:
      if ( sw == SWITCH_START_PRESSING || sw == SWITCH_SHORT )
      {
        update_menu();
        return;
      }
      if ( sw == SWITCH_LONG )
      {
        calib();
        notification_time = 3000;
        strcpy(status_lines[0], "CALIBRATE ");
        strcpy(status_lines[1], "COMPLETE  ");
        status_line_dirty[0] = 0xff;
        status_line_dirty[1] = 0xff;
        switch_ui_mode(UI_MODE_GRAPH);
        return;
      }
      break;
    case MENU_ITEM_3_ABOUT:
      if ( sw == SWITCH_SHORT )
      {
        update_menu_text(OLEDADDR1,
                        "WARNING   ",
                        "This is   ",
                        "not a     ",
                        "chew toy. ");

        Serial.println("This is not a chew toy.");
        return;
      }
      break;
  }

  if ( delta != 0 )
  {
    // Prevent blind navigation while screen is off.
    if ( dimming_values[dimming_mode] == 0x00 )
    {
      cycle_dimming_mode();
    }

    menu_item += delta;
    menu_item = menu_item % MENU_ITEM_COUNT;
    update_menu();
  }
}


void handle_knob_input_graph(int8_t sw, int8_t delta)
{
    if ( sw == SWITCH_SHORT )
    {
      switch_ui_mode(UI_MODE_MENU);
      return;
    }

  // Handle knob turns.
  if ( delta==1 && curz < NUMZ-1 )
  {
    curz++; // Zoom in.
    mark_graph_dirty();
    strcpy(status_lines[0], graphrng[curz]);
    status_line_dirty[0] = 0xff;
  }
  if ( delta==-1 && curz > 0 )
  {
    curz--; // Zoom out.
    mark_graph_dirty();
    strcpy(status_lines[0], graphrng[curz]);
    status_line_dirty[0] = 0xff;
  }
}


void __assert(const char* __func, const char* __file, int __lineno, const char *__sexp)
{
  // Transmit diagnostic informations through serial link.
  Serial.println(__func);
  Serial.println(__file);
  Serial.println(__lineno, DEC);
  Serial.println(__sexp);
  Serial.flush();
  // Abort program execution.
  abort();
}
