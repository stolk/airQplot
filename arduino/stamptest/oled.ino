
#include <Wire.h>

//~ DEFINES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Some defines for the SSD1306 controller driving a 128x64 resolution OLED display
// PART     - http://www.simplelabs.co.in/content/96-blue-i2c-oled-module
// DATASHEET  - https://www.adafruit.com/datasheets/SSD1306.pdf


// The SSD1306 datasheet (pg.20) says that a control byte has to be sent before sending a command
// Control byte consists of 
// bit 7    : Co   : Continuation bit - If 0, then it assumes all the next bytes are data (no more control bytes).
//        :    You can send a stream of data, ie: gRAM dump - if Co=0
//        :        For Command, you'd prolly wanna set this - one at a time. Hence, Co=1 for commands
//        :    For Data stream, Co=0 :)
// bit 6      : D/C# : Data/Command Selection bit, Data=1/Command=0
// bit [5-0]  : lower 6 bits have to be 0
#define OLED_CONTROL_BYTE_CMD_SINGLE  0x80
#define OLED_CONTROL_BYTE_CMD_STREAM  0x00
#define OLED_CONTROL_BYTE_DATA_STREAM 0x40

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST     0x81  // follow with 0x7F
#define OLED_CMD_DISPLAY_RAM      0xA4
#define OLED_CMD_DISPLAY_ALLON      0xA5
#define OLED_CMD_DISPLAY_NORMAL     0xA6
#define OLED_CMD_DISPLAY_INVERTED     0xA7
#define OLED_CMD_DISPLAY_OFF      0xAE
#define OLED_CMD_DISPLAY_ON       0xAF

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE 0x20  // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define OLED_CMD_SET_COLUMN_RANGE   0x21  // can be used only in HORZ/VERT mode - follow with 0x00 + 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE     0x22  // can be used only in HORZ/VERT mode - follow with 0x00 + 0x07 = PAGE7

// Hardware Config (pg.31)
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40
#define OLED_CMD_SET_SEGMENT_REMAP    0xA1  
#define OLED_CMD_SET_MUX_RATIO      0xA8  // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE    0xC8  
#define OLED_CMD_SET_DISPLAY_OFFSET   0xD3  // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP    0xDA  // follow with 0x12

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV  0xD5  // follow with 0x80
#define OLED_CMD_SET_PRECHARGE      0xD9  // follow with 0x22
#define OLED_CMD_SET_VCOMH_DESELCT    0xDB  // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP    0x8D  // follow with 0x14

// NOP
#define OLED_CMD_NOP          0xE3

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "font.h"

// Box pattern
static PROGMEM const uint8_t patterns[4][16] = 
{
  {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  },
  {
    0x00,0x7E,0x42,0x42,0x42,0x42,0x7E,0x00,
    0x00,0x7E,0x42,0x42,0x42,0x42,0x7E,0x00
  },
  {
    0x81,0x42,0x24,0x18,0x18,0x24,0x42,0x81,
    0x81,0x42,0x24,0x18,0x18,0x24,0x42,0x81
  },
  {
    0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
    0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01
  }
};


static PROGMEM const uint8_t label1900[17] =
{
  0x7f,0x5b,0x41,0x5f, 0x7f,0x51,0x55,0x41, 0x7f,0x41,0x5d,0x41, 0x7f,0x41,0x5d,0x41, 0x7f,
};
static PROGMEM const uint8_t label400[13] =
{
  0x7f,0x71,0x77,0x41, 0x7f,0x41,0x5d,0x41, 0x7f,0x41,0x5d,0x41, 0x7f,
};


void oled_setup( int devaddr )
{
  // keywords:
  // SEG = COL = segment = column byte data on a page
  // Page = 8 pixel tall row. Has 128 SEGs and 8 COMs
  // COM = row

  // Begin the I2C comm with SSD1306's address (SLA+Write)
  Wire.beginTransmission( devaddr );
  // Tell the SSD1306 that a command stream is incoming
  Wire.write(OLED_CONTROL_BYTE_CMD_STREAM);

  // Follow instructions on pg.64 of the dataSheet for software configuration of the SSD1306
  // Turn the Display OFF
  Wire.write(OLED_CMD_DISPLAY_OFF);
  // Set mux ratio to select max number of rows - 64
  Wire.write(OLED_CMD_SET_MUX_RATIO);
  Wire.write(0x3F);
  // Set the display offset to 0
  Wire.write(OLED_CMD_SET_DISPLAY_OFFSET);
  Wire.write(0x00);
  // Display start line to 0
  Wire.write(OLED_CMD_SET_DISPLAY_START_LINE);
  
  // Mirror the x-axis. In case you set it up such that the pins are north.
  // Wire.write(0xA0); - in case pins are south - default
  Wire.write(OLED_CMD_SET_SEGMENT_REMAP);

  // Mirror the y-axis. In case you set it up such that the pins are north.
  // Wire.write(0xC0); - in case pins are south - default
  Wire.write(OLED_CMD_SET_COM_SCAN_MODE);

  // Default - alternate COM pin map
  Wire.write(OLED_CMD_SET_COM_PIN_MAP);
  Wire.write(0x12);
  // set contrast
  Wire.write(OLED_CMD_SET_CONTRAST);
  Wire.write(0x50); // 0x7F is maximum.
  // Set display to enable rendering from GDDRAM (Graphic Display Data RAM)
  Wire.write(OLED_CMD_DISPLAY_RAM);
  // Normal mode!
  Wire.write(OLED_CMD_DISPLAY_NORMAL);
  // Default oscillator clock
  Wire.write(OLED_CMD_SET_DISPLAY_CLK_DIV);
  Wire.write(0x80);
  // Enable the charge pump
  Wire.write(OLED_CMD_SET_CHARGE_PUMP);
  Wire.write(0x14);
  // Set precharge cycles to high cap type
  Wire.write(OLED_CMD_SET_PRECHARGE);
  Wire.write(0x22);
  // Set the V_COMH deselect volatage to max
  Wire.write(OLED_CMD_SET_VCOMH_DESELCT);
  Wire.write(0x30);
  // Horizonatal addressing mode - same as the KS108 GLCD
  Wire.write(OLED_CMD_SET_MEMORY_ADDR_MODE);
  Wire.write(0x00);
  // Turn the Display ON
  Wire.write(OLED_CMD_DISPLAY_ON);

  // End the I2C comm with the SSD1306
  Wire.endTransmission();

}


void oled_write_row(int devaddr, int row, int col, const char* text, uint8_t attr)
{
  const uint8_t sl = strlen(text);
  uint8_t numcol = sl * 12;

  Wire.beginTransmission(devaddr);
  Wire.write(OLED_CONTROL_BYTE_CMD_STREAM);
  Wire.write(OLED_CMD_SET_COLUMN_RANGE);
  Wire.write(col*12+0);
  Wire.write(col*12+numcol-1);
  Wire.write(OLED_CMD_SET_PAGE_RANGE);
  Wire.write(row);
  Wire.write(row);
  Wire.endTransmission();

  uint8_t off = 1-(row&1);
  const unsigned char* PROGMEM fdat = font + off;

  for(uint8_t i=0; i<sl; ++i)
  {
    Wire.beginTransmission(devaddr);
    Wire.write(OLED_CONTROL_BYTE_DATA_STREAM);
    unsigned char c = text[i];
    c -= 32;
    c = c > 96 ? 32 : c;
    const unsigned char* PROGMEM cdat = fdat + c * 24;
    uint8_t o = ( attr & 1 ) ? 0x80 : 0;
    for (uint8_t j=0; j<12; ++j)
    {
      unsigned char b = pgm_read_byte_near(cdat+j*2);
      Wire.write(b | o);
    }
    Wire.endTransmission();
  }
}


void oled_pattern(int devaddr, uint8_t pat, int16_t shift)
{
  Wire.beginTransmission(devaddr);
  Wire.write(OLED_CONTROL_BYTE_CMD_STREAM);
  Wire.write(OLED_CMD_SET_COLUMN_RANGE);
  Wire.write(0x00);
  Wire.write(0x7F);
  Wire.write(OLED_CMD_SET_PAGE_RANGE);
  Wire.write(0);
  Wire.write(0x07);
  Wire.endTransmission();
        
  for(uint8_t r=0; r<8; r++)
  {
    for(uint8_t c=0; c<128; c++)
    {
      if ( (c&15) == 0 )
      {
        Wire.beginTransmission(devaddr);
        Wire.write(OLED_CONTROL_BYTE_DATA_STREAM);
      }
      unsigned char b = pgm_read_byte_near(patterns[pat] + (c&15));
      Wire.write(b);
      if ( (c&15) == 15 )
          Wire.endTransmission();
    }
  }
}


void oled_write_strip(uint8_t devaddr, uint8_t row, uint8_t logoff, uint8_t valoff, const uint8_t* lo, const uint8_t* hi, uint8_t labeled )
{
  Wire.beginTransmission(devaddr);
  Wire.write(OLED_CONTROL_BYTE_CMD_STREAM);
  Wire.write(OLED_CMD_SET_COLUMN_RANGE);
  Wire.write(0x00);
  Wire.write(0x7F);
  Wire.write(OLED_CMD_SET_PAGE_RANGE);
  Wire.write(row);
  Wire.write(row);
  Wire.endTransmission();

  for(uint16_t c=0; c<128; c++)
  {
    if ( (c&15) == 0 )
    {
      Wire.beginTransmission(devaddr);
      Wire.write(OLED_CONTROL_BYTE_DATA_STREAM);
    }
    uint8_t b = 0;
    uint8_t idx = (logoff+c) & 0xff;
    if ( (logoff + c) % 12 == 0 )
      b |= 0x44;
    for ( int j=0; j<8; ++j )
    {
      uint8_t v = valoff + j;
      if ( v >= lo[idx] && v <= hi[idx] )
        b |= ( 0x80 >> j );
    }
    if ( labeled )
    {
      if ( row==2 )
      {
       if ( c < sizeof(label1900) )
         b = pgm_read_byte_near(label1900+c);
      }
      else if ( row == 7 )
      {
        if ( c < sizeof(label400) )
          b = pgm_read_byte_near(label400+c);
      }
    }
    Wire.write(b);
    if ( (c&15) == 15 )
      Wire.endTransmission();
  }
}


void oled_set_contrast( uint8_t devaddr, uint8_t contrast )
{
  Wire.beginTransmission( devaddr );    
  Wire.write(OLED_CONTROL_BYTE_CMD_STREAM);
  Wire.write(OLED_CMD_SET_CONTRAST);
  Wire.write(contrast); // 0x7F is maximum.  
  Wire.endTransmission();
}
