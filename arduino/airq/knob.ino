#include <inttypes.h>
#include <Arduino.h>

#define NUMENC 6
static uint8_t encClkPin[NUMENC]; // clock pin.
static uint8_t encDatPin[NUMENC]; // data pin.
static uint8_t encSwtPin[NUMENC]; // switch pin.
static uint8_t encClkVal[NUMENC]; // value of clock line.
static uint8_t encHist2 [NUMENC]; // two last values.
static uint8_t encHist4 [NUMENC]; // four last values.



void knob_setup
(
  uint8_t nr,
  uint8_t clkPin,   // Connected to CLK on KY-040
  uint8_t datPin,   // Connected to DT  on KY-040
  uint8_t swtPin    // Connected to SW  on KY-040
)
{
  encClkPin[nr] = clkPin;
  encDatPin[nr] = datPin;
  encSwtPin[nr] = swtPin;
  pinMode(clkPin, INPUT_PULLUP);
  pinMode(datPin, INPUT_PULLUP);
  pinMode(swtPin, INPUT_PULLUP);
  encClkVal[nr] = digitalRead(clkPin);
}

static const uint8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};
static uint16_t store=0;

// A valid CW or CCW move returns 1, -1, invalid returns 0.
static int8_t read_rotary(int nr)
{
  const uint8_t clk = encClkPin[nr];
  const uint8_t dat = encDatPin[nr];
  encHist2[nr] <<= 2;
  if (digitalRead(clk)) encHist2[nr] |= 0x02;
  if (digitalRead(dat)) encHist2[nr] |= 0x01;
  encHist2[nr] &= 0x0f;

  // If valid then store as 16 bit data.
  if  (rot_enc_table[encHist2[nr]] )
  {
    encHist4[nr] <<= 4;
    encHist4[nr] |= encHist2[nr];
    if ((encHist4[nr] & 0xff)==0x2b) return  1;
    if ((encHist4[nr] & 0xff)==0x17) return -1;
  }
  return 0;
}


int8_t knob_update(uint8_t nr)
{
  const int8_t delta = read_rotary(nr);
  return delta;
}

