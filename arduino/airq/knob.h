
// Specifies which pins to use for CLK and DT.
extern void knob_setup
(
  uint8_t nr,
  uint8_t clkPin,   // Connected to CLK on KY-040
  uint8_t datPin,   // Connected to DT  on KY-040
  uint8_t swtPin    // Connected to SW  on KY-040
);


// Returns the delta to the encoder value. (-1, 0, 1)
extern int8_t  knob_update(uint8_t nr);

