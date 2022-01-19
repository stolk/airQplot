
extern void oled_setup( int devaddr );

extern void oled_pattern( int devaddr, uint8_t pat, int16_t shift );

extern void oled_write_row( int devaddr, int row, int col, const char* text, uint8_t attr=0 );

extern void oled_set_contrast( uint8_t devaddr, uint8_t contrast );

extern void oled_write_strip(uint8_t devaddr, uint8_t row, uint8_t logoff, uint8_t valoff, const uint8_t* lo, const uint8_t* hi, uint8_t labeled );
