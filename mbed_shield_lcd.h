#ifndef MBED_SHIELD_LCD_H_
#define MBED_SHIELD_LCD_H_

/**
 * Preparation global symbols - bool and integer types
 */
#ifndef bool
#include <stdbool.h>
#endif

#ifndef uint8_t
#include <stdint.h>
#endif

void MBED_LCD_InitVideoRam(uint8_t val);      ///< Fill all Video RAM by value (bytes = columns, MSB on top)
bool MBED_LCD_VideoRam2LCD();                 ///< Copy Video RAM content to LCD using SPI

bool MBED_LCD_init(void);                     ///< signal initialization, RESET, first init commands

uint8_t MBED_LCD_GetColumns(void);            ///< Number of pixels horizontaly
uint8_t MBED_LCD_GetRows(void);               ///< Number of pixels verticaly
uint8_t MBED_LCD_GetLines(void);              ///< Number of text lines (hor. pix / 8)
uint8_t MBED_LCD_GetCharPerLine(void);        ///< Number of characters (vert. pix / 8)

bool MBED_LCD_WriteCharXY(char c, int colPix, int rowPix);      ///< Write 8x8 char to position counted in chars
bool MBED_LCD_WriteStringXY(char *cp, int colPix, int rowPix);  ///< Write sequence of8x8 chars to position counted in chars
bool MBED_LCD_PutPixel(int x, int y, bool black);         ///< Put pixel - 1 = black, 0 = white (background)

bool MBED_LCD_DrawLine(int x0, int y0, int x1, int y1, bool color);
bool MBED_LCD_DrawRect(int x, int y, int w, int h, bool color);
bool MBED_LCD_FillRect(int x, int y, int w, int h, bool color);
bool MBED_LCD_DrawCircle(int centerX, int centerY, int radius, bool colorSet);
bool MBED_LCD_FillCircle(int x0, int y0, int radius, bool color);

#endif /* MBED_SHIELD_LCD_H_ */
