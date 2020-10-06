/**
 * Required contants for accessing to other parts of app
 */
#include "mbed_shield_lcd_defines.h"
/**
 * Public function for using from main part of app
 */
#include "mbed_shield_lcd.h"

/**
 * Include common core function, must contain setGPIO, setAF, GPIOWrite
 */
#include _MBED_LCD_CORE_FUNCTIONS_H

#ifndef NULL
#define NULL  ((void *)0)
#endif

// #define USE_DMA_REFRESH
#define REFRESH_TIMER 4

/**
 * Differencies between platforms
 * !! not completed, can emit error
 */
#if defined(STM32F4)
#define SPI_IS_BUSY(SPIx) (((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))
//TODO #elseif another platform, for example - F103: SPIx->SR & SPI_SR_BSY
#else
#error Not supported platform
#endif

/**
 * Physical dimensions of LCD, controler can support 132x64 max
 */
#define _MBED_LCD_COLUMNS   128           ///< Horizontal pixels count
#define _MBED_LCD_ROWS      32            ///< Vertical pixels couns
#define _MBED_LCD_DATA_LINES   (_MBED_LCD_ROWS / 8)    ///< Count of 8px rows for LCD data

#if 1
#include "font_atari_8x8.h"       ///< Font defintion with header
static FONT_Header_t *ptrFontHeader = (FONT_Header_t *)font_atari_8x8_data;
#endif

#if 0
#include "font_thin_8x8.h"       ///< Font defintion with header
static FONT_Header_t *ptrFontHeader = (FONT_Header_t*)font_thin_8x8_data;
#endif

/**
 * Returns count of pixel horizontaly
 */
uint8_t MBED_LCD_GetColumns(void)
{
  return _MBED_LCD_COLUMNS;
}

/**
 * Return count of pixel verticaly
 */
uint8_t MBED_LCD_GetRows(void)
{
  return _MBED_LCD_ROWS;
}

/**
 * Returns count of text rows with included font
 */
uint8_t MBED_LCD_GetLines(void)
{
  return _MBED_LCD_ROWS / ptrFontHeader->height;
}

/**
 * Returns count of text columns with default fonr 8x8
 */
uint8_t MBED_LCD_GetCharPerLine(void)
{
  return _MBED_LCD_COLUMNS / ptrFontHeader->width;
}

/**
 *  Private memory buffer
 *  Reuqired LINES * COLUMNS bytes in BSS segment
 */
static uint8_t m_videoRam[_MBED_LCD_DATA_LINES * _MBED_LCD_COLUMNS];

/**
 * Private funcions
 */
static void MBED_LCD_send(uint8_t val, bool a0) ///< Write single value to LCD - using SPI, A0 selects CMD = 0, DATA = 1
{
  _MBED_LCD_NUCLEO_FN_GPIO_WR(_MBED_LCD_PIN_A0, a0 ? 1 : 0);
  _MBED_LCD_NUCLEO_FN_GPIO_WR(_MBED_LCD_PIN_CSN, 0);

  _MBED_LCD_SPI->DR = val;
  while (SPI_IS_BUSY(_MBED_LCD_SPI))
    // waiting is different fo F4xx and another Fxxx
    ;// blocking waiting

  _MBED_LCD_NUCLEO_FN_GPIO_WR(_MBED_LCD_PIN_CSN, 1);
}

#ifndef USE_DMA_REFRESH
static void MBED_LCD_sendData(uint8_t *val, uint16_t len) ///< Write block of data, pointer to start and length
{
  _MBED_LCD_NUCLEO_FN_GPIO_WR(_MBED_LCD_PIN_A0, 1);      // always data
  _MBED_LCD_NUCLEO_FN_GPIO_WR(_MBED_LCD_PIN_CSN, 0);

  for (; len; len--)
  {
    _MBED_LCD_SPI->DR = *val;
    val++;

    while (_MBED_LCD_SPI->SR & SPI_SR_BSY)
      // blocking wait
      ;
  }

  _MBED_LCD_NUCLEO_FN_GPIO_WR(_MBED_LCD_PIN_CSN, 1);
}
#endif

static bool MBED_LCD_reset(void)                     ///< Perform reset sequence
{
  uint16_t w, x = 0;

  _MBED_LCD_NUCLEO_FN_GPIO_WR(_MBED_LCD_PIN_A0, 0);
  _MBED_LCD_NUCLEO_FN_GPIO_WR(_MBED_LCD_PIN_RSTN, 0);

  for (w = 0; w < 10000; w++)
    x++;                              // Dummy increment prevents optimalisation

  _MBED_LCD_NUCLEO_FN_GPIO_WR(_MBED_LCD_PIN_RSTN, 1);

  for (w = 0; w < 1000; w++)
    x++;

  return (x > 0);                    // Trick to keep variable unoptimalised ...
}

static void MBED_LCD_set_start_line(uint8_t x) ///< Send command to LCD, info from DS
{
  MBED_LCD_send(0x10 | ((x & 0xf0) >> 4), 0); // (2) Display start line set = Sets the display RAM display start lineaddress - lower 6 bits
  MBED_LCD_send(0x00 | (x & 0x0f), 0); // (2) Display start line set = Sets the display RAM display start lineaddress - lower 6 bits
}

static void MBED_LCD_set_page(uint8_t p)  ///< Send command to LCD, info from DS
{
  MBED_LCD_send(0xB0 | (p & 0x0f), 0); // (3) Page address set = Sets the display RAM page address - lower 4 bits
}

static bool MBED_LCD_init_hw()                          ///< Init SPI, GPIO, ...
{
  _MBED_LCD_NUCLEO_FN_SET_GPIO(_MBED_LCD_PIN_RSTN,
      _MBED_LCD_NUCLEO_ENUM_OUT_PP);
  _MBED_LCD_NUCLEO_FN_GPIO_WR(_MBED_LCD_PIN_RSTN, 1);
  _MBED_LCD_NUCLEO_FN_SET_GPIO(_MBED_LCD_PIN_CSN, _MBED_LCD_NUCLEO_ENUM_OUT_PP);
  _MBED_LCD_NUCLEO_FN_GPIO_WR(_MBED_LCD_PIN_CSN, 1);
  _MBED_LCD_NUCLEO_FN_SET_GPIO(_MBED_LCD_PIN_A0, _MBED_LCD_NUCLEO_ENUM_OUT_PP);

  _MBED_LCD_NUCLEO_FN_SET_GPIO(_MBED_LCD_SPI_MOSI, _MBED_LCD_NUCLEO_ENUM_AF_PP);
  _MBED_LCD_NUCLEO_FN_SET_AF(_MBED_LCD_SPI_MOSI, _MBED_LCD_SPI_AF_NUM);  // AFxx
  _MBED_LCD_NUCLEO_FN_SET_GPIO(_MBED_LCD_SPI_SCK, _MBED_LCD_NUCLEO_ENUM_AF_PP);
  _MBED_LCD_NUCLEO_FN_SET_AF(_MBED_LCD_SPI_SCK, _MBED_LCD_SPI_AF_NUM);   // AFxx

  switch ((uint32_t) _MBED_LCD_SPI)
  // Switch reuired due using ohter APBx for some SPIx
  {
    case (uint32_t) SPI1:
      if (!(RCC->APB2ENR & RCC_APB2ENR_SPI1EN))
      {
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
      }
      break;
      //TODO other SPIx peripheral
    default:
      return false;
  }

  _MBED_LCD_SPI->CR1 = 0 | SPI_CR1_CPHA | SPI_CR1_CPOL     // polarity from DS
      | SPI_CR1_SSI | SPI_CR1_SSM       // required for correct function
      | SPI_CR1_MSTR;
  _MBED_LCD_SPI->CR2 = 0;

  {                                  // from DS - max clock 10MHz (100ns period)
    uint32_t apb2 = GetBusClock(busClockAPB2);
    uint32_t BRDiv = 0;                 // 000 = pclk / 2

    if (apb2 > 20e6)
      BRDiv = 0x01;      // 001 = pclk / 4
    if (apb2 > 40e6)
      BRDiv = 0x02;      // 010 = pclk / 8
    if (apb2 > 80e6)
      BRDiv = 0x03;      // 011 = pclk / 16

    _MBED_LCD_SPI->CR1 &= ~SPI_CR1_BR;
    //  only for F4xx:
    _MBED_LCD_SPI->CR1 |= (BRDiv & 0x07) << 3; // isolate 3 bits and set to bits 5..3
    //TODO another platforms
  }

  _MBED_LCD_SPI->CR1 |= SPI_CR1_SPE;             // enable

#ifdef USE_DMA_REFRESH
//  bbUseDMA = false;
  if (!(RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN))
  {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA2RST;
    RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMA2RST;
  }

  NVIC_EnableIRQ(DMA2_Stream3_IRQn);

#ifdef REFRESH_TIMER
  uint32_t apb = GetTimerClock(REFRESH_TIMER);
  if (apb != 0)
  {
    TIM_TypeDef *timPtr = NULL;
    IRQn_Type irqN = 0;
    switch(REFRESH_TIMER)
    {
      case 4:
        if (!(RCC->APB1ENR & RCC_APB1ENR_TIM4EN))
        {
          RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
          RCC->APB1RSTR |= RCC_APB1RSTR_TIM4RST;
          RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM4RST;
        }

        timPtr = TIM4;
        irqN = TIM4_IRQn;
        break;
        //TODO maji jine citace jine registry ?
      default:
        return false;
    }

    timPtr->CR1 = TIM_CR1_URS;
    timPtr->CR2 = 0;
    //    TIM3->EGR = TIM_EGR_UG;

    timPtr->PSC = apb / 100000 - 1;   // 100us = 10kHz
    timPtr->ARR = 500 - 1;             // reload 5ms (500 x 0.1ms)

    if (irqN > 0)
    {
      timPtr->DIER |= TIM_DIER_UIE;
      NVIC_EnableIRQ(irqN);
    }

    timPtr->CR1 |= TIM_CR1_CEN;
  }
#endif
  //  bbUseDMA = true;
#endif
  return true;
}

/**
 * Area of public functions
 */

/**
 * Fill videoram with value, bytes = columns, MSB on top
 */
void MBED_LCD_InitVideoRam(uint8_t val)
{
  for (int r = 0; r < _MBED_LCD_DATA_LINES; r++)
    for (int x = 0; x < _MBED_LCD_COLUMNS; x++)
      m_videoRam[r * _MBED_LCD_COLUMNS + x] = val;
}

/**
 * Initialisation - HW parts and init commands for LCD controller (see DS and MBED sample init code)
 * Returns false if ini fails
 */
bool MBED_LCD_init(void)
{
  if (!MBED_LCD_init_hw())  // check success of HW init
    return false;

  MBED_LCD_reset();

  MBED_LCD_send(0xAE, 0);   //  display off
  MBED_LCD_send(0xA2, 0);   //  bias voltage

  MBED_LCD_send(0xA0, 0);
  MBED_LCD_send(0xC8, 0);   //  colum normal

  MBED_LCD_send(0x22, 0);   //  voltage resistor ratio
  MBED_LCD_send(0x2F, 0);   //  power on
  //wr_cmd(0xA4);   //  LCD display ram
  MBED_LCD_send(0x40, 0);   // start line = 0
  MBED_LCD_send(0xAF, 0);     // display ON

  MBED_LCD_send(0x81, 0);   //  set contrast
  MBED_LCD_send(0x17, 0);   //  set contrast

  MBED_LCD_send(0xA6, 0);     // display normal
//  MBED_LCD_send(0xA7, 0);     // display inverted
//  MBED_LCD_send(0xa5, 0);

  return true;                // ALL init OK
}

/**
 * Writes 8x8 character at position in pixels
 * Return false if coordinates are outside working area
 */
bool MBED_LCD_WriteCharXY(char c, int colPix, int rowPix)
{
  int i, j;

  if (ptrFontHeader == NULL)
    return false;                // DISP_SetFont(-1);     // use default font

  if ((c >= (ptrFontHeader->first + ptrFontHeader->count))
      || (c < ptrFontHeader->first))
    return false;

  c -= ptrFontHeader->first;
  uint8_t *bp = (uint8_t *)ptrFontHeader + sizeof(FONT_Header_t) + (c * ptrFontHeader->width);

  if ((ptrFontHeader->height == 8)    // height 8px accesible by bytes
      && ((rowPix % 8) == 0))         // but only for starting on first row of disp data lines
  {
    for (i = 0; i < ptrFontHeader->width; i++)
    {
      m_videoRam[colPix + i + (rowPix / 8) * _MBED_LCD_COLUMNS] = bp[i];
      if ((colPix + i) >= _MBED_LCD_COLUMNS)    // overflow visible area ?
        break;
    }
  }
  else
  {
    if (ptrFontHeader->height > 8)
      return false;     //TODO fonts with higher then byte

    for (i = 0; i < ptrFontHeader->width; i++)
    {
      uint8_t b = bp[i];

      if (ptrFontHeader->height <= 8)
      {
        for (j = 0; j < ptrFontHeader->height; j++)
        {
          MBED_LCD_PutPixel(colPix + i, rowPix + j, (b & 1));    // LSB first
          b >>= 1;
        }

        continue;
      }
    }
  }

  return true;
}

/**
 * Writes string of 8x8 character at position in pixels
 * Return false if coordinates of first chracter are outside working area
 * TODO chceking max. length on line
 */
bool MBED_LCD_WriteStringXY(char *cp, int colPix, int rowPix)
{
  if ((colPix >= _MBED_LCD_COLUMNS) || (rowPix >= (_MBED_LCD_ROWS)))
    return false;

  for (; *cp; cp++)
  {
    MBED_LCD_WriteCharXY(*cp, colPix, rowPix);
    colPix += ptrFontHeader->width;
  }

  return true;
}

static volatile bool refreshInProgress = false;
static volatile int refreshDMAStage = -1;

#ifdef USE_DMA_REFRESH
uint8_t m_sendBuffer[_MBED_LCD_DATA_LINES * _MBED_LCD_COLUMNS];
#endif
/**
 * Copying content of videoRAM to LCD controller, based on SPI bulk transfer
 *
 * Duration ca 1.8ms without DMA when closk HSI 16MHz
 * 700us
 */
bool MBED_LCD_VideoRam2LCD(void)
{
  if (refreshInProgress)
    return false;

  refreshInProgress = true;
#ifdef USE_DMA_REFRESH
  DMA2_Stream3->CR &= ~DMA_SxCR_EN;

  // Writing 1 to these bits clears the corresponding flags in the DMA_LISR register
  DMA2->LIFCR = (DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3 | DMA_LIFCR_CDMEIF3);

  DMA2_Stream3->CR = 0
     | DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1  // 011 = channel 3 in stream 3
     | DMA_SxCR_DIR_1  // 10 = mem to mem = DMA_SxPAR to DMA_SxM0AR
     | DMA_SxCR_MINC
     | DMA_SxCR_PINC
     | DMA_SxCR_TCIE   // irq "complete" fire          DMA_CCR3_MINC
     ;

  DMA2_Stream3->PAR = (uint32_t)m_videoRam;       // SRC
  DMA2_Stream3->M0AR = (uint32_t)m_sendBuffer;          // DEST

  DMA2_Stream3->NDTR = _MBED_LCD_DATA_LINES * _MBED_LCD_COLUMNS;

  refreshDMAStage = -1;                            // start stage: MEM
  DMA2_Stream3->CR |= DMA_SxCR_EN;                    // go
#else
  for (uint8_t r = 0; r < _MBED_LCD_DATA_LINES; r++)
  {
    MBED_LCD_set_page(r);
    MBED_LCD_set_start_line(0);

#if 1
    MBED_LCD_sendData(m_videoRam + r * _MBED_LCD_COLUMNS, _MBED_LCD_COLUMNS); // block operation
#else
    for(uint8_t x = 0; x < _MBED_LCD_COLUMNS; x++)
      MBED_LCD_send(m_videoRam[r * _MBED_LCD_COLUMNS + x], 1);
#endif
  }

  refreshInProgress = false;
#endif

  return true;
}

#ifdef USE_DMA_REFRESH
void DMA2_Stream3_IRQHandler(void)
{
  if (DMA2->LISR & DMA_LISR_TCIF3)
  {
    DMA2->LIFCR = DMA_LIFCR_CTCIF3;   // only write 1 available

    if (refreshDMAStage >= 0)      // nebude se delat pro M2M, ktery je -1
    {
      while(SPI1->SR & SPI_SR_BSY)    // dokud se jeste odesila
        ;

      // musim cekat na doodeslani (viz. Figure 205. Transmission using DMA - pg.572/836 RM F411)
      // i kdyz posilam 132 bajtu, ale zobrazuje se pouze 128 (?)
      _MBED_LCD_NUCLEO_FN_GPIO_WR(_MBED_LCD_PIN_CSN, 1);
    }

    refreshDMAStage++;
    if (refreshDMAStage >= _MBED_LCD_DATA_LINES)
    {
      DMA2_Stream3->CR &= ~(DMA_SxCR_EN | DMA_SxCR_TCIE);   // stop and dis. irq

      SPI1->CR2 &= ~SPI_CR2_TXDMAEN;
      refreshInProgress = false;
    }
    else
    {
      DMA2_Stream3->CR &= ~DMA_SxCR_EN;
      DMA2->LIFCR |= (DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3 | DMA_LIFCR_CDMEIF3);

      SPI1->CR2 &= ~SPI_CR2_TXDMAEN;

      MBED_LCD_set_page(refreshDMAStage);
      MBED_LCD_set_start_line(0);

      DMA2_Stream3->CR = 0
        | DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1  // 011 = channel 3 in stream 3
        | DMA_SxCR_DIR_0  // 01 = mem to peripheral = DMA_SxM0AR to DMA_SxPAR
        | DMA_SxCR_MINC
        | DMA_SxCR_TCIE   // irq "complete" fire          DMA_CCR3_MINC
        ;

      DMA2_Stream3->PAR = (uint32_t)&(SPI1->DR);      // DEST
      DMA2_Stream3->M0AR = (uint32_t)&(m_sendBuffer[refreshDMAStage * _MBED_LCD_COLUMNS]);// SRC

      DMA2_Stream3->NDTR = _MBED_LCD_COLUMNS;

      SPI1->CR2 |= SPI_CR2_TXDMAEN;

      _MBED_LCD_NUCLEO_FN_GPIO_WR(_MBED_LCD_PIN_A0, 1);       // data transfer
      _MBED_LCD_NUCLEO_FN_GPIO_WR(_MBED_LCD_PIN_CSN, 0);

      DMA2_Stream3->CR |= DMA_SxCR_EN;        // go
    }
  }
}
#endif

#ifdef REFRESH_TIMER
#if (REFRESH_TIMER == 4)
void TIM4_IRQHandler(void)
{
  TIM4->SR = ~TIM_SR_UIF;  // see RM 15.4.5

  MBED_LCD_VideoRam2LCD();
}
//#elif
#else
//TODO  reagovat na to, ze to chybi
#error Invalid REFRESH_TIMER settings
#endif
#endif

/**
 * Puts pixel with color black = 1, background = 0
 * TODO check valied coordinates
 */
bool MBED_LCD_PutPixel(int x, int y, bool black)
{
  if ((x >= _MBED_LCD_COLUMNS) || (y >= _MBED_LCD_ROWS))
    return false;

  register uint32_t offset = _MBED_LCD_COLUMNS * (y / 8) + x; // !! division by 8 as first !!
  if (black)
    m_videoRam[offset] |= 1 << (y % 8);
  else
    m_videoRam[offset] &= ~(1 << (y % 8));
  return true;
}

/**
 * Draw line with color black = 1, background = 0
 * Coordinates x,y of start point a x,y of end point
 * Bresenham algorithm used (see rosetacode.org)
 * TODO check valied coordinates
 */
bool MBED_LCD_DrawLine(int x0, int y0, int x1, int y1, bool color)
{
  int dx = (x0 < x1) ? (x1 - x0) : (x0 - x1), sx = (x0 < x1) ? 1 : -1;
  int dy = (y0 < y1) ? (y1 - y0) : (y0 - y1), sy = (y0 < y1) ? 1 : -1;
  int err = ((dx > dy) ? dx : -dy) / 2, e2;

  for (;;)
  {
    if ((x0 == x1) && (y0 == y1))
      break;

    MBED_LCD_PutPixel((uint8_t) x0, (uint8_t) y0, color);

    e2 = err;
    if (e2 > -dx)
    {
      err -= dy;
      x0 += sx;
    }

    if (e2 < dy)
    {
      err += dx;
      y0 += sy;
    }
  }

  return true;
}

/**
 * Draw lines as rectangle with color black = 1, background = 0
 * TODO check valied coordinates
 */
bool MBED_LCD_DrawRect(int x, int y, int w, int h, bool color)
{
  MBED_LCD_DrawLine(x, y, x + w, y, color);
  MBED_LCD_DrawLine(x + w, y, x + w, y + h, color);
  MBED_LCD_DrawLine(x + w, y + h, x, y + h, color);
  MBED_LCD_DrawLine(x, y + h, x, y, color);

  return true;
}

/**
 * Draw lines as filled rectangle with color black = 1, background = 0
 * TODO check valied coordinates
 */
bool MBED_LCD_FillRect(int x, int y, int w, int h, bool color)
{
  int ww = w, xx = x;

  for (; h; h--)
  {
    x = xx;

    for (w = ww; w; w--)
    {
      MBED_LCD_PutPixel(x, y, color);
      x++;
    }

    y++;
  }

  return true;
}

/**
 * Draw circle with color black = 1, background = 0
 * Algorithm see rosetacode.org
 * Checks valied coordinates for clipping at display margins
 */
bool MBED_LCD_DrawCircle(int centerX, int centerY, int radius, bool colorSet)
{
  int d = (5 - radius * 4) / 4;
  int x = 0;
  int y = radius;

  do
  {
    // ensure index is in range before setting (depends on your image implementation)
    // in this case we check if the pixel location is within the bounds of the image before setting the pixel
    if (((centerX + x) >= 0) && ((centerX + x) <= (_MBED_LCD_COLUMNS - 1))
        && ((centerY + y) >= 0) && ((centerY + y) <= (_MBED_LCD_ROWS - 1)))
      MBED_LCD_PutPixel(centerX + x, centerY + y, colorSet);
    if (((centerX + x) >= 0) && ((centerX + x) <= (_MBED_LCD_COLUMNS - 1))
        && ((centerY - y) >= 0) && ((centerY - y) <= (_MBED_LCD_ROWS - 1)))
      MBED_LCD_PutPixel(centerX + x, centerY - y, colorSet);
    if (((centerX - x) >= 0) && ((centerX - x) <= (_MBED_LCD_COLUMNS - 1))
        && ((centerY + y) >= 0) && ((centerY + y) <= (_MBED_LCD_ROWS - 1)))
      MBED_LCD_PutPixel(centerX - x, centerY + y, colorSet);
    if (((centerX - x) >= 0) && ((centerX - x) <= (_MBED_LCD_COLUMNS - 1))
        && ((centerY - y) >= 0) && ((centerY - y) <= (_MBED_LCD_ROWS - 1)))
      MBED_LCD_PutPixel(centerX - x, centerY - y, colorSet);
    if (((centerX + y) >= 0) && ((centerX + y) <= (_MBED_LCD_COLUMNS - 1))
        && ((centerY + x) >= 0) && ((centerY + x) <= (_MBED_LCD_ROWS - 1)))
      MBED_LCD_PutPixel(centerX + y, centerY + x, colorSet);
    if (((centerX + y) >= 0) && ((centerX + y) <= (_MBED_LCD_COLUMNS - 1))
        && ((centerY - x) >= 0) && ((centerY - x) <= (_MBED_LCD_ROWS - 1)))
      MBED_LCD_PutPixel(centerX + y, centerY - x, colorSet);
    if (((centerX - y) >= 0) && ((centerX - y) <= (_MBED_LCD_COLUMNS - 1))
        && ((centerY + x) >= 0) && ((centerY + x) <= (_MBED_LCD_ROWS - 1)))
      MBED_LCD_PutPixel(centerX - y, centerY + x, colorSet);
    if (((centerX - y) >= 0) && ((centerX - y) <= (_MBED_LCD_COLUMNS - 1))
        && ((centerY - x) >= 0) && ((centerY - x) <= (_MBED_LCD_ROWS - 1)))
      MBED_LCD_PutPixel(centerX - y, centerY - x, colorSet);
    if (d < 0)
    {
      d += 2 * x + 1;
    }
    else
    {
      d += 2 * (x - y) + 1;
      y--;
    }
    x++;
  } while (x <= y);

  return true;
}

/**
 * Draw lines as filled circle with color black = 1, background = 0
 * TODO check valid coordinates
 */
bool MBED_LCD_FillCircle(int x0, int y0, int radius, bool color)
{
  int f = 1 - radius;
  int ddF_x = 0;
  int ddF_y = -2 * (int) radius;
  int x = 0;
  int y = (int) radius;
  int t1, t2;

  MBED_LCD_PutPixel(x0, y0 + radius, color);
  t1 = y0 - radius;
  MBED_LCD_PutPixel(x0, (t1 > 0) ? t1 : 0, color);
  t1 = x0 - radius;
  MBED_LCD_DrawLine(x0 + radius, y0, (t1 > 0) ? t1 : 0, y0, color);

  while (x < y)
  {
    if (f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x + 1;

    t1 = x0 - x;
    t2 = y0 - y;
    MBED_LCD_DrawLine((t1 > 0) ? t1 : 0, y0 + y, x0 + x, y0 + y, color);
    MBED_LCD_DrawLine((t1 > 0) ? t1 : 0, (t2 > 0) ? t2 : 0, x0 + x,
        (t2 > 0) ? t2 : 0, color);
    t1 = x0 - y;
    t2 = y0 - x;
    MBED_LCD_DrawLine((t1 > 0) ? t1 : 0, y0 + x, x0 + y, y0 + x, color);
    MBED_LCD_DrawLine((t1 > 0) ? t1 : 0, (t2 > 0) ? t2 : 0, x0 + y,
        (t2 > 0) ? t2 : 0, color);
  }

  return true;
}
