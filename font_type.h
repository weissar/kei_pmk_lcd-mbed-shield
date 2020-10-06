/*
 * font_type.h
 *
 *  Created on: 4. 12. 2016
 *      Author: weiss_000
 */

#ifndef COMMON_FONT_TYPE_H_
#define COMMON_FONT_TYPE_H_

#define USE_FONT_HEADER_T

typedef struct
{
  uint16_t size;    //  font_Size_in_Bytes_over_all_included_Size_it_self;
  uint8_t  width;   //  font_Width_in_Pixel_for_fixed_drawing;
  uint8_t  height;  //  font_Height_in_Pixel_for_all_characters;
  uint8_t  first;   //  font_First_Char;
  uint8_t  count;   //  font_Char_Count;
//  uint8_t    font_Char_Widths[font_Last_Char - font_First_Char +1];
// for each character the separate width in pixels,
// characters < 128 have an implicit virtual right empty row
//  uint8_t    font_data[];
// bit field of all characters
} FONT_Header_t;

#endif /* COMMON_FONT_TYPE_H_ */
