//	MIT License
//
//	Copyright (c) 2017 Matej Artnak
//
//	Permission is hereby granted, free of charge, to any person obtaining a copy
//	of this software and associated documentation files (the "Software"), to deal
//	in the Software without restriction, including without limitation the rights
//	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//	copies of the Software, and to permit persons to whom the Software is
//	furnished to do so, subject to the following conditions:
//
//	The above copyright notice and this permission notice shall be included in all
//	copies or substantial portions of the Software.
//
//	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//	SOFTWARE.
//
//
//
//-----------------------------------
//	ILI9341 GFX library for STM32
//-----------------------------------
//
//	Very simple GFX library built upon ILI9342_STM32_Driver library.
//	Adds basic shapes, image and font drawing capabilities to ILI9341
//
//	Library is written for STM32 HAL library and supports STM32CUBEMX. To use the library with Cube software
//	you need to tick the box that generates peripheral initialization code in their own respective .c and .h file
//
//
//-----------------------------------
//	How to use this library
//-----------------------------------
//
//	-If using MCUs other than STM32F7 you will have to change the #include "stm32f7xx_hal.h" in the ILI9341_GFX.h to your respective .h file
//
//	If using "ILI9341_STM32_Driver" then all other prequisites to use the library have allready been met
//	Simply include the library and it is ready to be used
//
//-----------------------------------


#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include "5x5_font.h"
#include "spi.h"

#define CHECK_BIT(var,pos) (((var)>>(pos)) & 1)

/*Draw hollow circle at X,Y location with specified radius and colour. X and Y represent circles center */
void ILI9341_Draw_Hollow_Circle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Colour)
{
	int x = Radius-1;
    int y = 0;
    int dx = 1;
    int dy = 1;
    int err = dx - (Radius << 1);

    while (x >= y)
    {
        ILI9341_Draw_Pixel(X + x, Y + y, Colour);
        ILI9341_Draw_Pixel(X + y, Y + x, Colour);
        ILI9341_Draw_Pixel(X - y, Y + x, Colour);
        ILI9341_Draw_Pixel(X - x, Y + y, Colour);
        ILI9341_Draw_Pixel(X - x, Y - y, Colour);
        ILI9341_Draw_Pixel(X - y, Y - x, Colour);
        ILI9341_Draw_Pixel(X + y, Y - x, Colour);
        ILI9341_Draw_Pixel(X + x, Y - y, Colour);

        if (err <= 0)
        {
            y++;
            err += dy;
            dy += 2;
        }
        if (err > 0)
        {
            x--;
            dx += 2;
            err += (-Radius << 1) + dx;
        }
    }
}

/*Draw filled circle at X,Y location with specified radius and colour. X and Y represent circles center */
void ILI9341_Draw_Filled_Circle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Colour)
{
	
		int x = Radius;
    int y = 0;
    int xChange = 1 - (Radius << 1);
    int yChange = 0;
    int radiusError = 0;

    while (x >= y)
    {
        for (int i = X - x; i <= X + x; i++)
        {
            ILI9341_Draw_Pixel(i, Y + y,Colour);
            ILI9341_Draw_Pixel(i, Y - y,Colour);
        }
        for (int i = X - y; i <= X + y; i++)
        {
            ILI9341_Draw_Pixel(i, Y + x,Colour);
            ILI9341_Draw_Pixel(i, Y - x,Colour);
        }

        y++;
        radiusError += yChange;
        yChange += 2;
        if (((radiusError << 1) + xChange) > 0)
        {
            x--;
            radiusError += xChange;
            xChange += 2;
        }
    }
		//Really slow implementation, will require future overhaul
		//TODO:	https://stackoverflow.com/questions/1201200/fast-algorithm-for-drawing-filled-circles	
}

/*Draw a hollow rectangle between positions X0,Y0 and X1,Y1 with specified colour*/
void ILI9341_Draw_Hollow_Rectangle_Coord(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Colour)
{
	uint16_t 	X_length = 0;
	uint16_t 	Y_length = 0;
	uint8_t		Negative_X = 0;
	uint8_t 	Negative_Y = 0;
	float 		Calc_Negative = 0;
	
	Calc_Negative = X1 - X0;
	if(Calc_Negative < 0) Negative_X = 1;
	Calc_Negative = 0;
	
	Calc_Negative = Y1 - Y0;
	if(Calc_Negative < 0) Negative_Y = 1;
	
	
	//DRAW HORIZONTAL!
	if(!Negative_X)
	{
		X_length = X1 - X0;		
	}
	else
	{
		X_length = X0 - X1;		
	}
	ILI9341_Draw_Horizontal_Line(X0, Y0, X_length, Colour);
	ILI9341_Draw_Horizontal_Line(X0, Y1, X_length, Colour);
	
	
	
	//DRAW VERTICAL!
	if(!Negative_Y)
	{
		Y_length = Y1 - Y0;		
	}
	else
	{
		Y_length = Y0 - Y1;		
	}
	ILI9341_Draw_Vertical_Line(X0, Y0, Y_length, Colour);
	ILI9341_Draw_Vertical_Line(X1, Y0, Y_length, Colour);
	
	if((X_length > 0)||(Y_length > 0)) 
	{
		ILI9341_Draw_Pixel(X1, Y1, Colour);
	}
	
}

/*Draw a filled rectangle between positions X0,Y0 and X1,Y1 with specified colour*/
void ILI9341_Draw_Filled_Rectangle_Coord(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Colour)
{
	uint16_t 	X_length = 0;
	uint16_t 	Y_length = 0;
	uint8_t		Negative_X = 0;
	uint8_t 	Negative_Y = 0;
	int32_t 	Calc_Negative = 0;
	
	uint16_t X0_true = 0;
	uint16_t Y0_true = 0;
	
	Calc_Negative = X1 - X0;
	if(Calc_Negative < 0) Negative_X = 1;
	Calc_Negative = 0;
	
	Calc_Negative = Y1 - Y0;
	if(Calc_Negative < 0) Negative_Y = 1;
	
	
	//DRAW HORIZONTAL!
	if(!Negative_X)
	{
		X_length = X1 - X0;
		X0_true = X0;
	}
	else
	{
		X_length = X0 - X1;
		X0_true = X1;
	}
	
	//DRAW VERTICAL!
	if(!Negative_Y)
	{
		Y_length = Y1 - Y0;
		Y0_true = Y0;		
	}
	else
	{
		Y_length = Y0 - Y1;
		Y0_true = Y1;	
	}
	
	ILI9341_Draw_Rectangle(X0_true, Y0_true, X_length, Y_length, Colour);	
}

/*Draws a character (fonts imported from fonts.h) at X,Y location with specified font colour, size and Background colour*/
/*See fonts.h implementation of font on what is required for changing to a different font when switching fonts libraries*/
void ILI9341_Draw_Char(char Character, uint16_t X, uint8_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
		uint8_t 	function_char;
    uint8_t 	i,j;
		
		function_char = Character;
		
    if (function_char < ' ') {
        Character = 0;
    } else {
        function_char -= 32;
		}
   	
		char temp[CHAR_WIDTH];
		for(uint8_t k = 0; k<CHAR_WIDTH; k++)
		{
		temp[k] = font[function_char][k];
		}
		
    // Draw pixels
		ILI9341_Draw_Rectangle(X-2, Y, CHAR_WIDTH*Size, CHAR_HEIGHT*Size, Background_Colour);
    for (j=0; j<CHAR_WIDTH; j++) {
        for (i=0; i<CHAR_HEIGHT; i++) {
            if (temp[j] & (1<<i)) {			
							if(Size == 1)
							{
              ILI9341_Draw_Pixel(X+j, Y+i, Colour);
							}
							else
							{
							ILI9341_Draw_Rectangle(X+(j*Size), Y+(i*Size), Size, Size, Colour);
							}
            }						
        }
    }
}

/*Draws an array of characters (fonts imported from fonts.h) at X,Y location with specified font colour, size and Background colour*/
/*See fonts.h implementation of font on what is required for changing to a different font when switching fonts libraries*/
void ILI9341_Draw_Text(const char* Text, uint16_t X, uint8_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
    while (*Text) {
        ILI9341_Draw_Char(*Text++, X, Y, Colour, Size, Background_Colour);
        if(Size==1){
        	X += CHAR_WIDTH*Size+1;
        }
        else{
        X += CHAR_WIDTH*Size;
        }
    }
}

/*Draws a full screen picture from flash. Image converted from RGB .jpeg/other to C array using online converter*/
//USING CONVERTER: http://www.digole.com/tools/PicturetoC_Hex_converter.php
//65K colour (2Bytes / Pixel)
void ILI9341_Draw_Image(const char* Image_Array, uint8_t Orientation,uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2)
{
	if(Orientation == SCREEN_HORIZONTAL_1)
	{
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
		ILI9341_Set_Address(X1,Y1,X2,Y2);
			
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		
		unsigned char Temp_small_buffer[BURST_MAX_SIZE];
		uint32_t counter = 0;
		for(uint32_t i = 0; i < (X2-X1)*(Y2-Y1)/BURST_MAX_SIZE; i++)
		{			
				for(uint32_t k = 0; k< BURST_MAX_SIZE; k++)
				{
					Temp_small_buffer[k]	= Image_Array[counter+k];		
				}						
				HAL_SPI_Transmit(&hspi1, (unsigned char*)Temp_small_buffer, BURST_MAX_SIZE, 10);
				counter += BURST_MAX_SIZE;			
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	}
	else if(Orientation == SCREEN_HORIZONTAL_2)
	{
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
		ILI9341_Set_Address(X1,Y1,X2,Y2);
			
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		
		unsigned char Temp_small_buffer[BURST_MAX_SIZE];
		uint32_t counter = 0;
		for(uint32_t i = 0; i < (X2-X1)*(Y2-Y1)/BURST_MAX_SIZE; i++)
		{			
				for(uint32_t k = 0; k< BURST_MAX_SIZE; k++)
				{
					Temp_small_buffer[k]	= Image_Array[counter+k];
				}						
				HAL_SPI_Transmit(&hspi1, (unsigned char*)Temp_small_buffer, BURST_MAX_SIZE, 10);
				counter += BURST_MAX_SIZE;			
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	}
	else if(Orientation == SCREEN_VERTICAL_2)
	{
		ILI9341_Set_Rotation(SCREEN_VERTICAL_2);
		ILI9341_Set_Address(0,0,ILI9341_SCREEN_HEIGHT,ILI9341_SCREEN_WIDTH);
			
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		
		unsigned char Temp_small_buffer[BURST_MAX_SIZE];
		uint32_t counter = 0;
		for(uint32_t i = 0; i < ILI9341_SCREEN_WIDTH*ILI9341_SCREEN_HEIGHT*2/BURST_MAX_SIZE; i++)
		{			
				for(uint32_t k = 0; k< BURST_MAX_SIZE; k++)
				{
					Temp_small_buffer[k]	= Image_Array[counter+k];		
				}						
				HAL_SPI_Transmit(&hspi1, (unsigned char*)Temp_small_buffer, BURST_MAX_SIZE, 10);
				counter += BURST_MAX_SIZE;			
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	}
	else if(Orientation == SCREEN_VERTICAL_1)
	{
		ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
		ILI9341_Set_Address(0,0,ILI9341_SCREEN_HEIGHT,ILI9341_SCREEN_WIDTH);
			
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		
		unsigned char Temp_small_buffer[BURST_MAX_SIZE];
		uint32_t counter = 0;
		for(uint32_t i = 0; i < ILI9341_SCREEN_WIDTH*ILI9341_SCREEN_HEIGHT*2/BURST_MAX_SIZE; i++)
		{			
				for(uint32_t k = 0; k< BURST_MAX_SIZE; k++)
				{
					Temp_small_buffer[k]	= Image_Array[counter+k];		
				}						
				HAL_SPI_Transmit(&hspi1, (unsigned char*)Temp_small_buffer, BURST_MAX_SIZE, 10);
				counter += BURST_MAX_SIZE;			
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	}
}

void ILI9341_DrawBitmap(const char* Image_Array,uint16_t X1, uint16_t Y1, uint16_t Width, uint16_t Height,uint16_t Background_Color){

	ILI9341_Set_Address(X1,Y1,X1+Width-1,Y1+Height);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	unsigned char Temp_small_buffer[BURST_MAX_SIZE];
			uint32_t counter = 0;
			for(uint32_t i = 0; i < (Width*Height*2)/BURST_MAX_SIZE; i++)
			{
					for(uint32_t k = 0; k< BURST_MAX_SIZE; k+=2)
					{
							if((Image_Array[(counter+k)/16])&1<<(7-((k/2)%8))){
								Temp_small_buffer[k]=Background_Color>>8;
								Temp_small_buffer[k+1]=Background_Color;
							}
							else{
								Temp_small_buffer[k]=0xFF;
								Temp_small_buffer[k+1]=0xFF;
							}
					}
					HAL_SPI_Transmit_DMA(&hspi1, (unsigned char*)Temp_small_buffer, BURST_MAX_SIZE);
					counter += BURST_MAX_SIZE;

			}
			uint32_t Remainder_from_block = ((Width*Height*2))%BURST_MAX_SIZE;
			unsigned char temp_remainder[Remainder_from_block];
			for(uint32_t l=0;l<Remainder_from_block;l++){
				for(uint32_t k = 0; k< Remainder_from_block; k+=2)
									{
											if((Image_Array[(counter+k)/16])&1<<(7-((k/2)%8))){
												temp_remainder[k]=Background_Color>>8;
												temp_remainder[k+1]=Background_Color;
											}
											else{
												temp_remainder[k]=0xFF;
												temp_remainder[k+1]=0xFF;
											}
									}
			}
			HAL_SPI_Transmit(&hspi1, (unsigned char *)temp_remainder, Remainder_from_block,1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}


