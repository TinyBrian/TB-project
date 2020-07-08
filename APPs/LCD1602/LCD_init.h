#ifndef __LCD_init_H
#define __LCD_init_H
#include "Delay.h"
#include "Bit_ADDR.h"
#include "IIC_init.h"
#include "ADC_init.h"



/**1. Clear display
 * 
 * - Execution time: 1.52ms
 * RS|R/W | DB7|DB6|DB5|DB4|DB3|DB2|DB1|DB0
 *  0   0     0   0   0   0   0   0  0   1
 */


/**2. Return home
 * 
 * - Execution time: 1.52ms
 * RS|R/W | DB7|DB6|DB5|DB4|DB3|DB2|DB1|DB0
 *  0   0     0   0   0   0   0   0   1   -
 */



/**3. Entry mode set
 * 
 * - Execution time: 38us
 * RS|R/W | DB7|DB6|DB5|DB4|DB3|DB2|DB1|DB0
 *  0   0     0   0   0   0   0   1 I/D  SH
 *                                    |    `- Shift of entire display: 1 - shift
 *                                    `- Increment/decrement of DDRAM address: 0 - dec, 1 - inc
 */


/**4. Display on/off control:(光标+显示开关)
 * 
 * - Execution time: 38us
 * RS|R/W | DB7|DB6|DB5|DB4|DB3|DB2|DB1|DB0
 *  0   0     0   0   0   0   1   D   C   B
 *                                |   |   `- Cursor Blink ON/OFF: 0 - off, 1 - on
 *                                |   `- Cursor ON/OFF:  0 - off, 1 - on
 *                                `- Display ON/OFF: 0 - off, 1 - on
 */


/**5. Cursor or display shift(显示左右移)
 * 
 * - Execution time: 38us
 * RS|R/W | DB7|DB6|DB5|DB4|DB3|DB2|DB1|DB0
 *  0   0     0   0   0   1 S/C R/L   -   -
 *                            0   0 - Shift cursor to the left, AC is decreased by 1
 *                            0   1 - Shift cursor to the right, AC is increased by 1
 *                            1   0 - Shift all the display to the left, cursor moves according to the display
 *                            1   1 - Shift all the display to the right, cursor moves according to the display
 */


/**6. Function set:(功能设置：一个字符多少点，显示行数，4位模式，8位模式
 * )
 * - Execution time: 38us
 * RS|R/W | DB7|DB6|DB5|DB4|DB3|DB2|DB1|DB0
 *  0   0     0   0   1  DL   N   F   -   -
 *                        |   |   `- Display font type: 0 - 5x8 dots, 1 - 5x11 dots
 *                        |   `- Display line number: 0 - 1-line, 1 - 2-line
 *                        `- Interface data length: 0 - 4-bit, 1 - 8 bit
 */


/** 7. Set CGRAM address(用户编程地址)
 *
 * - Execution time: 38us
 * RS|R/W | DB7|DB6|DB5|DB4|DB3|DB2|DB1|DB0
 *  0   0     0   1 AC5 AC4 AC3 AC2 AC1 AC0
 */


/**8. Set DDRAM address(设置显示字符地址)
 * 
 * - Execution time: 38us
 * RS|R/W | DB7|DB6|DB5|DB4|DB3|DB2|DB1|DB0
 *  0   0     1 AC6 AC5 AC4 AC3 AC2 AC1 AC0
 */


/**9. Read busy flag & address
 * 
 * - Execution time: 0us
 * RS|R/W | DB7|DB6|DB5|DB4|DB3|DB2|DB1|DB0
 *  0   1    BF AC6 AC5 AC4 AC3 AC2 AC1 AC0
 *           `- busy flag: 0 - idle, 1 - busy
 */

/**10. Write data from RAM
 * 
 * - Execution time: 38us
 * RS|R/W | DB7|DB6|DB5|DB4|DB3|DB2|DB1|DB0
 *  1   0    D7  D6  D5  D4  D3  D2  D1  D0
 */

/**11. Read data from RAM
 * 
 * - Execution time: 38us
 * RS|R/W | DB7|DB6|DB5|DB4|DB3|DB2|DB1|DB0
 *  1   1    D7  D6  D5  D4  D3  D2  D1  D0
 */

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

#define LCD_SALVE 1
#define LCD_MASTER 0
#define LCD4bit 1
#define LCD8bit 0
#define LCD1602_SALVE_ADDR          (0x3f<<1)

#if(LCD4bit)

#define LCD_4_Bit  0x20

#endif

#if(LCD8bit)

// commands
#define LCD_CLEARDISPLAY 0x01   			// clear display
#define LCD_RETURNHOME 0x02      			//return home 
#define LCD_ENTRYMODESET 0x04      		//entry mode set
#define LCD_DISPLAYCONTROL 0x08     	//display control
#define LCD_CURSORSHIFT 0x10     			//cursor shift
#define LCD_FUNCTIONSET 0x20     			//function set
#define LCD_SETCGRAMADDR 0x40    			 //set CGRAMADDR
#define LCD_SETDDRAMADDR 0x80     		// set DDRAMADDR

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00     					//entry right  ->
#define LCD_ENTRYLEFT 0x02     						//entrl left <-
#define LCD_ENTRYSHIFTINCREMENT 0x01     //entry shift increment+
#define LCD_ENTRYSHIFTDECREMENT 0x00     //entry shift decrement-

// flags for display on/off control
#define LCD_DISPLAYON 0x04     					// display on
#define LCD_DISPLAYOFF 0x00   			  //display off
#define LCD_CURSORON 0x02     			//cursor on
#define LCD_CURSOROFF 0x00    			 //cursor off
#define LCD_BLINKON 0x01     					//blink on
#define LCD_BLINKOFF 0x00     				//blink off

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08     //display move
#define LCD_CURSORMOVE 0x00     //cursor move
#define LCD_MOVERIGHT 0x04     //move right
#define LCD_MOVELEFT 0x00     //move left

// flags for function set
#define LCD_8BITMODE 0x10     // 8bit mode
#define LCD_4BITMODE 0x00     // 4bit mode
#define LCD_2LINE 0x08     // 2 line
#define LCD_1LINE 0x00     // 1 line
#define LCD_5x10DOTS 0x04     //50 dot 
#define LCD_5x8DOTS 0x00     //40 dot

// flags for backlight control
#define LCD_BACKLIGHT 0x08     //backlight on
#define LCD_NOBACKLIGHT 0x00     //backlight off

#define En 0x04  // Enable bit
#define Rw 0x02  // Read/Write bit
#define Rs 0x01  // Register select bit


#endif




void LCD_Return_data(char data);
void LCD_Receive_command(u8 command);

void LCD_init(void);
void LCD_setCursor(u8 row,u8 col);  //设置光标位置

void LCD_Sent_status(void);
void LCD_clear(void );
void LCD_backlight(status Newstatus);

void LCD_scrollDisplayLeft(void);
void LCD_scrollDisplayRight(void);
void LCD_autoscroll(void);


int LCD_Check_busy(void);

void lcd_send_string (char *str);
void lcd_send_char (char str);

void lcd_send_string_t(char *str,int i);




//////////////display software//////////////
void LCD_Startpage(void);
void LCD_ADC_percent(void);
void LCD_MPU6050_u8(u8* pdata,u8 length);

#endif

