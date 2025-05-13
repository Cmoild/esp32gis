#ifndef DEFINES_H
#define DEFINES_H

#define LCD_HOST               SPI2_HOST
#define LCD_H_RES              (320)
#define LCD_V_RES              (240)
#define LCD_BIT_PER_PIXEL      (16)

#define DASH_BAR_HEIGHT 80

#define PIN_NUM_LCD_CS         (GPIO_NUM_15)
#define PIN_NUM_LCD_PCLK       (GPIO_NUM_18)
#define PIN_NUM_LCD_DATA0      (GPIO_NUM_23)
#define PIN_NUM_LCD_DC         (GPIO_NUM_2)
#define PIN_NUM_LCD_RST        (GPIO_NUM_4)
#define PIN_NUM_LCD_BL         (GPIO_NUM_16)
#define PIN_NUM_TOUCH_CS       (GPIO_NUM_21)

#define DELAY_TIME_MS          (3000)

#define BUFFER_HEIGHT 4

#endif