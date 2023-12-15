/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "stdio.h"

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_inputmux.h"

#include "fsl_power.h"

#include "lcd.h"
#include "ov7670.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PINFUNC_CAMERA		15	/* Pin function for Camera engine to be configured by ARM CPU*/

#define EOL "\r\n"
#define GRAY_IMG_SIZE (28)
#define ZOOM (4)
#define FILL_MUL (3) //One point fills three points
#define RGB_SIZE (28*ZOOM)

#define CAMERA_ZOOM (8)  //image from camera is (28*8)*(28*8)

uint16_t g_image_rgb565[RGB_SIZE*RGB_SIZE] = {0};

//uint16_t g_image_buf[GRAY_IMG_SIZE*RGB_SIZE*GRAY_IMG_SIZE*RGB_SIZE] = {0};

uint8_t g_grayscale_img[GRAY_IMG_SIZE*GRAY_IMG_SIZE] = {0};

char object_list[10][16] = {
                        "T-shirt/top",
                        "Trouser",
                        "Pullover",
                        "Dress",
                        "Coat",
                        "Sandal",
                        "Shirt",
                        "Sneaker",
                        "Bag",
                        "Ankle boot"
};
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/**
 * @brief	camera initialization
 * @return	Nothing
 */
extern void Camera_Init(void);
/**
 * @brief	camera start
 * @return	Nothing
 */
extern void Camera_Start(void);
/*******************************************************************************
 * Code
 ******************************************************************************/
volatile uint32_t DataReadyFlag;
void Reserved46_IRQHandler(){
		DataReadyFlag = 1; // tell ARM data is ready from Camera engine
};
/*!
 * @brief Main function
 */
extern uint8_t img_data[];
extern uint8_t img_data_end[];
#define img_data_len (img_data_end - img_data)

int GetImage(void *ptr, uint32_t w, uint32_t h, uint32_t c){
	uint32_t len = w * h * c;
	memcpy(ptr, g_grayscale_img, len);

	return 0;
}

#define SYSTICK_PRESCALE 1U
#define TICK_PRIORITY 1U
void TIMER_Init(void)
{
    uint32_t priorityGroup = NVIC_GetPriorityGrouping();
    SysTick_Config(CLOCK_GetFreq(kCLOCK_CoreSysClk) / (SYSTICK_PRESCALE * 1000U));
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(priorityGroup, TICK_PRIORITY, 0U));
}

volatile uint32_t msTicks;
int64_t TIMER_GetTimeInUS(void)
{
    int64_t us = ((SystemCoreClock / 1000) - SysTick->VAL) / (SystemCoreClock / 1000000);
    us += msTicks * 1000;
    return us;
}

void SysTick_Handler(void)
{
    msTicks++;
}

extern status_t MODEL_Init(void);
extern uint8_t* MODEL_Run();
extern void MODEL_AllocateTensor(void* tensorArena, uint32_t size);
static uint32_t tensorArena[20 * 1024 / 4] __ALIGNED(16) = {0};

#define OUTPUT_DIMS 10

void display_grayscale_image(uint8_t* p_gray)
{
    uint32_t x_0,x_zoom,x_fill = 0;
    uint32_t y_0,y_zoom,y_fill = 0;
    uint32_t idx_zoom = 0;
    for(uint32_t idx = 0;idx < GRAY_IMG_SIZE*GRAY_IMG_SIZE; idx++)
    {
        x_0 = idx%GRAY_IMG_SIZE;
        y_0 = (idx - (idx%GRAY_IMG_SIZE))/GRAY_IMG_SIZE;
        x_zoom = x_0 *ZOOM;
        y_zoom = y_0 *ZOOM;
        idx_zoom = y_zoom * RGB_SIZE + x_zoom;
        g_image_rgb565[idx_zoom] = (p_gray[idx]>>2) << 5; //Green

        for(uint32_t fill_idx = 0; fill_idx < FILL_MUL; fill_idx++)
        {
            x_fill = x_zoom + fill_idx;
            for(uint32_t fill_y_idx = 0; fill_y_idx < FILL_MUL; fill_y_idx++)
            {
                y_fill = y_zoom + fill_y_idx;
                idx_zoom = y_fill * RGB_SIZE + x_fill;
                g_image_rgb565[idx_zoom] = (p_gray[idx]>>2) << 5; //Green
            }
        }
    }
    
    lcd_stream_win((240-RGB_SIZE)/2,50,(240-RGB_SIZE)/2 + RGB_SIZE-1,RGB_SIZE-1+50,g_image_rgb565,RGB_SIZE*RGB_SIZE);
}

void get_grayscale_img_from_camera_img(uint16_t* camera_img, uint8_t* grayscale_img)
{
    uint32_t x_camera, y_camera, idx_camera = 0;
    uint32_t x_gray, y_gray, idx_gray = 0;
    uint8_t R,G,B = 0;
    memset(grayscale_img,0,GRAY_IMG_SIZE*GRAY_IMG_SIZE);
    for(idx_gray = 0; idx_gray < GRAY_IMG_SIZE*GRAY_IMG_SIZE; idx_gray++)
    {
        x_gray = idx_gray%GRAY_IMG_SIZE;
        y_gray = (idx_gray - (idx_gray%GRAY_IMG_SIZE))/GRAY_IMG_SIZE;
        x_camera = x_gray * CAMERA_ZOOM;
        y_camera = y_gray * CAMERA_ZOOM;
        idx_camera = y_camera * GRAY_IMG_SIZE * CAMERA_ZOOM + x_camera;
//        grayscale_img[idx_gray] = ((camera_img[idx_camera] >> 5) << 2) & 0xFC;

//        grayscale_img[idx_gray] = 0xFF - grayscale_img[idx_gray];
//        
//        if(grayscale_img[idx_gray] < 0x80)
//        {
//            grayscale_img[idx_gray] = 0;
//        }
        R = ((camera_img[idx_camera] >> 11) & 0x1F) << 3;
        G = ((camera_img[idx_camera] >> 5) & 0x3F) << 2;
        B = (camera_img[idx_camera] & 0x1F) << 3;
        grayscale_img[idx_gray] = (uint8_t)((int)((R*30 + G*59 + B*11 + 50) / 100));
        grayscale_img[idx_gray] = 0xFF - grayscale_img[idx_gray];
        
        if(grayscale_img[idx_gray] < 0x60)
        {
            grayscale_img[idx_gray] = 0;
        }
    }
}

uint8_t lcd_printbuf[256] = {0};
int score = 0;
uint32_t last_object_idx = 0xff;
int main(void)
{
    char ch;

    /* Define the init structure for the output pin*/
    gpio_pin_config_t gpio_config = {
        kGPIO_DigitalOutput,
        1,
    };	

    /* Init board hardware. */
    /* set BOD VBAT level to 1.65V */
    POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
    /* attach main clock divide to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    
    CLOCK_EnableClock(kCLOCK_InputMux);                        /* Enables the clock for the kCLOCK_InputMux block. 0 = Disable; 1 = Enable.: 0x01u */
    CLOCK_EnableClock(kCLOCK_Iocon);                           /* Enables the clock for the IOCON block. 0 = Disable; 1 = Enable.: 0x01u */
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
		
    /* attach 12 MHz clock to FLEXCOMM4 (I2C master) */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);
    GPIO_PortInit(GPIO,0);
    /* reset FLEXCOMM for I2C */
    RESET_PeripheralReset(kFC4_RST_SHIFT_RSTn);		

    /* Init output GPIO. */
    GPIO_PortInit(GPIO, 1);
    GPIO_PinInit(GPIO, 1, 6, &gpio_config);
    CLOCK_AttachClk(kMAIN_CLK_to_CLKOUT);				//main clock source to clkout
    CLOCK_SetClkDiv(kCLOCK_DivClkOut,3,false);	//150MHz / 3 = 50MHz clkout to XCLK of camera module

    /* Connect trigger sources to camera engine */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX->CAMERA_ENGINE_INPUTMUX[0]	 =  13; // set p0_13 as VSYNC input function pin, every edge will be responded
    INPUTMUX->CAMERA_ENGINE_INPUTMUX[1]	 =  15; // set p0_15 as pixel input function pin, every edge will be responded
    /* Turnoff clock to inputmux to save power. Clock is only needed to make changes */
    INPUTMUX_Deinit(INPUTMUX);		
    // configure camera interface pins
    IOCON->PIO[0][0]  = PINFUNC_CAMERA | 1<<8|1<<10|2<<4| 1<<6;	//set p0_0 D0 on the camera port
    IOCON->PIO[0][1]  = PINFUNC_CAMERA | 1<<8|2<<4| 1<<6;				//set p0_1 D1 on the camera port
    IOCON->PIO[0][2]  = PINFUNC_CAMERA | 1<<8|2<<4| 1<<6;				//set p0_2 D2 on the camera port
    IOCON->PIO[0][3]  = PINFUNC_CAMERA | 1<<8|1<<6;							//set p0_3 D3 on the camera port
    IOCON->PIO[0][4]  = PINFUNC_CAMERA | 1<<8|1<<6;							//set p0_4 D4 on the camera port
    IOCON->PIO[0][5]  = PINFUNC_CAMERA | 1<<8|2<<4|1<<6;				//set p0_5 D5 on the camera port
    IOCON->PIO[0][6]  = PINFUNC_CAMERA | 1<<8;									//set p0_6 D6 on the camera port
    IOCON->PIO[0][7]  = PINFUNC_CAMERA | 1<<8;									//set p0_7 D7 on the camera port
    IOCON->PIO[0][18] = PINFUNC_CAMERA | 1<<8| 1<<10;						//P0_18 will toggle when camera engine receive every VSYNC dege

    /* configure P1_11 as output gpio for measuring the timing */
    GPIO_PortInit(GPIO, 1);
    GPIO_PinInit(GPIO, 1, 11, &gpio_config);
    /* configure P0_17 as output gpio for power up the camera moudule */
    GPIO_PortInit(GPIO, 0);
    GPIO_PinInit(GPIO, 0, 17, &gpio_config);
    GPIO_PinWrite(GPIO, 0, 17, 0);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    TIMER_Init();

    lcd_init(); // LCD initialization

    Ov7670_Init(0); // outside camera module initialization

    DisableIRQ(Reserved46_IRQn);    // Camera engine irq NUMBER 30
    CLOCK_EnableClock(kCLOCK_Ezhb); // enable camera engine clock
    Camera_Init();                  // camera engine initialization
    Camera_Start();                 // start camera engine
    EnableIRQ(Reserved46_IRQn);     // Camera engine irq NUMBER 30

    PRINTF("hello world.\r\n");

    uint16_t *g_CameraImageBuf = (uint16_t *)(0x20020000);
    MODEL_AllocateTensor((void *)tensorArena, sizeof(tensorArena));

    if (MODEL_Init() != kStatus_Success)
    {
        PRINTF("Failed initializing model" EOL);
        for (;;)
        {
        }
    }

    while (1)
    {

        while (DataReadyFlag == 0); // Waiting for respond from EZH, the waiting time is about 7ms

        GPIO_PinWrite(GPIO, 1, 11, 0); // toggle P1_11

        get_grayscale_img_from_camera_img(g_CameraImageBuf, g_grayscale_img);

        DataReadyFlag = 0; // clear the flag

        uint32_t start = TIMER_GetTimeInUS();
        uint8_t *output_data = MODEL_Run();
        uint32_t end = TIMER_GetTimeInUS();

        // find the max one
        int8_t max = -128;
        uint32_t idx = 0;
        for (int i = 0; i < OUTPUT_DIMS; i++)
        {
            int8_t value = ((int8_t *)output_data)[i];
            if (value >= max)
            {
                max = value;
                idx = i;
            }
        }
        sprintf((char *)lcd_printbuf, "Total time: %d ms, score %2d, idx %d", (end - start) / 1000, (int)((max + 128) / 255.0 * 100), idx);
        score = (int)((max + 128) / 255.0 * 100);
        if (last_object_idx != idx)
        {
            last_object_idx = idx;
            if (score > 50)
            {
                // PRINTF("Total time: %d ms, score %2d, idx %d\r\n", (end - start) / 1000, score, idx);
                lcd_clear_block(0, 248, LCD_COLOR_BLACK);
                lcd_clear_block(0, 260, LCD_COLOR_BLACK);
                PRINTF("%s\r\n", lcd_printbuf);
                // lcd_display_string(10, 180 + 70, lcd_printbuf, 12, 50712);
                // lcd_display_string(10, 160 + 70, "Object Recognition demo", 12, 50712);
                sprintf((char *)lcd_printbuf, "IT IS %s !", object_list[idx]);
                PRINTF("%s\r\n", lcd_printbuf); 
                // lcd_display_string(10, 200 + 70, lcd_printbuf, 16, 50712);

                set_lcd_window(0, 0, 240, 320);

                GPIO_PinWrite(GPIO, 1, 11, 1); // toggle P1_11
            }
            else
            {
                //                    lcd_clear_block(0, 230, LCD_COLOR_BLACK);
                lcd_clear_block(0, 248, LCD_COLOR_BLACK);
                //                    lcd_clear_block(0, 250, LCD_COLOR_BLACK);
                lcd_clear_block(0, 260, LCD_COLOR_BLACK);
            }
        }
        lcd_stream_win((240 - GRAY_IMG_SIZE * CAMERA_ZOOM) / 2, 0, (240 - GRAY_IMG_SIZE * CAMERA_ZOOM) / 2 + GRAY_IMG_SIZE * CAMERA_ZOOM - 1, GRAY_IMG_SIZE * CAMERA_ZOOM - 1, (uint16_t *)g_CameraImageBuf, GRAY_IMG_SIZE * CAMERA_ZOOM * GRAY_IMG_SIZE * CAMERA_ZOOM);
        set_lcd_window(0, 0, 240, 320);
    }
    return 0;
}
