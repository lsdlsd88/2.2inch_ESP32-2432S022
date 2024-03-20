#include <Arduino.h>
#include <lvgl.h>
#include <demos/lv_demos.h>
#include "CST820.h"
#define GFX
//如包含触摸解除下面行的注释chaduo
//#define TOUCH 

/*更改屏幕分辨率*/
#define TFT_WIDTH 240
#define TFT_HEIGHT 320

/*定义触摸屏引脚*/
#define I2C_SDA 21
#define I2C_SCL 22

#ifdef TOUCH
CST820 touch(I2C_SDA, I2C_SCL, -1, -1); /* 触摸实例 */
#endif

static lv_disp_draw_buf_t draw_buf;
// static lv_color_t buf[TFT_WIDTH * 100];
lv_indev_t *myInputDevice;

#if defined(GFX)
#define LGFX_USE_V1      // set to use new version of library
#include <LovyanGFX.hpp> // main library

class LGFX : public lgfx::LGFX_Device
{

    lgfx::Panel_ST7789 _panel_instance; // ST7789UI
    lgfx::Bus_Parallel8 _bus_instance;  // MCU8080 8B

public:
    LGFX(void)
    {
        {
            auto cfg = _bus_instance.config();
            cfg.freq_write = 25000000;
            cfg.pin_wr = 4;
            cfg.pin_rd = 2;
            cfg.pin_rs = 16;

            cfg.pin_d0 = 15;
            cfg.pin_d1 = 13;
            cfg.pin_d2 = 12;
            cfg.pin_d3 = 14;
            cfg.pin_d4 = 27;
            cfg.pin_d5 = 25;
            cfg.pin_d6 = 33;
            cfg.pin_d7 = 32;

            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        {
            auto cfg = _panel_instance.config();

            cfg.pin_cs = 17;
            cfg.pin_rst = -1;
            cfg.pin_busy = -1;

            cfg.panel_width = 240;
            cfg.panel_height = 320;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.offset_rotation = 0;
            // cfg.dummy_read_pixel = 8;
            // cfg.dummy_read_bits = 1;
            cfg.readable = false;
            cfg.invert = false;
            cfg.rgb_order = false;
            cfg.dlen_16bit = false;
            cfg.bus_shared = true;

            _panel_instance.config(cfg);
        }

        setPanel(&_panel_instance);
    }
};

static LGFX tft; // declare display variable

#endif

#if defined(GFX)
/*
    lcd interface
    transfer pixel data range to lcd
*/
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    int w = (area->x2 - area->x1 + 1);
    int h = (area->y2 - area->y1 + 1);

    tft.startWrite();                            /* Start new TFT transaction */
    tft.setAddrWindow(area->x1, area->y1, w, h); /* set the working window */
    tft.writePixels(&color_p->full, w * h,false);//true

    tft.endWrite();            /* terminate TFT transaction */
    lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

#ifdef TOUCH
/*读取触摸板*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{

    bool touched;
    uint8_t gesture;
    uint16_t touchX, touchY;

    touched = touch.getTouch(&touchX, &touchY, &gesture);

    if (!touched)
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;
    }
}
#endif
#endif


void setup()
{
    Serial.begin(115200);
    pinMode(0, OUTPUT);
    digitalWrite(0, HIGH);

#ifdef TOUCH
    touch.begin(); /*初始化触摸板*/
#endif

#if defined(GFX)
    lv_init();
    // lv_img_cache_set_size(10);
#if LV_USE_LOG != 0
    lv_log_register_print_cb(my_print); // register print function for debugging
#endif

    tft.init();
    tft.fillScreen(TFT_BLACK);

    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(TFT_HEIGHT * 150 * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(TFT_HEIGHT * 150 * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, TFT_HEIGHT * 120);

    // initialise the display
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    // settings for display driver
    disp_drv.hor_res = TFT_WIDTH;
    disp_drv.ver_res = TFT_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

#ifdef TOUCH
    /*初始化（虚拟）输入设备驱动程序*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);
#endif

    tft.fillScreen(TFT_BLACK);
    delay(1000);
    tft.fillScreen(TFT_RED);
    delay(1000);
    tft.fillScreen(TFT_GREEN);
    delay(1000);
    tft.fillScreen(TFT_BLUE);
    delay(1000);

    lv_demo_benchmark();
    
#endif
}

void loop()
{
    lv_timer_handler(); /* let the GUI do its work */
    delay(5);
}
