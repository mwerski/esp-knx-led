#pragma once

#include <Arduino.h>
#if defined(ESP32)
#include "driver/ledc.h"
extern byte nextEsp32LedChannel; // next available LED channel for ESP32
#elif defined(ESP8266)
// No special includes needed for ESP8266
#elif defined(LIBRETINY)
// No special includes needed for LIBRETINY
#else
#error "Wrong hardware. Not ESP8266 or ESP32 or LIBRETINY"
#endif

#define MIN_BRIGHTNESS 12
#define MAX_BRIGHTNESS 255

#define min_f(a, b, c) (fminf(a, fminf(b, c)))
#define max_f(a, b, c) (fmaxf(a, fmaxf(b, c)))

const uint16_t lookupTable[256] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 24, 25, 26, 27, 28, 29, 30, 31, 32, 34, 35, 36, 37, 38, 39, 41, 42, 43, 44, 45,
    47, 48, 49, 51, 52, 53, 54, 56, 57, 59, 60, 61, 63, 64, 66, 67, 69, 70, 72, 73, 75, 76, 78, 79, 81, 83, 84, 86, 88, 89, 91, 93, 95, 96, 98, 100, 102, 104, 106,
    108, 109, 111, 113, 115, 117, 120, 122, 124, 126, 128, 130, 132, 135, 137, 139, 142, 144, 146, 149, 151, 154, 156, 159, 161, 164, 166, 169, 172, 174, 177, 180,
    183, 185, 188, 191, 194, 197, 200, 203, 206, 209, 212, 215, 219, 222, 225, 228, 232, 235, 238, 242, 245, 249, 252, 256, 260, 263, 267, 271, 275, 278, 282, 286,
    290, 294, 298, 302, 306, 310, 315, 319, 323, 327, 332, 336, 341, 345, 350, 354, 359, 364, 368, 373, 378, 383, 388, 392, 397, 403, 408, 413, 418, 423, 428, 434,
    439, 445, 450, 456, 461, 467, 472, 478, 484, 490, 496, 502, 508, 514, 520, 526, 532, 538, 545, 551, 557, 564, 570, 577, 584, 590, 597, 604, 611, 618, 625, 632,
    639, 646, 653, 660, 668, 675, 683, 690, 698, 705, 713, 721, 729, 736, 744, 752, 760, 769, 777, 785, 793, 802, 810, 819, 827, 836, 845, 853, 862, 871, 880, 889,
    898, 907, 917, 926, 935, 945, 954, 964, 973, 983, 993, 1003, 1013, 1023};
    
// lookup table for E27 LED Bulb with logarithmic dimming curve
const uint16_t lookupTableTwBulb[256] = {
    0, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 65, 66, 67, 68, 69, 70, 
    72, 73, 74, 75, 77, 78, 79, 80, 82, 83, 84, 86, 87, 88, 90, 91, 92, 94, 95, 97, 98, 100, 101, 103, 104, 106, 107, 109, 110, 112, 114, 115, 117, 119, 120, 
    122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142, 144, 146, 148, 150, 152, 155, 157, 159, 161, 163, 165, 168, 170, 172, 175, 177, 179, 182, 184, 
    187, 189, 192, 194, 197, 199, 202, 205, 207, 210, 213, 215, 218, 221, 224, 227, 230, 233, 236, 239, 242, 245, 248, 251, 255, 258, 261, 264, 268, 271, 274, 
    278, 281, 285, 289, 292, 296, 299, 303, 307, 311, 314, 318, 322, 326, 330, 334, 338, 342, 347, 351, 355, 359, 364, 368, 372, 377, 381, 386, 390, 395, 400, 
    404, 409, 414, 419, 424, 428, 433, 438, 443, 449, 454, 459, 464, 470, 475, 480, 486, 491, 497, 502, 508, 514, 519, 525, 531, 537, 543, 549, 555, 561, 567, 
    573, 580, 586, 592, 599, 605, 612, 618, 625, 632, 638, 645, 652, 659, 666, 673, 680, 687, 694, 702, 709, 716, 724, 731, 739, 746, 754, 762, 770, 778, 785, 
    793, 801, 810, 818, 826, 834, 843, 851, 859, 868, 877, 885, 894, 903, 912, 920, 929, 938, 948, 957, 966, 975, 985, 994, 1004, 1013, 1023 };

enum __cctMode
{// CCT mode: normal (2 separate channels), bipolar (2 instead of 3 wires), temperature control channel (CH1=brightness, CH2=temperature)
    NORMAL,
    BIPOLAR,
    TEMP_CHANNEL
};

enum __dimMode
{    
    STOP=0,
    DOWN=1,
    UP=9,
    IDLE=10
};

typedef struct __dpt3
{
    __dimMode dimMode=IDLE;
    uint8_t steps=0;

    void fromDPT3(uint8_t raw)
    {
        steps = raw & 0b111;
        if(steps == 0)
        {
            dimMode = STOP;            
        }
        else if((raw & 0b1000) == 0)
        {
            dimMode = DOWN;
        }
        else
        {
            dimMode = UP;
        }        
    }

    uint8_t toDPT3(void)
    {
        return (dimMode & 0b1111) | steps;
    }
} dpt3_t;

typedef struct __hsv
{
    uint8_t h; // hue
    uint8_t s; // sat
    uint8_t v; // val==brightness

    void fromDPT232600(uint32_t raw)
    {
        h = (raw >> 16) & 0xFF;
        s = (raw >> 8) & 0xFF;
        v = raw & 0xFF;
    }

    uint32_t toDPT232600(void)
    {
        return (h << 16) | (s << 8) | v;
    }

    inline bool operator!=(const __hsv &cmp) const
    {
        return h != cmp.h || s != cmp.s || v != cmp.v;
    }

    int changedPercent(const __hsv &cmp)
    {
        uint8_t _h = round(abs(h - cmp.h) * 255.0f / 100.0f);
        uint8_t _s = round(abs(s - cmp.s) * 255.0f / 100.0f);
        uint8_t _v = round(abs(v - cmp.v) * 255.0f / 100.0f);
        return max(_h, max(_s, _v));
    }
} hsv_t;

typedef struct __rgb
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;

    void fromDPT232600(uint32_t raw)
    {
        red = (raw >> 16) & 0xFF;
        green = (raw >> 8) & 0xFF;
        blue = raw & 0xFF;
    }

    uint32_t toDPT232600(void)
    {
        return (red << 16) | (green << 8) | blue;
    }
    inline bool operator!=(const __rgb &cmp) const
    {
        return (red != cmp.red || green != cmp.green || blue != cmp.blue);
    }
    int changedPercent(const __rgb &cmp)
    {
        uint8_t _r = round(abs(blue - cmp.blue) * 255.0f / 100.0f);
        uint8_t _g = round(abs(green - cmp.green) * 255.0f / 100.0f);
        uint8_t _b = round(abs(blue - cmp.blue) * 255.0f / 100.0f);
        return max(_r, max(_g, _b));
    }
} rgb_t;

typedef void callbackBool(bool);
typedef void callbackUint8(uint8_t);
typedef void callbackUint16(uint16_t);
typedef void callbackRgb(rgb_t);
typedef void callbackHsv(hsv_t);

class KnxLed
{
public:
    enum LightTypes
    {
        SWITCHABLE,
        DIMMABLE,
        TUNABLEWHITE,
        RGB,
        RGBW,
        RGBCT
    };

    enum LightMode
    {
        MODE_CCT,
        MODE_RGB
    };

    void initSwitchableLight(uint8_t switchPin);
    void initDimmableLight(uint8_t ledPin);
    void initTunableWhiteLight(uint8_t cwPin, uint8_t wwPin, __cctMode cctMode);
    void initRgbLight(uint8_t rPin, uint8_t gPin, uint8_t bPin);
    void initRgbwLight(uint8_t rPin, uint8_t gPin, uint8_t bPin, uint8_t wPin, rgb_t whiteLedRgbEquivalent);
    void initRgbcctLight(uint8_t rPin, uint8_t gPin, uint8_t bPin, uint8_t cwPin, uint8_t wwPin, __cctMode cctMode);

    void configDefaultBrightness(uint8_t brightness);
    void configDefaultTemperature(uint16_t temperature);
    void configDefaultHsv(hsv_t hsv);
    void configDimSpeed(uint8_t dimSetSpeed);
    void configFadeSpeed(uint8_t fadeUpTime, uint8_t fadeDownTime, uint8_t fadeColorTime);

    void registerStatusCallback(callbackBool *fctn);
    void registerBrightnessCallback(callbackUint8 *fctn);
    void registerTemperatureCallback(callbackUint16 *fctn);
    void registerColorRgbCallback(callbackRgb *fctn);
    void registerColorHsvCallback(callbackHsv *fctn);

    void switchLight(bool state);
    void setBrightness(uint8_t brightness);
    void setBrightness(uint8_t brightness, bool saveValue);
    void setTemperature(uint16_t temperature);
    void setRgb(rgb_t rgb);
    void setHsv(hsv_t hsv);

    void setRelDimCmd(dpt3_t dimCmd);
    void setRelTemperatureCmd(dpt3_t temperatureCmd);
    void setRelHueCmd(dpt3_t hueCmd);
    void setRelSaturationCmd(dpt3_t saturationCmd);

    void sendStatusUpdate();

    bool getSwitchState();
    uint8_t getBrightness();
    uint16_t getTemperature();
    rgb_t getRgb();
    hsv_t getHsv();
    void rgb2hsv(const rgb_t rgb, hsv_t &hsv);
    void hsv2rgb(const hsv_t hsv, rgb_t &rgb);
    void kelvin2rgb(const uint16_t temperature, const uint8_t brightness, rgb_t &rgb);

    void loop();

private:
    bool initialized = false;
    LightTypes lightType;
    byte outputPins[5];
    LightMode currentLightMode = MODE_CCT;
    unsigned int pwmResolution = 10;  // 2^10 = 1024
    // Default is 1023
    // All 1022 PWM steps are available at 977Hz, 488Hz, 325Hz, 244Hz, 195Hz, 162Hz, 139Hz, 122Hz, 108Hz, 97Hz, 88Hz, 81Hz, 75Hz, etc.
    // Calculation = truncate(1/(1E-6 * 1023)) for the PWM frequencies with all (or most) discrete PWM steps. (master)
#if defined(ESP32)
    unsigned int pwmFrequency = 5000; // 5kHz
    ledc_channel_t esp32LedCh[5];
#elif defined(ESP8266)
    unsigned int pwmFrequency = 2000;  // 2kHz bei Library >=3.0.0, 50Hz bei Library 2.6.3
#elif defined(LIBRETINY)
    unsigned int pwmFrequency = 1000;  // 1kHz
#endif
    uint8_t cycleTime = 4; // in ms, loop cycle time, used for fading
    uint8_t relDimInterval = 6;  // in seconds, speed for relative dimming commands
    uint8_t fadeUpInterval = 2; // in seconds, speed for fading up
    uint8_t fadeDownInterval = 2; // in seconds, speed for fading down
    uint8_t fadeColorInterval = 2; // in seconds, speed for fading color
    uint8_t relDimCount = 0;
    uint8_t fadeUpCount = 0;
    uint8_t fadeDownCount = 0;
    uint8_t fadeColorCount = 0;

    uint8_t defaultBrightness = MAX_BRIGHTNESS;
    uint8_t savedBrightness = 0;
    uint8_t setpointBrightness = 0;
    uint8_t actBrightness = 0;

    uint16_t defaultTemperature = 3500;
    uint16_t setpointTemperature = defaultTemperature;
    uint16_t actTemperature = defaultTemperature;

    hsv_t defaultHsv;
    hsv_t savedHsv;
    hsv_t setpointHsv;
    hsv_t actHsv;    

    bool isTwBipolar = false;     // Tunable White with 2-Wires and different polarity for each channel
    bool isTwTempCh = false;      // Tunable White with brightness channel and temperature channel
    rgb_t whiteRgbEquivalent;     // Color temperature of white LED for RGBW

    dpt3_t relDimCmd;
    dpt3_t relTemperatureCmd;
    dpt3_t relHueCmd;
    dpt3_t relSaturationCmd;

    callbackBool *returnStatusFctn;
    callbackUint8 *returnBrightnessFctn;
    callbackUint16 *returnTemperatureFctn;
    callbackRgb *returnColorRgbFctn;
    callbackHsv *returnColorHsvFctn;

    void initOutputChannels(uint8_t usedChannels);
    void relativeDimming();
    void fade();
    void pwmControl();
    void ledAnalogWrite(byte channel, uint16_t duty);
    void returnStatus();
    void returnBrightness();
    void returnTemperature();
    void returnColors();
    uint8_t rgb2White(const rgb_t rgb);
};