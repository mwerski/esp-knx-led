#include "esp-knx-led.h"
#if defined(ESP32)
byte nextEsp32LedChannel = LEDC_CHANNEL_0; // next available LED channel for ESP32
#endif

void KnxLed::switchLight(bool state)
{
	switch (lightType)
	{
	case SWITCHABLE:
	{
		setBrightness(int(state));
		break;
	}
	case DIMMABLE:
	case TUNABLEWHITE:
	{
		if (state)
		{
			if (defaultBrightness >= MIN_BRIGHTNESS)
			{
				setBrightness(defaultBrightness);
			}
			// If default brightness is set to 0, the last brightness will be restored
			else if (savedBrightness >= MIN_BRIGHTNESS)
			{
				setBrightness(savedBrightness);
			}
			else
			{
				setBrightness(MAX_BRIGHTNESS);
			}

			if (lightType == TUNABLEWHITE && defaultTemperature > 0)
			{
				setTemperature(defaultTemperature);
			}
		}
		else
		{
			if (setpointBrightness > 0)
			{
				savedBrightness = setpointBrightness;
			}
			setBrightness(0);
		}
		break;
	}
	case RGB:
	case RGBW:
	case RGBCT:
	{
		if (state)
		{
			hsv_t hsv;
			if (defaultHsv.v >= MIN_BRIGHTNESS)
			{
				hsv = defaultHsv;
				setHsv(hsv);
			}
			else if (defaultTemperature > 0)
			{
				setTemperature(defaultTemperature);
				if (defaultBrightness >= MIN_BRIGHTNESS)
				{
					setBrightness(defaultBrightness);
				}
				// If default brightness is set to 0, the last brightness will be restored
				else if (savedBrightness >= MIN_BRIGHTNESS)
				{
					setBrightness(savedBrightness);
				}
				else
				{
					setBrightness(MAX_BRIGHTNESS);
				}
			}
			else if (defaultBrightness >= MIN_BRIGHTNESS)
			{
				setBrightness(defaultBrightness);
			}
			// If default HSV brightness value is set to 0, the last HSV will be restored
			else if (savedHsv.v >= MIN_BRIGHTNESS)
			{
				hsv = savedHsv;
				setHsv(hsv);
			}
			else
			{
				hsv = {50, 255, 255};
				setHsv(hsv);
			}
		}
		else
		{
			if (setpointBrightness > 0)
			{
				savedHsv = setpointHsv;
			}
			setBrightness(0);
		}
		break;
	}
	}
}

void KnxLed::setBrightness(uint8_t brightness)
{
	setBrightness(brightness, true);
}

void KnxLed::setBrightness(uint8_t brightness, bool saveValue)
{
	if (brightness != setpointBrightness)
	{
		setpointBrightness = constrain(brightness, 0, MAX_BRIGHTNESS);
		if (setpointBrightness > 0 && saveValue)
		{
			savedBrightness = setpointBrightness;
		}
		returnBrightness();
		relDimCmd.dimMode = IDLE;
		relTemperatureCmd.dimMode = IDLE;
		setpointHsv.v = brightness;
	}
}

void KnxLed::setTemperature(uint16_t temperature)
{
	setpointTemperature = constrain(temperature, minTemperature, maxTemperature);
	returnTemperature();
	relDimCmd.dimMode = IDLE;
	relTemperatureCmd.dimMode = IDLE;
	if (currentLightMode != MODE_CCT)
	{
		actTemperature = setpointTemperature;
		currentLightMode = MODE_CCT;
	}
	if (lightType == RGB /*|| lightType == RGBW*/) // no separate CCT channels
	{
		rgb_t _rgb;
		hsv_t _hsv;
		kelvin2rgb(setpointTemperature, MAX_BRIGHTNESS, _rgb);
		rgb2hsv(_rgb, _hsv);
		_hsv.v = setpointBrightness;
		setpointHsv = _hsv;
	}
}

// set RGB value. This will be converted to HSV internally
void KnxLed::setRgb(rgb_t rgb)
{
	hsv_t _hsv;
	if (rgb.red + rgb.green + rgb.blue == 0)
	{
		_hsv.h = actHsv.h;
		_hsv.s = actHsv.s;
		_hsv.v = 0;
	}
	else
	{
		rgb2hsv(rgb, _hsv);
	}
	// if light is already on, don't change brightness unless brightness is set in RGB
	if (actHsv.v > 0 && _hsv.v == 255)
	{
		_hsv.v = setpointHsv.v;
	}
	setHsv(_hsv);
}

// set HSV value.
void KnxLed::setHsv(hsv_t hsv)
{
	setpointHsv = hsv;
	if (actHsv.v == 0)
	{
		actHsv.h = hsv.h;
		actHsv.s = hsv.s;
	}

	returnColors();
	relDimCmd.dimMode = IDLE;
	relTemperatureCmd.dimMode = IDLE;
	currentLightMode = MODE_RGB;
	setBrightness(hsv.v);
}

void KnxLed::configDefaultBrightness(uint8_t brightness)
{
	if (brightness >= 0 && brightness <= MAX_BRIGHTNESS)
	{
		defaultBrightness = brightness;
		if (defaultHsv.v > 0)
		{
			defaultHsv.v = brightness;
		}
	}
}

void KnxLed::configDefaultTemperature(uint16_t temperature)
{
	if (temperature == 0 || (temperature >= minTemperature && temperature <= maxTemperature))
	{
		defaultTemperature = temperature;
		if(setpointBrightness == 0)
		{
			setpointTemperature = defaultTemperature;
    		actTemperature = defaultTemperature;
		}
	}
}

void KnxLed::configMinTemperature(uint16_t temperature)
{
	if (temperature < maxTemperature) {
		minTemperature = temperature;
		if (defaultTemperature < minTemperature) configDefaultTemperature(minTemperature);
	}
}

void KnxLed::configMaxTemperature(uint16_t temperature)
{
	if(temperature > minTemperature) {
		maxTemperature = temperature;
		if (defaultTemperature > maxTemperature) configDefaultTemperature(maxTemperature);
	}
}

void KnxLed::configDefaultHsv(hsv_t hsv)
{
	defaultHsv = hsv;
}

void KnxLed::configDimSpeed(uint8_t relDimTime)
{
	relDimInterval = relDimTime;
}

void KnxLed::configFadeSpeed(uint8_t fadeUpTime, uint8_t fadeDownTime, uint8_t fadeColorTime)
{
	fadeUpInterval = fadeUpTime;
	fadeDownInterval = fadeDownTime;
	fadeColorInterval = fadeColorTime;
}

void KnxLed::setRelDimCmd(dpt3_t dimCmd)
{
	relDimCmd = dimCmd;
}

void KnxLed::setRelTemperatureCmd(dpt3_t temperatureCmd)
{
	if(temperatureCmd.dimMode != STOP)
	{
		if (currentLightMode != MODE_CCT)
		{
			setTemperature(actTemperature);
		}
	}
	relTemperatureCmd = temperatureCmd;
}

void KnxLed::setRelHueCmd(dpt3_t hueCmd)
{
	if(hueCmd.dimMode != STOP)
	{
		if (currentLightMode != MODE_RGB)
		{
			hsv_t _hsv = actHsv;
			_hsv.v = setpointBrightness;
			if(_hsv.s == 0)
			{
				_hsv.s = 255;
			}
			setHsv(_hsv);
		}
	}
	relHueCmd = hueCmd;
}

void KnxLed::setRelSaturationCmd(dpt3_t saturationCmd)
{
	relSaturationCmd = saturationCmd;
	if(saturationCmd.dimMode != STOP)
	{
		if (currentLightMode != MODE_RGB)
		{
			hsv_t _hsv = actHsv;
			_hsv.v = setpointBrightness;
			setHsv(_hsv);
		}
	}
}

void KnxLed::loop()
{
	if (initialized)
	{
		relativeDimming();
		fade();
	}
}

void KnxLed::relativeDimming()
{
	relDimCount++;
	if (relDimCount >= relDimInterval)
	{
		relDimCount = 0;
		if (relDimCmd.dimMode == UP && actBrightness < MAX_BRIGHTNESS)
		{
			setpointBrightness = actBrightness + 1;
			if ((int)(setpointBrightness / 2.55 * 2 + 0.7) % 20 == 0)
			{
				returnBrightness();
			}
		}
		else if (relDimCmd.dimMode == DOWN && actBrightness > MIN_BRIGHTNESS)
		{
			setpointBrightness = actBrightness - 1;
			if ((int)(setpointBrightness / 2.55 * 2 + 0.7) % 20 == 0)
			{
				returnBrightness();
			}
		}
		else if (relDimCmd.dimMode == STOP)
		{
			savedBrightness = setpointBrightness; // = actBrightness;
			returnBrightness();
			relDimCmd.dimMode = IDLE;
		}

		if (relTemperatureCmd.dimMode == UP && actTemperature < maxTemperature)
		{
			setpointTemperature = min<uint16_t>(actTemperature + 20, maxTemperature);
			if (setpointTemperature % 300 == 0)
			{
				returnTemperature();
			}
		}
		else if (relTemperatureCmd.dimMode == DOWN && actTemperature > minTemperature)
		{
			setpointTemperature = max<uint16_t>(actTemperature - 20, minTemperature);
			if (setpointTemperature % 300 == 0)
			{
				returnTemperature();
			}
		}
		else if (relTemperatureCmd.dimMode == STOP)
		{
			returnTemperature();
			relTemperatureCmd.dimMode = IDLE;
		}

		if (relHueCmd.dimMode == UP)
		{
			setpointHsv.h = actHsv.h + 1;
			if ((int)(setpointHsv.h / 2.55 * 2 + 0.7) % 20 == 0)
			{
				returnColors();
			}
		}
		else if (relHueCmd.dimMode == DOWN)
		{
			setpointHsv.h = actHsv.h - 1;
			if ((int)(setpointHsv.h / 2.55 * 2 + 0.7) % 20 == 0)
			{
				returnColors();
			}
		}
		else if (relHueCmd.dimMode == STOP)
		{
			savedHsv.h = setpointHsv.h;
			returnColors();
			relHueCmd.dimMode = IDLE;
		}

		if (relSaturationCmd.dimMode == UP && actHsv.s < 255)
		{
			setpointHsv.s = actHsv.s + 1;
			if ((int)(setpointHsv.s / 2.55 * 2 + 0.7) % 20 == 0)
			{
				returnColors();
			}
		}
		else if (relSaturationCmd.dimMode == DOWN && actHsv.s > 0)
		{
			setpointHsv.s = actHsv.s - 1;
			if ((int)(setpointHsv.s / 2.55 * 2 + 0.7) % 20 == 0)
			{
				returnColors();
			}
		}
		else if (relSaturationCmd.dimMode == STOP)
		{
			savedHsv.s = setpointHsv.s;
			returnColors();
			relSaturationCmd.dimMode = IDLE;
		}
	}
}

void KnxLed::fade()
{	
	// Fade immediately if any relative command is active
	bool relCmdActive = relDimCmd.dimMode != IDLE || relTemperatureCmd.dimMode != IDLE || relHueCmd.dimMode != IDLE || relSaturationCmd.dimMode != IDLE;

	// Increment counters and check if fade should occur
	bool relFadeUp = (++fadeUpCount >= fadeUpInterval) || relCmdActive;
	bool relFadeDown = (++fadeDownCount >= fadeDownInterval) || relCmdActive;
	bool relFadeColor = (++fadeColorCount >= fadeColorInterval) || relCmdActive;
	if (relFadeUp) fadeUpCount = 0;
	if (relFadeDown) fadeDownCount = 0;
	if (relFadeColor) fadeColorCount = 0;

	bool updatePwm = false;		
	if (setpointBrightness != actBrightness)
	{
		int oldBrightness = actBrightness;
		if (setpointBrightness > actBrightness && relFadeUp)
		{
			actBrightness++;
			updatePwm = true;
		}
		else if (setpointBrightness < actBrightness && relFadeDown)
		{
			actBrightness--;
			updatePwm = true;
		}

		if ((actBrightness == 0) != (oldBrightness == 0))
		{
			returnStatus();
		}
	}

	if (currentLightMode == MODE_CCT && (lightType == RGBCT || lightType == RGBW))
	{
		if (actHsv.v > 0 && relFadeDown)
		{
			actHsv.v--;
			updatePwm = true;
		}
	}
	else
	{
		if (setpointBrightness != actHsv.v)
		{
			if (setpointBrightness > actHsv.v && relFadeUp)
			{
				actHsv.v++;
				updatePwm = true;
			}
			else if (setpointBrightness < actHsv.v && relFadeDown)
			{
				actHsv.v--;
				updatePwm = true;
			}
		}
	}

	if(relFadeColor)
	{
		if (setpointTemperature > actTemperature)
		{
			actTemperature = min<uint16_t>(actTemperature + 20, setpointTemperature);
			updatePwm = true;
		}
		else if (setpointTemperature < actTemperature)
		{
			actTemperature = max<uint16_t>(actTemperature - 20, setpointTemperature);
			updatePwm = true;
		}

		uint8_t diffH = abs(setpointHsv.h - actHsv.h);
		if (diffH > 0)
		{
			bool do360overflow = diffH > 128;
			if ((setpointHsv.h > actHsv.h) != do360overflow)
			{
				actHsv.h = (actHsv.h + 1) % 256;
			}
			else
			{
				actHsv.h = (actHsv.h + 255) % 256;
			}
			updatePwm = true;
		}

		if (diffH > 43 && (setpointHsv.s - actHsv.s) < diffH && actHsv.s > 1)
		{
			actHsv.s -= 2;
			updatePwm = true;
		}
		else if (setpointHsv.s != actHsv.s)
		{
			actHsv.s += setpointHsv.s > actHsv.s ? 1 : -1;
			updatePwm = true;
		}
	}

	// to avoid flickering, only update on change
	if (updatePwm)
	{
		pwmControl();
	}
}

void KnxLed::pwmControl()
{
	switch (lightType)
	{
	case SWITCHABLE:
	{
		digitalWrite(outputPins[0], actBrightness > 0);
		break;
	}
	case DIMMABLE:
	{
		int dutyCh0 = actBrightness;
		ledAnalogWrite(0, lookupTable[dutyCh0]);
		break;
	}
	case TUNABLEWHITE:
	{
		if (isTwBipolar)
		{
			// 2-Wire tunable LEDs. Different polarity for each channel controlled by 4quadrant H-Brige
			float maxBt = actBrightness * 1023.0 / MAX_BRIGHTNESS / rangeTemperature;

			int dutyCh0 = constrain((actTemperature - minTemperature) * maxBt, 0, 1023) + 0.5;
			int dutyCh1 = constrain((maxTemperature - actTemperature) * maxBt, 0, 1023) + 0.5;
#if defined(ESP32)
			ledc_set_duty_with_hpoint(LEDC_HIGH_SPEED_MODE, esp32LedCh[0], dutyCh0, 0);
			ledc_set_duty_with_hpoint(LEDC_HIGH_SPEED_MODE, esp32LedCh[1], dutyCh1, dutyCh0);
			ledc_update_duty(LEDC_HIGH_SPEED_MODE, esp32LedCh[0]);
			ledc_update_duty(LEDC_HIGH_SPEED_MODE, esp32LedCh[1]);
#else
			// TODO
			ledAnalogWrite(0, lookupTable[dutyCh0]);
			ledAnalogWrite(1, lookupTable[dutyCh1]);
#endif
		}
		else
		{
			uint16_t dutyCh0 = 0;
			uint16_t dutyCh1 = 0;
			if (!isTwTempCh)
			{
				dutyCh0 = constrain(min<uint16_t>(2 * (actTemperature - minTemperature), rangeTemperature) / rangeTemperature * actBrightness, 0, 255) + 0.5;
				dutyCh1 = constrain(min<uint16_t>(2 * (maxTemperature - actTemperature), rangeTemperature) / rangeTemperature * actBrightness, 0, 255) + 0.5;
				dutyCh0 = lookupTable[dutyCh0];
				dutyCh1 = lookupTable[dutyCh1];
			}
			else if (actBrightness > 0)
			{
				dutyCh0 = lookupTableTwBulb[actBrightness];
				dutyCh1 = constrain((actTemperature - minTemperature) / rangeTemperature * 1023, 0, 1023) + 0.5;
			}
			ledAnalogWrite(0, dutyCh0);
			ledAnalogWrite(1, dutyCh1);
		}
		break;
	}
	case RGB:
	{
		rgb_t _rgb;
		hsv2rgb(actHsv, _rgb);

		ledAnalogWrite(0, lookupTable[_rgb.red]);
		ledAnalogWrite(1, lookupTable[_rgb.green]);
		ledAnalogWrite(2, lookupTable[_rgb.blue]);
		break;
	}
	case RGBW:
	{
		/*rgb_t _rgb;
		hsv2rgb(actHsv, _rgb);
		uint8_t white = rgb2White(_rgb);*/
		
		rgb_t _rgb;
		uint8_t white;				
		if (currentLightMode == MODE_CCT)
		{
			kelvin2rgb(actTemperature, MAX_BRIGHTNESS, _rgb);
			float r = _rgb.red + (_rgb.red - whiteRgbEquivalent.red)/2.0;
			float g = _rgb.green + (_rgb.green - whiteRgbEquivalent.green)/2.0;
			float b = _rgb.blue + (_rgb.blue - whiteRgbEquivalent.blue)/2.0;
			float factor = max_f(r, g, b)/actBrightness;
			_rgb.red = constrain(r/factor, 0, 255) + 0.5;
			_rgb.green = constrain(g/factor, 0, 255) + 0.5;
			_rgb.blue = constrain(b/factor, 0, 255) + 0.5;
			white = actBrightness;
		}
		else
		{
			hsv2rgb(actHsv, _rgb);
			white = rgb2White(_rgb);
		}

		ledAnalogWrite(0, lookupTable[_rgb.red]);
		ledAnalogWrite(1, lookupTable[_rgb.green]);
		ledAnalogWrite(2, lookupTable[_rgb.blue]);
		ledAnalogWrite(3, lookupTable[white]);
		break;
	}
	case RGBCT:
		rgb_t _rgb;
		hsv2rgb(actHsv, _rgb);
		// Serial.printf("PWM IST: R=%3d,G=%3d,B=%3d H=%3d,S=%3d,V=%3d\n", _rgb.red, _rgb.green, _rgb.blue, actHsv.h, actHsv.s, actHsv.v);

		ledAnalogWrite(0, lookupTable[_rgb.red]);
		ledAnalogWrite(1, lookupTable[_rgb.green]);
		ledAnalogWrite(2, lookupTable[_rgb.blue]);
		uint16_t dutyCh3 = 0;
		uint16_t dutyCh4 = 0;

		if (!isTwTempCh)
		{
			dutyCh3 = constrain(min<uint16_t>(2 * (actTemperature - minTemperature), rangeTemperature) / rangeTemperature * (actBrightness - actHsv.v), 0, 255) + 0.5;
			dutyCh4 = constrain(min<uint16_t>(2 * (maxTemperature - actTemperature), rangeTemperature) / rangeTemperature * (actBrightness - actHsv.v), 0, 255) + 0.5;
			dutyCh3 = lookupTable[dutyCh3];
			dutyCh4 = lookupTable[dutyCh4];		
		}
		else if (actBrightness > actHsv.v)
		{
			dutyCh3 = lookupTableTwBulb[constrain(actBrightness - actHsv.v, 0, 255)];
			dutyCh4 = constrain((actTemperature - minTemperature) / rangeTemperature * 1023, 0, 1023) + 0.5;
		}
		ledAnalogWrite(3, dutyCh3);
		ledAnalogWrite(4, dutyCh4);
	}
}

void KnxLed::ledAnalogWrite(byte channel, uint16_t duty)
{
#if defined(ESP32)
	ledcWrite(esp32LedCh[channel], duty);
#elif defined(LIBRETINY)
	// on Beken hardware, for some reason the LED will flicker if the PWM value changes from 1022 to 1023
	// therefore limit the value to 1022
	if(duty == 1023)
	{
		duty = 1022;
	}
	analogWrite(outputPins[channel], duty);
#else
	analogWrite(outputPins[channel], duty);
#endif
}

bool KnxLed::getSwitchState()
{
	return setpointBrightness > 0;
}

uint8_t KnxLed::getBrightness()
{
	return max<uint8_t>(0, actBrightness);
}

uint16_t KnxLed::getTemperature()
{
	return actTemperature;
}

rgb_t KnxLed::getRgb()
{
	rgb_t _rgb;
	hsv2rgb(actHsv, _rgb);
	return _rgb;
}

hsv_t KnxLed::getHsv()
{
	return actHsv;
}

void KnxLed::returnStatus()
{
	if (returnStatusFctn != nullptr)
	{
		returnStatusFctn(getSwitchState());
	}
}

void KnxLed::returnBrightness()
{
	if (returnBrightnessFctn != nullptr)
	{
		returnBrightnessFctn(setpointBrightness);
	}
}

void KnxLed::returnTemperature()
{
	if (returnTemperatureFctn != nullptr)
	{
		returnTemperatureFctn(setpointTemperature);
	}
}

void KnxLed::returnColors()
{
	if (returnColorHsvFctn != nullptr)
	{
		returnColorHsvFctn(setpointHsv);
	}
	if (returnColorRgbFctn != nullptr)
	{
		rgb_t _rgb;
		hsv2rgb(setpointHsv, _rgb);
		returnColorRgbFctn(_rgb);
	}
}

void KnxLed::registerStatusCallback(callbackBool *fctn)
{
	returnStatusFctn = fctn;
}

void KnxLed::registerBrightnessCallback(callbackUint8 *fctn)
{
	returnBrightnessFctn = fctn;
}

void KnxLed::registerTemperatureCallback(callbackUint16 *fctn)
{
	returnTemperatureFctn = fctn;
}

void KnxLed::registerColorRgbCallback(callbackRgb *fctn)
{
	returnColorRgbFctn = fctn;
}

void KnxLed::registerColorHsvCallback(callbackHsv *fctn)
{
	returnColorHsvFctn = fctn;
}

void KnxLed::sendStatusUpdate()
{
	returnStatus();
    returnBrightness();
    returnTemperature();
    returnColors();
}

void KnxLed::initSwitchableLight(uint8_t switchPin)
{
	lightType = SWITCHABLE;
	outputPins[0] = switchPin;
	pinMode(outputPins[0], OUTPUT);
	initialized = true;
}

void KnxLed::initDimmableLight(uint8_t ledPin)
{
	lightType = DIMMABLE;
	outputPins[0] = ledPin;
	initOutputChannels(1);
}

void KnxLed::initTunableWhiteLight(uint8_t cwPin, uint8_t wwPin, __cctMode cctMode)
{
	lightType = TUNABLEWHITE;
	outputPins[0] = cwPin;
	outputPins[1] = wwPin;
	isTwBipolar = cctMode == BIPOLAR;
	isTwTempCh = cctMode == TEMP_CHANNEL;
	initOutputChannels(2);
}

void KnxLed::initRgbLight(uint8_t rPin, uint8_t gPin, uint8_t bPin)
{
	lightType = RGB;
	outputPins[0] = rPin;
	outputPins[1] = gPin;
	outputPins[2] = bPin;
	initOutputChannels(3);
}

void KnxLed::initRgbwLight(uint8_t rPin, uint8_t gPin, uint8_t bPin, uint8_t wPin, rgb_t whiteLedRgbEquivalent)
{
	lightType = RGBW;
	currentLightMode = MODE_RGB;
	outputPins[0] = rPin;
	outputPins[1] = gPin;
	outputPins[2] = bPin;
	outputPins[3] = wPin;
	whiteRgbEquivalent = whiteLedRgbEquivalent;
	initOutputChannels(4);
}

void KnxLed::initRgbcctLight(uint8_t rPin, uint8_t gPin, uint8_t bPin, uint8_t cwPin, uint8_t wwPin, __cctMode cctMode)
{
	lightType = RGBCT;
	currentLightMode = MODE_RGB;
	outputPins[0] = rPin;
	outputPins[1] = gPin;
	outputPins[2] = bPin;
	outputPins[3] = cwPin;
	outputPins[4] = wwPin;
	isTwBipolar = cctMode == BIPOLAR;
	isTwTempCh = cctMode == TEMP_CHANNEL;
	initOutputChannels(5);
}

// internal helper which will be called by init
void KnxLed::initOutputChannels(uint8_t usedChannels)
{
#if defined(ESP32)
	if (nextEsp32LedChannel <= LEDC_CHANNEL_MAX - usedChannels)
	{
		for (uint i = 0; i < usedChannels; i++)
		{
			esp32LedCh[i] = static_cast<ledc_channel_t>(nextEsp32LedChannel++);
			ledcSetup(esp32LedCh[i], pwmFrequency, pwmResolution);
			ledcAttachPin(outputPins[i], esp32LedCh[i]);
		}
	}
#else
	for (uint8_t i = 0; i < usedChannels; i++)
	{
		pinMode(outputPins[i], OUTPUT);
	}
	analogWriteResolution(pwmResolution);
	#if defined(ESP8266)
		analogWriteFreq(pwmFrequency);
	#else
		analogWriteFrequency(pwmFrequency);
	#endif
#endif
	initialized = true;
}

void KnxLed::rgb2hsv(const rgb_t rgb, hsv_t &hsv)
{
	float r = rgb.red / 255.0f;
	float g = rgb.green / 255.0f;
	float b = rgb.blue / 255.0f;

	float max = max_f(r, g, b);
	float min = min_f(r, g, b);
	float d = max - min;

	float h = 0;
	float s = max == 0 ? 0 : d / max;
	float v = max;

	if (max != min)
	{
		if (max == r)
		{
			h = (g - b) / d + (g < b ? 6 : 0);
		}
		else if (max == g)
		{
			h = (b - r) / d + 2;
		}
		else
		{
			h = (r - g) / d + 4;
		}
		h /= 6;
	}

	hsv.h = constrain(h * 255.0f, 0, 255) + 0.5;
	hsv.s = constrain(s * 255.0f, 0, 255) + 0.5;
	hsv.v = constrain(v * 255.0f, 0, 255) + 0.5;
}

void KnxLed::hsv2rgb(const hsv_t hsv, rgb_t &rgb)
{
	float h = hsv.h / 255.0f;
	float s = hsv.s / 255.0f;
	float v = hsv.v / 255.0f;

	float r = 0, g = 0, b = 0;

	int i = floor(h * 6);
	float f = h * 6 - i;
	float p = v * (1 - s);
	float q = v * (1 - f * s);
	float t = v * (1 - (1 - f) * s);
	switch (i % 6)
	{
	case 0:
		r = v, g = t, b = p;
		break;
	case 1:
		r = q, g = v, b = p;
		break;
	case 2:
		r = p, g = v, b = t;
		break;
	case 3:
		r = p, g = q, b = v;
		break;
	case 4:
		r = t, g = p, b = v;
		break;
	case 5:
		r = v, g = p, b = q;
		break;
	}

	rgb.red = constrain(r * 255.0f, 0, 255) + 0.5;
	rgb.green = constrain(g * 255.0f, 0, 255) + 0.5;
	rgb.blue = constrain(b * 255.0f, 0, 255) + 0.5;
}

void KnxLed::kelvin2rgb(const uint16_t temperature, const uint8_t brightness, rgb_t &rgb)
{
	float red;
	float green;
	float blue;
	float temp = constrain(temperature, 500, 40000) / 100;

	// Calculate red
	if (temp <= 66)
	{
		red = 255;
	}
	else
	{
		red = 329.698727446 * pow(temp - 60, -0.1332047592);
	}

	// Calculate green
	if (temp < 66)
	{
		green = 99.4708025861 * log(temp) - 161.1195681661;
	}
	else
	{
		green = 288.1221695283 * pow(temp - 60, -0.0755148492);
	}

	// Calculate blue
	if (temp <= 19)
	{
		blue = 0;
	}
	else if (temp <= 66)
	{
		blue = 138.5177312231 * log(temp - 10) - 305.0447927307;
	}
	else
	{
		blue = 255;
	}

	red = red * brightness / 255.0f;
	green = green * brightness / 255.0f;
	blue = blue * brightness / 255.0f;

	// Constrains values for 8 bit PWM
	rgb.red = constrain(red, 0, 255) + 0.5;
	rgb.green = constrain(green, 0, 255) + 0.5;
	rgb.blue = constrain(blue, 0, 255) + 0.5;
}

uint8_t KnxLed::rgb2White(const rgb_t rgb)
{
	// Set the white value to the highest it can be for the given color
	// (without over saturating any channel - thus the minimum of them).
	float minWhiteValue = min_f(rgb.red * 255.0f / whiteRgbEquivalent.red, rgb.green * 255.0f / whiteRgbEquivalent.green, rgb.blue * 255.0f / whiteRgbEquivalent.blue);

	return constrain(minWhiteValue, 0, 255) + 0.5;
}