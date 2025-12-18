#include "esp-knx-led.h"
#if defined(ESP32)
uint8_t KnxLed::nextLedcChannel = 0;

bool KnxLed::allocateLedc(uint8_t count, ledc_channel_t* out) {
	if (count == 0 || out == nullptr) return false;
	// LEDC_CHANNEL_MAX ist der höchste gueltige Channel-Index (typ. 7/15 je nach SoC/SDK)
	if ((uint16_t)nextLedcChannel + (uint16_t)count > (uint16_t)LEDC_CHANNEL_MAX + 1u) return false;
	for (uint8_t i = 0; i < count; i++) {
		out[i] = static_cast<ledc_channel_t>(nextLedcChannel++);
	}
	return true;
}
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
		if (lightType == RGB || lightType == RGBW || lightType == RGBCT) returnColors();
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
		rangeTemperature = maxTemperature - minTemperature;   // <-- hinzufügen
		if (defaultTemperature < minTemperature) configDefaultTemperature(minTemperature);
		setpointTemperature = constrain(setpointTemperature, minTemperature, maxTemperature);
		actTemperature = constrain(actTemperature, minTemperature, maxTemperature);
	}
}

void KnxLed::configMaxTemperature(uint16_t temperature)
{
	if(temperature > minTemperature) {
		maxTemperature = temperature;
		rangeTemperature = maxTemperature - minTemperature;   // <-- hinzufügen
		if (defaultTemperature > maxTemperature) configDefaultTemperature(maxTemperature);
		setpointTemperature = constrain(setpointTemperature, minTemperature, maxTemperature);
		actTemperature = constrain(actTemperature, minTemperature, maxTemperature);
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

	// Helper: clamp float to [lo..hi]
	auto clampf = [](float x, float lo, float hi) -> float {
			return (x < lo) ? lo : (x > hi) ? hi : x;
	};

	// Helper: temperature fraction 0..1 (guarded)
	auto tempFrac01 = [&]() -> float {
			if (rangeTemperature == 0) return 0.0f;
			float num = (float)(actTemperature - minTemperature);
			float den = (float)rangeTemperature;
			return clampf(num / den, 0.0f, 1.0f);
	};

	switch (lightType)
	{
	case SWITCHABLE:
	{
		digitalWrite(outputPins[0], actBrightness > 0);
		break;
	}
	case DIMMABLE:
	{
		const uint8_t dutyCh0 = actBrightness;
		ledAnalogWrite(0, lookupTable[dutyCh0]);
		break;
	}
	case TUNABLEWHITE:
	{
		if (isTwBipolar)
		{
			if (rangeTemperature == 0)
			{
				ledAnalogWrite(0, 0);
				ledAnalogWrite(1, 0);
				break;
			}
			// 2-Wire tunable LEDs. Different polarity for each channel controlled by 4quadrant H-Brige
			float maxBt = ((float)actBrightness * 1023.0f) / ((float)MAX_BRIGHTNESS * (float)rangeTemperature);

			int dutyCh0 = (int)(clampf(((float)(actTemperature - minTemperature) * maxBt), 0.0f, 1023.0f) + 0.5f);
			int dutyCh1 = (int)(clampf(((float)(maxTemperature - actTemperature) * maxBt), 0.0f, 1023.0f) + 0.5f);
#if defined(ESP32)
			ledc_set_duty_with_hpoint(LEDC_HIGH_SPEED_MODE, esp32LedCh[0], dutyCh0, 0);
			ledc_set_duty_with_hpoint(LEDC_HIGH_SPEED_MODE, esp32LedCh[1], dutyCh1, dutyCh0);
			ledc_update_duty(LEDC_HIGH_SPEED_MODE, esp32LedCh[0]);
			ledc_update_duty(LEDC_HIGH_SPEED_MODE, esp32LedCh[1]);
#else
			// TODO
			ledAnalogWrite(0, lookupTable[(uint8_t)constrain(dutyCh0 / 4, 0, 255)]);
			ledAnalogWrite(1, lookupTable[(uint8_t)constrain(dutyCh1 / 4, 0, 255)]);
#endif
		}
		else
		{
			uint16_t dutyCh0 = 0;
			uint16_t dutyCh1 = 0;
			if (!isTwTempCh)
			{
				if (rangeTemperature == 0)
				{
						dutyCh0 = 0;
						dutyCh1 = 0;
				}
				else
				{
					// Use float math; result in 0..255 then lookup -> 0..1023
					float f0 = (2.0f * (float)(actTemperature - minTemperature)) / (float)rangeTemperature;
					float f1 = (2.0f * (float)(maxTemperature - actTemperature)) / (float)rangeTemperature;

					uint8_t v0 = (uint8_t)(clampf(f0, 0.0f, 1.0f) * (float)actBrightness + 0.5f);
					uint8_t v1 = (uint8_t)(clampf(f1, 0.0f, 1.0f) * (float)actBrightness + 0.5f);

					dutyCh0 = lookupTable[v0];
					dutyCh1 = lookupTable[v1];
				}

			}
			else 
			{
				if (actBrightness > 0)
				{
					dutyCh0 = lookupTableTwBulb[actBrightness];
					dutyCh1 = (uint16_t)(tempFrac01() * 1023.0f + 0.5f);
				}
				else
				{
					dutyCh0 = 0;
					dutyCh1 = 0;
				}
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
		rgb_t _rgb;
		uint8_t white = 0;				
		if (currentLightMode == MODE_CCT)
		{
			// Guard: avoid division by zero and weird scaling when brightness is 0
			if (actBrightness == 0)
			{
				_rgb = {0, 0, 0};
				white = 0;
			}
			else
			{
				kelvin2rgb(actTemperature, MAX_BRIGHTNESS, _rgb);
				// If the equivalent is invalid, fall back to "pure white channel"
				if (whiteRgbEquivalent.red == 0 || whiteRgbEquivalent.green == 0 || whiteRgbEquivalent.blue == 0)
				{
					_rgb = {0, 0, 0};
					white = actBrightness;
				}
				else
				{
					float r = (float)_rgb.red   + ((float)_rgb.red   - (float)whiteRgbEquivalent.red)   / 2.0f;
					float g = (float)_rgb.green + ((float)_rgb.green - (float)whiteRgbEquivalent.green) / 2.0f;
					float b = (float)_rgb.blue  + ((float)_rgb.blue  - (float)whiteRgbEquivalent.blue)  / 2.0f;

					float mx = max_f(r, g, b);
					float factor = (mx <= 0.0f) ? 1.0f : (mx / (float)actBrightness);

					_rgb.red   = (uint8_t)(clampf(r / factor, 0.0f, 255.0f) + 0.5f);
					_rgb.green = (uint8_t)(clampf(g / factor, 0.0f, 255.0f) + 0.5f);
					_rgb.blue  = (uint8_t)(clampf(b / factor, 0.0f, 255.0f) + 0.5f);
					white = actBrightness;
				}
			}
		}
		else
		{
			hsv2rgb(actHsv, _rgb);
			// White calculation can divide by 0 if equivalent is invalid
			if (whiteRgbEquivalent.red == 0 || whiteRgbEquivalent.green == 0 || whiteRgbEquivalent.blue == 0)
			{
				white = 0;
			}
			else
			{
				white = rgb2White(_rgb);
			}
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
		#ifdef DEBUG
		Serial.printf("PWM IST: R=%3d,G=%3d,B=%3d H=%3d,S=%3d,V=%3d\n", _rgb.red, _rgb.green, _rgb.blue, actHsv.h, actHsv.s, actHsv.v);
		#endif
		ledAnalogWrite(0, lookupTable[_rgb.red]);
		ledAnalogWrite(1, lookupTable[_rgb.green]);
		ledAnalogWrite(2, lookupTable[_rgb.blue]);
		uint16_t dutyCh3 = 0;
		uint16_t dutyCh4 = 0;

		if (!isTwTempCh)
		{
			if (rangeTemperature == 0 || actBrightness <= actHsv.v)
			{
				dutyCh3 = 0;
				dutyCh4 = 0;
			}
			else
			{
				float extra = (float)(actBrightness - actHsv.v);
				float f3 = (2.0f * (float)(actTemperature - minTemperature)) / (float)rangeTemperature;
				float f4 = (2.0f * (float)(maxTemperature - actTemperature)) / (float)rangeTemperature;
				uint8_t v3 = (uint8_t)(clampf(f3, 0.0f, 1.0f) * extra + 0.5f);
				uint8_t v4 = (uint8_t)(clampf(f4, 0.0f, 1.0f) * extra + 0.5f);
				dutyCh3 = lookupTable[v3];
				dutyCh4 = lookupTable[v4];
			}
		}
		else if (actBrightness > actHsv.v)
		{
			uint8_t extra = (uint8_t)constrain((int)actBrightness - (int)actHsv.v, 0, 255);
			dutyCh3 = lookupTableTwBulb[extra];
			dutyCh4 = (uint16_t)(tempFrac01() * 1023.0f + 0.5f);
		}
		else
		{
			dutyCh3 = 0;
			dutyCh4 = 0;
		}
		ledAnalogWrite(3, dutyCh3);
		ledAnalogWrite(4, dutyCh4);
	}
}

void KnxLed::ledAnalogWrite(byte channel, uint16_t duty)
{
	#ifdef DEBUG
	Serial.print("Analog Write: "); Serial.print(channel); Serial.print(" / "); Serial.print("X"); Serial.print(" / "); Serial.println(duty);
	#endif
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
	if (initialized) return; // Schutz gegen Re-Init
	// Channels für diese Instanz reservieren (instanzlokal in esp32LedCh[])
	if (!allocateLedc(usedChannels, esp32LedCh)) {
		#ifdef DEBUG
		Serial.println("LEDC allocation failed");
		#endif
		initialized = false;
		return;
	}

	// Setup + Attach pro Pin/Channel dieser Instanz
	for (uint8_t i = 0; i < usedChannels; i++) {
		ledcSetup(esp32LedCh[i], pwmFrequency, pwmResolution);
		ledcAttachPin(outputPins[i], esp32LedCh[i]);
	}

	initialized = true;
	#ifdef DEBUG
	Serial.printf("LEDC channels: ");
	for (uint8_t i=0;i<usedChannels;i++) Serial.printf("%d ", (int)esp32LedCh[i]);
	Serial.println();
	#endif
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
	initialized = true;
#endif
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