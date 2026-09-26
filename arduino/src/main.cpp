#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>
#include "AcceleratedRotaryEncoder.h"
constexpr uint8_t kLcdAddress = 0x27;
constexpr uint8_t kLcdColumns = 20;
constexpr uint8_t kLcdRows = 4;
constexpr uint8_t kDACdAddress = 0x50;
constexpr time_t kBounceGuardDuration = 200; // in milliseconds

// DAC Control bits and registers
constexpr uint8_t kDACRateRegister = 0;
constexpr uint8_t kDACFifoRegister = 1;
constexpr uint8_t kDACStatusRegister = 2;
constexpr uint8_t kDACControlRegister = 3;
constexpr uint8_t kDACASRCErrorRegister0 = 4;
constexpr uint8_t kDACASRCErrorRegister1 = 5;
constexpr uint8_t kDACAdjustmentRegister0 = 6;
constexpr uint8_t kDACAdjustmentRegister1 = 7;
constexpr uint8_t kDACAdjustmentRegister2 = 8;
constexpr uint8_t kDACAdjustmentRegister3 = 9;
constexpr uint8_t kDACVolumetRegister = 12;


// Control bits for the DAC
constexpr uint8_t kDACControlDither = 0; 
constexpr uint8_t kDACControlTestTone = 1;
constexpr uint8_t kDACControlInhibitInterpolation = 2;
constexpr uint8_t kDACControlOutputMute = 3;
constexpr uint8_t kDACControlInputMute = 4;

AcceleratedRotaryEncoder volumeKnob(1, 4); // Pins 1 and 4 for the rotary encoder

typedef struct DACStatus
{
    float fs;
    uint8_t fifoFullness;
    bool rateLocked;
    bool ditherEnabled;
    bool testToneEnabled;
    bool inhibitInterpolation;
    bool inputMute;
    bool outputMute;
    float volume;
    int32_t asrcAdjustment;
    int16_t asrcError;
} DACStatus;

LiquidCrystal_I2C lcd(kLcdAddress, kLcdColumns, kLcdRows);

OneButton ditherButton(3, true); // Pin 3, active low
bool ditherEnabled = true;
bool forceDisplayUpdate = true;

// Increments locally with no I2C involved, isolates Arduino-side bugs from bus/DAC issues.
uint16_t debugCounter = 0;

void printDACStatus(const DACStatus &status)
{
    lcd.setCursor(0, 0);
    lcd.print("Dit: " + String(status.ditherEnabled ? "On " : "Off"));
    lcd.setCursor(10, 0);
    lcd.print("FIR: " + String(status.inhibitInterpolation ? "Off" : "On "));
//    lcd.setCursor(0, 1);
//    lcd.print("DAC Status: OK");
    lcd.setCursor(0, 1);
    lcd.print((status.rateLocked ? String(status.fs, 1) : "--.-") + "kHz");
    lcd.setCursor(10, 1);
    lcd.print("FIFO: " + String(status.fifoFullness) + "% ");
    lcd.setCursor(7, 3);
    lcd.print(String(status.volume, 2) + "dB  ");
    lcd.setCursor(0, 2);
    char bar[21];
    int8_t barLength = (80 + status.volume) / 4;
    if (barLength < 0) barLength = 0;
    if (barLength > 20) barLength = 20;
    for (int i = 0; i < 20; i++) {
        bar[i] = i < barLength ? (char) 255 : ' ';
    }
    bar[20] = '\0';
    lcd.print(bar);
}

bool readDACRegister(uint8_t reg, uint8_t &value)
{
    Wire.beginTransmission(kDACdAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
    {
        Serial.printf("Failed to select DAC reg %02X\n", reg);
        return false;
    }
    if (Wire.requestFrom(kDACdAddress, (uint8_t)1) != 1)
    {
        return false;
    }
    if (Wire.available() == 1)
    {
        value = Wire.read();
        Serial.printf("Read byte from reg %02X: %02X\n", reg, value);
        return true;
    }
    Serial.printf("Failed to read byte from reg %02X\n", reg);
    return false;
}

bool writeDACRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(kDACdAddress);
    Wire.write(reg);
    Wire.write(value);
    if (Wire.endTransmission() != 0)
    {
        Serial.printf("Failed to write byte to reg %02X\n", reg);
        return false;
    }
    Serial.printf("Wrote byte to reg %02X: %02X\n", reg, value);
    return true;
}

void readDACStatus(DACStatus &status)
{
    uint8_t fsByte, fifoByte, controlByte, adjustment0, adjustment1, adjustment2, adjustment3, asrcError0, asrcError1, volumeByte;
    if(readDACRegister(kDACVolumetRegister, volumeByte)) {
        status.volume = -((float)volumeByte)/2;
    }
    if (readDACRegister(0, fsByte))
    {
        status.fs = fsByte & 0x80 ? 48.0 * float(fsByte & 0x7) : 44.1 * float(fsByte & 0x7);
    }
    if(readDACRegister(1, fifoByte)) {
        status.fifoFullness = fifoByte / 255.0 * 100;
    }
    
    status.rateLocked = fsByte != 0;
    
    if(readDACRegister(kDACControlRegister, controlByte)) { 
        status.ditherEnabled = (controlByte & (1 << kDACControlDither)) != 0;
    }
    if(readDACRegister(kDACControlRegister, controlByte)) {
        status.testToneEnabled = (controlByte & (1 << kDACControlTestTone)) != 0;
        status.inhibitInterpolation = (controlByte & (1 << kDACControlInhibitInterpolation)) != 0;
        status.inputMute = (controlByte & (1 << kDACControlInputMute)) != 0;
        status.outputMute = (controlByte & (1 << kDACControlOutputMute)) != 0;
    }
    if(readDACRegister(kDACAdjustmentRegister0, adjustment0) &&
       readDACRegister(kDACAdjustmentRegister1, adjustment1) &&
       readDACRegister(kDACAdjustmentRegister2, adjustment2) &&
       readDACRegister(kDACAdjustmentRegister3, adjustment3))
    {
        status.asrcAdjustment = (uint32_t(adjustment0) ) |
                                (uint32_t(adjustment1) << 8) |
                                (uint32_t(adjustment2) << 16) |
                                (uint32_t(adjustment3) << 24);
    }
    if(readDACRegister(kDACASRCErrorRegister0, asrcError0) &&
       readDACRegister(kDACASRCErrorRegister1, asrcError1))
    {
        status.asrcError = (uint16_t(asrcError0) ) |
                           (uint16_t(asrcError1) << 8);
    }
}

void updateDACControl(uint8_t controlValue, uint8_t updateMask)
{
    uint8_t currentValue;
    if (readDACRegister(kDACControlRegister, currentValue))
    {
        uint8_t newValue = (currentValue & ~updateMask) | (controlValue & updateMask);
        writeDACRegister(kDACControlRegister, newValue);
    }
}

void setVolume(uint8_t volume)
{
    writeDACRegister(kDACVolumetRegister, volume);
}

void setDither(bool enable)
{
    updateDACControl(enable ? (1 << kDACControlDither) : 0, 1 << kDACControlDither);
}

void setTestTone(bool enable)
{
    updateDACControl(enable ? (1 << kDACControlTestTone) : 0, 1 << kDACControlTestTone);
}

void setInhibitInterpolation(bool enable)
{
    updateDACControl(enable ? (1 << kDACControlInhibitInterpolation) : 0, 1 << kDACControlInhibitInterpolation);
}

void setInputMute(bool enable)
{
    updateDACControl(enable ? (1 << kDACControlInputMute) : 0, 1 << kDACControlInputMute);
}

void setOutputMute(bool enable)
{
    updateDACControl(enable ? (1 << kDACControlOutputMute) : 0, 1 << kDACControlOutputMute);
}

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);

    // Make pin 3 an input with an internal pull-up resistor
    pinMode(4, INPUT_PULLUP);
    pinMode(1, INPUT_PULLUP); 
    pinMode(3, INPUT_PULLUP); // For the rotary encoder

    ditherButton.attachClick([]() {
        ditherEnabled = !ditherEnabled;
        setDither(ditherEnabled);
        forceDisplayUpdate = true;
    });

    volumeKnob.setMaxPosition(255);
    volumeKnob.setMinPosition(0);
    volumeKnob.setPosition(128);

    setDither(ditherEnabled);
    setVolume(128);
}

void loop()
{
    static DACStatus status = {0.0, 0, false, false, false, false, false};
    static time_t lastUpdate = 0;
    long volume;

    // Tick the controls
    ditherButton.tick();
    volumeKnob.tick();

    if (volumeKnob.getNewPosition(volume)) {
        setVolume(volume);
        forceDisplayUpdate = true;
    }

    // Time to update?
    if (forceDisplayUpdate || (millis() - lastUpdate >= 1000))
    {
        readDACStatus(status);
        printDACStatus(status);

        uint8_t low, high;
        readDACRegister(10, low);
        readDACRegister(11, high);
        uint16_t servoFifo =
            uint16_t(low) |
            (uint16_t(high) << 8);

        forceDisplayUpdate = false;
        lastUpdate = millis();
        Serial.printf("ASRC Error: %d, ASRC Adjustment: %d, FIFO: %u, Servo FIFO: %u\n", status.asrcError, status.asrcAdjustment, status.fifoFullness, servoFifo);
        Serial.printf("FIFO servo bytes: %02X %02X\n", low, high);

        uint8_t lo, hi;
        readDACRegister(0x0C, lo);
        readDACRegister(0x0D, hi);
        uint16_t debugCounter = uint16_t(lo) | (uint16_t(hi) << 8);
        Serial.printf("Debug Counter: %04x\n", debugCounter);
    }
}