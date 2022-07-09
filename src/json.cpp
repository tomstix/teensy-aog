#include "json.h"

#include <ArduinoJson.h>
#include <SD.h>
#include <SPI.h>

StaticJsonDocument<2048> steerSettingsJson;
StaticJsonDocument<2048> steerConfigJson;

const char *steerConfigFile = "steerConfig.txt";
const char *steerSettingsFile = "steerSettings.txt";

Threads::Mutex sdCardMutex;

void printSettings()
{
    serializeJsonPretty(steerConfigJson, Serial);
    serializeJsonPretty(steerSettingsJson, Serial);
}

void loadSteerConfig()
{
    sdCardMutex.lock();
    File file = SD.open(steerConfigFile);

    DeserializationError error = deserializeJson(steerConfigJson, file);

    if (error)
        Serial.println("Failed to read steer config file, using default configuration");

    steerConfig.InvertWAS = steerConfigJson["InvertWAS"] | 0;
    steerConfig.IsRelayActiveHigh = steerConfigJson["IsRelayActiveHigh"] | 0;
    steerConfig.MotorDriveDirection = steerConfigJson["MotorDriveDirection"] | 0;
    steerConfig.SingleInputWAS = steerConfigJson["SingleInputWAS"] | 0;
    steerConfig.CytronDriver = steerConfigJson["CytronDriver"] | 0;
    steerConfig.SteerSwitch = steerConfigJson["SteerSwitch"] | 0;
    steerConfig.SteerButton = steerConfigJson["SteerButton"] | 0;
    steerConfig.ShaftEncoder = steerConfigJson["ShaftEncoder"] | 0;
    steerConfig.PressureSensor = steerConfigJson["PressureSensor"] | 0;
    steerConfig.CurrentSensor = steerConfigJson["CurrentSensor"] | 0;
    steerConfig.PulseCountMax = steerConfigJson["PulseCountMax"] | 0;
    steerConfig.IsDanfoss = steerConfigJson["IsDanfoss"] | 0;

    //IMU Type
    /*if (steerConfigJson["imuType"] == "CMPS14")
    {
        steerConfig.imuType = SteerConfig::ImuType::CMPS14;
    }
    else if (steerConfigJson["imuType"] == "BNO085")
    {
        steerConfig.imuType = SteerConfig::ImuType::BNO085;
    }
    else
        steerConfig.imuType = SteerConfig::ImuType::None;*/

    //WAS Type
    if (steerConfigJson["wasType"] == "ADS1115")
    {
        steerConfig.wasType = SteerConfig::WASType::ADS1115;
    }
    else if (steerConfigJson["wasType"] == "CAN")
    {
        steerConfig.wasType = SteerConfig::WASType::CAN;
    }
    else
        steerConfig.wasType = SteerConfig::WASType::None;

    //Output Type
    if (steerConfigJson["outputType"] == "PWM")
    {
        steerConfig.outputType = SteerConfig::OutputType::PWM;
    }
    else if (steerConfigJson["outputType"] == "PWM2")
    {
        steerConfig.outputType = SteerConfig::OutputType::PWM2;
    }
    else if (steerConfigJson["outputType"] == "FendtCAN")
    {
        steerConfig.outputType = SteerConfig::OutputType::FendtCAN;
    }
    else
        steerConfig.outputType = SteerConfig::OutputType::None;

    //Workswitch Type
    if (steerConfigJson["workswitchType"] == "Hitch")
    {
        steerConfig.workswitchType = SteerConfig::WorkswitchType::Hitch;
    }
    else if (steerConfigJson["workswitchType"] == "PTO")
    {
        steerConfig.workswitchType = SteerConfig::WorkswitchType::PTO;
    }
    else
        steerConfig.workswitchType = SteerConfig::WorkswitchType::None;

    file.close();
    sdCardMutex.unlock();
}

void saveSteerConfig()
{
    sdCardMutex.lock();
    SD.remove(steerConfigFile);

    File file = SD.open(steerConfigFile, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to create steerConfig file");
        return;
    }

    steerConfigJson["InvertWAS"] = steerConfig.InvertWAS;
    steerConfigJson["IsRelayActiveHigh"] = steerConfig.IsRelayActiveHigh;
    steerConfigJson["MotorDriveDirection"] = steerConfig.MotorDriveDirection;
    steerConfigJson["SingleInputWAS"] = steerConfig.SingleInputWAS;
    steerConfigJson["CytronDriver"] = steerConfig.CytronDriver;
    steerConfigJson["SteerSwitch"] = steerConfig.SteerSwitch;
    steerConfigJson["SteerButton"] = steerConfig.SteerButton;
    steerConfigJson["ShaftEncoder"] = steerConfig.ShaftEncoder;
    steerConfigJson["PressureSensor"] = steerConfig.PressureSensor;
    steerConfigJson["CurrentSensor"] = steerConfig.CurrentSensor;
    steerConfigJson["PulseCountMax"] = steerConfig.PulseCountMax;
    steerConfigJson["IsDanfoss"] = steerConfig.IsDanfoss;

    //IMU Type
    if (steerConfig.imuType == SteerConfig::ImuType::CMPS14)
    {
        steerConfigJson["imuType"] == "CMPS14";
    }
    else if (steerConfig.imuType == SteerConfig::ImuType::BNO085)
    {
        steerConfigJson["imuType"] = "BNO085";
    }
    else
        steerConfigJson["imuType"] = "None";

    //WAS Type
    if (steerConfig.wasType == SteerConfig::WASType::ADS1115)
    {
        steerConfigJson["wasType"] = "ADS1115";
    }
    else if (steerConfig.wasType == SteerConfig::WASType::CAN)
    {
        steerConfigJson["wasType"] = "CAN";
    }
    else
        steerConfigJson["wasType"] = "None";

    //Output Type
    if (steerConfig.outputType == SteerConfig::OutputType::PWM)
    {
        steerConfigJson["outputType"] = "PWM";
    }
    else if (steerConfig.outputType == SteerConfig::OutputType::PWM2)
    {
        steerConfigJson["outputType"] = "PWM2";
    }
    else if (steerConfig.outputType == SteerConfig::OutputType::FendtCAN)
    {
        steerConfigJson["outputType"] = "FendtCAN";
    }
    else
        steerConfigJson["outputType"] = "None";

    //Workswitch Type
    if (steerConfig.workswitchType == SteerConfig::WorkswitchType::Hitch)
    {
        steerConfigJson["workswitchType"] = "Hitch";
    }
    else if (steerConfig.workswitchType == SteerConfig::WorkswitchType::PTO)
    {
        steerConfigJson["workswitchType"] = "PTO";
    }
    else
        steerConfigJson["workswitchType"] = "None";

    /*if (serializeJson(steerConfigJson, file) == 0)
    {
        Serial.println("Failed to write steerConfig to file");
    }*/

    size_t b = serializeJson(steerConfigJson, file);
    Serial.print("Bytes written: "); Serial.println(b);

    serializeJsonPretty(steerConfigJson, Serial);

    file.close();

    Serial.println("SteerConfig written!");

    sdCardMutex.unlock();

    yield();
}

void loadSteerSettings()
{
    sdCardMutex.lock();
    File file = SD.open(steerSettingsFile);

    Serial.println("Loading Steer Settings");

    DeserializationError error = deserializeJson(steerSettingsJson, file);

    if (error)
        Serial.println("Failed to read steer settings file, using default configuration");

    steerSettings.Kp = steerSettingsJson["Kp"] | 100;
    steerSettings.Ki = steerSettingsJson["Ki"] | 0;
    steerSettings.Kd = steerSettingsJson["Kd"] | 0;
    steerSettings.highPWM = steerSettingsJson["highPWM"] | 60;
    steerSettings.lowPWM = steerSettingsJson["lowPWM"] | 10;
    steerSettings.minPWM = steerSettingsJson["minPWM"] | 9;
    steerSettings.steerSensorCounts = steerSettingsJson["steerSensorCounts"] | 5;
    steerSettings.wasOffset = steerSettingsJson["wasOffset"] | 0;
    steerSettings.AckermanFix = steerSettingsJson["AckermanFix"] | 100;

    file.close();
    sdCardMutex.unlock();
}

void saveSteerSettings()
{
    sdCardMutex.lock();
    SD.remove(steerSettingsFile);

    File file = SD.open(steerSettingsFile, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to create steerSettings file");
        return;
    }

    steerSettingsJson["Kp"] = steerSettings.Kp;
    steerSettingsJson["Ki"] = steerSettings.Ki;
    steerSettingsJson["Kd"] = steerSettings.Kd;
    steerSettingsJson["highPWM"] = steerSettings.highPWM;
    steerSettingsJson["lowPWM"] = steerSettings.lowPWM;
    steerSettingsJson["minPWM"] = steerSettings.minPWM;
    steerSettingsJson["steerSensorCounts"] = steerSettings.steerSensorCounts;
    steerSettingsJson["wasOffset"] = steerSettings.wasOffset;
    steerSettingsJson["AckermanFix"] = steerSettings.AckermanFix;

    if (serializeJson(steerSettingsJson, file) == 0)
    {
        Serial.println("Failed to write steerSettings to file");
    }

    serializeJsonPretty(steerSettingsJson, Serial);

    file.close();
    sdCardMutex.unlock();
    Serial.println("SteerSettings written");
    yield();
}

void initSD()
{
    Serial.println("Initalizing SD Card");
    while (!SD.begin(BUILTIN_SDCARD))
    {
        Serial.println("Failed to initialize SD library");
        delay(1000);
    }
    Serial.println("SD Card initialized.");
}