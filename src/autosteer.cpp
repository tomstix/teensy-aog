#include "main.h"
#include "autosteer.h"
#include "sensors.h"

#include <AutoPID.h>

#define THRESHOLD_PTO 200
#define THRESHOLD_HIT 40

AutoPID pid(&steerSetpoints.actualSteerAngle,
            &steerSetpoints.requestedSteerAngle,
            &steerSetpoints.pidOutput,
            -steerSettings.highPWM, steerSettings.highPWM,
            (double)steerSettings.Kp,
            steerSettings.Ki,
            steerSettings.Kd);

void driveMotor(uint8_t pwm, double dir)
{
    if ( steerConfig.MotorDriveDirection )
    {
        dir = -dir;
    }

    digitalWrite(PIN_ENA, HIGH);
    digitalWrite(PIN_ENB, HIGH);
    if (dir > 0)
    {
        digitalWrite(PIN_INA, HIGH);
        digitalWrite(PIN_INB, LOW);
    }
    else if (dir < 0)
    {
        digitalWrite(PIN_INA, LOW);
        digitalWrite(PIN_INB, HIGH);
    }
    else
    {
        digitalWrite(PIN_INA, LOW);
        digitalWrite(PIN_INB, LOW);
    }

    analogWrite(PIN_PWM, pwm);
}

void autosteerWorker(void *arg)
{
    const TickType_t xInterval = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    pid.setTimeStep(10);
    while (1)
    {
        pid.setGains((double)steerSettings.Kp, steerSettings.Ki, steerSettings.Kd);
        pid.setOutputRange(-steerSettings.highPWM, steerSettings.highPWM);
        vTaskDelayUntil(&xLastWakeTime, xInterval);

        if (steerConfig.wasType == SteerConfig::WASType::ADS1115)
        {
            int16_t counts = adsWorker();

            steerSetpoints.wasCountsRaw = counts;

            if (steerConfig.InvertWAS)
            {
                float wasTemp = (counts - 20000 - steerSettings.wasOffset);
                steerSetpoints.actualSteerAngle = (float)(wasTemp) / -steerSettings.steerSensorCounts;
            }
            else
            {
                float wasTemp = (counts - 20000 + steerSettings.wasOffset);
                steerSetpoints.actualSteerAngle = (float)(wasTemp) / steerSettings.steerSensorCounts;
            }

            if (steerSetpoints.actualSteerAngle < 0)
                steerSetpoints.actualSteerAngle = (steerSetpoints.actualSteerAngle * steerSettings.AckermanFix);

            steerSetpoints.csenseRaw = analogRead(PIN_CSENSE);

            if ( steerConfig.CurrentSensor )
            {
                steerSetpoints.csenseRaw = analogRead(PIN_CSENSE);
                double amps = (steerSetpoints.csenseRaw / 0.31) * 140;
                if ( amps > steerConfig.PulseCountMax )
                {
                    switches.steerSwitch = 1;
                    steerSetpoints.guidanceStatus = 0;
                }
            }

            if (steerSetpoints.guidanceStatus && (steerSetpoints.speed > 0.5) && (millis() - steerSetpoints.lastPacketReceived < 500))
            {
                pid.run();

                uint8_t absPID = abs(steerSetpoints.pidOutput);

                if (absPID < steerSettings.lowPWM)
                {
                    absPID = 0;
                }
                else if (absPID < steerSettings.minPWM)
                {
                    absPID = steerSettings.minPWM;
                }

                driveMotor(absPID, steerSetpoints.pidOutput);
            }
            else
            {
                pid.reset();
                digitalWrite(PIN_ENA, LOW);
                digitalWrite(PIN_ENB, LOW);
            }

            //Serial.println(absPID);

            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void switchWorker(void *arg)
{
    while (1)
    {
        if (steerConfig.workswitchType == SteerConfig::WorkswitchType::Hitch)
        {
            switches.workSwitch = (isobusData.rearHitchPosition > THRESHOLD_HIT);
        }
        else if (steerConfig.workswitchType == SteerConfig::WorkswitchType::PTO)
        {
            switches.workSwitch = (isobusData.rearPtoRpm > THRESHOLD_PTO);
        }
        else
            switches.workSwitch = 1;

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void initAutosteer()
{
    if ((steerConfig.outputType == SteerConfig::OutputType::PWM) || (steerConfig.outputType == SteerConfig::OutputType::PWM2))
    {
        pinMode(PIN_ENA, OUTPUT);
        pinMode(PIN_ENB, OUTPUT);
        pinMode(PIN_INA, OUTPUT);
        pinMode(PIN_INB, OUTPUT);
        pinMode(PIN_PWM, OUTPUT);
        pinMode(PIN_CSENSE, INPUT);
        analogWriteFrequency(PIN_PWM, 5000);
        xTaskCreate(autosteerWorker, NULL, 2048, NULL, 2, NULL);
    }

    xTaskCreate(switchWorker, NULL, 1024, NULL, 2, NULL);
}