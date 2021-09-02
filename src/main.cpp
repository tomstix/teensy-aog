#include "main.h"
#include "sensors.h"
#include "can.h"
#include "json.h"
#include "coms.h"
#include "gps.h"
#include "autosteer.h"

#include <SerialCommands.h>
#include <ArduinoJson.h>

SteerSettings steerSettings;
SteerConfig steerConfig;
SteerSetpoints steerSetpoints;
Switches switches;

uint16_t printstatus_freq = 1;
StaticJsonDocument<1024> statusJson;

char serial_command_buffer_[128];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");

void DBG(String msg)
{
    if (DEBUG)
    {
        Serial.print(millis());
        Serial.print("\t");
        Serial.println(msg);
    }
}

void serialWorker(void *arg)
{
    while(1)
    {
        serial_commands_.ReadSerial();

        if ( printstatus_freq )
        {
            statusJson["reqAngle"] = steerSetpoints.requestedSteerAngle;
            statusJson["wasCounts"] = steerSetpoints.wasCountsRaw;
            statusJson["CSENSE Raw"] = steerSetpoints.csenseRaw;
            statusJson["PTOSpeed"] = isobusData.rearPtoRpm;
            statusJson["HitchPos"] = isobusData.rearHitchPosition;
            serializeJsonPretty(statusJson, Serial);
            printSettings();
            vTaskDelay(pdMS_TO_TICKS(1000/printstatus_freq));
        }

        taskYIELD();
    }
}

void heartbeat(void *arg)
{
    while (1)
    {
        //DBG("Test!");
        digitalToggle(9);
        //Serial.print("NTRIP in: "); Serial.println(ntripBytesIn);
        //Serial.print("NTRIP out: "); Serial.println(ntripBytesOut);
        ntripBytesIn = ntripBytesOut = 0;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
	sender->GetSerial()->print("ERROR: Unrecognized command [");
	sender->GetSerial()->print(cmd);
	sender->GetSerial()->println("]");
}

void cmd_hello(SerialCommands* sender)
{
	//Do not use Serial.Print!
	//Use sender->GetSerial this allows sharing the callback method with multiple Serial Ports
	sender->GetSerial()->println("HELLO from arduino!");
}
void cmd_setki(SerialCommands* sender)
{
    char* val_str = sender->Next();
    if (val_str == NULL)
	{
		sender->GetSerial()->println("ERROR NO_VAL");
		return;
	}
    double val = atof(val_str);
    steerSettings.Ki = val;
    sender->GetSerial()->print("Setting Ki to: ");
    sender->GetSerial()->println(steerSettings.Ki);
    saveSteerSettings();
}
void cmd_setkd(SerialCommands* sender)
{
    char* val_str = sender->Next();
    if (val_str == NULL)
	{
		sender->GetSerial()->println("ERROR NO_VAL");
		return;
	}
    double val = atof(val_str);
    steerSettings.Kd = val;
    sender->GetSerial()->print("Setting Kd to: ");
    sender->GetSerial()->println(steerSettings.Kd);
    saveSteerSettings();
}
void cmd_printstatus(SerialCommands* sender)
{
    char* val_str = sender->Next();
    if (val_str == NULL)
	{
		sender->GetSerial()->println("ERROR NO_VAL");
		return;
	}
    printstatus_freq = atoi(val_str);
}

SerialCommand cmd_hello_("hello", cmd_hello);
SerialCommand cmd_setki_("setki", cmd_setki);
SerialCommand cmd_setkd_("setkd", cmd_setkd);
SerialCommand cmd_printstatus_("printstatus", cmd_printstatus);

void setup()
{
    Serial.begin(115200);
    serial_commands_.AddCommand(&cmd_hello_);
    serial_commands_.AddCommand(&cmd_setki_);
    serial_commands_.AddCommand(&cmd_setkd_);
    serial_commands_.AddCommand(&cmd_printstatus_);
    serial_commands_.SetDefaultHandler(&cmd_unrecognized);


    delay(100);
    pinMode(13, OUTPUT);
    pinMode(9, OUTPUT);

    initSD();
    delay(500);
    loadSteerConfig();
    loadSteerSettings();

    initCAN();

    initSensors();

    initEthernet();

    initGPS();

    initAutosteer();

    DBG("Starting Task Scheduler");
    xTaskCreate(heartbeat, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(serialWorker, NULL, 2048, NULL, 2, NULL);
    vTaskStartScheduler();
}

void loop()
{
}