#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

char    g_command_line[256];
int     g_cmd_index = 0;


int parse(char* cmd_line, char* argv[])
{
    int argc = 0;
    char* p = NULL;
    char* str = cmd_line;
    while(1) {
        p = strtok(str, " ");
        if(p != NULL){
            str = NULL;
            argv[argc] = p;
            argc ++;
        }
        else {
            break;
        }
    }
    
    return  argc;
}

void setup() 
{
    // Initialize serial-port (115200bps)
    Serial.begin(115200);

    // Initialize Bluetooth
	SerialBT.begin("ESP32-12135");

}

void loop()
{
    if(SerialBT.available() > 0) { // received data
        char getstr = SerialBT.read(); // Read data from serial-port
        g_command_line[g_cmd_index] = getstr;
        g_cmd_index ++;
        if(getstr == '\n') {
            g_command_line[g_cmd_index-1] = '\0';
            SerialBT.print(" >>");
            SerialBT.println(g_command_line);
            g_cmd_index = 0;
    
            // Parse command line
            char* argv[10];
            int argc = parse(g_command_line, argv);
            
            // Command procedures
            if(strcmp(argv[0], "speed") == 0) {
                if(argc > 1) {
                    int speed = atoi(argv[1]);
                    SerialBT.print("speed = ");
                    SerialBT.println(speed);
                }
            }
            else if(strcmp(argv[0], "right") == 0) {
                SerialBT.println("Right turn");
            }
        }
    }

}


