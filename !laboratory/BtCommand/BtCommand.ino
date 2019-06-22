#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

char    g_command_line[256];
int     g_cmd_index = 0;

struct _command_info{
    char*   cmd;
    int    (*proc)(int argc, char* argv[]);
};

typedef struct _command_info T_command_info;


int    _cmd__speed(int argc, char* argv[])
{
    if(argc > 1) {
        int speed = atoi(argv[1]);
        SerialBT.print("speed = ");
        SerialBT.println(speed);
    }
    
    return  0;
}

int    _cmd__right(int argc, char* argv[])
{
    SerialBT.println("Right turn");
    return  1;
}

T_command_info  g_command_table[] = {
    {"speed",       _cmd__speed},
    {"right",       _cmd__right},
    // The last line must be NULL
    {NULL,          NULL},
};


int parse_cmdline(char* cmd_line, char* argv[])
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
            int argc = parse_cmdline(g_command_line, argv);
            // Search and execute command
            if(argc > 0) {
                for(int i = 0; ; i ++) {
                    if(g_command_table[i].cmd != NULL) {
                        if(strcmp(argv[0], g_command_table[i].cmd) == 0) {
                            g_command_table[i].proc(argc, argv);
                        }
                    }
                    else {
                        // The last line
                        break;
                    }
                }
            }
            
        }
    }

}


