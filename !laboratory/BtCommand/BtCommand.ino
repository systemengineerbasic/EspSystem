#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

char    g_command_line[256];
int     g_cmd_index = 0;

struct _command_info{
    char*   cmd;
    void    (*proc)(int argc, char* argv[]);
};

typedef struct _command_info T_command_info;


//=====================================
// Command procedures
//=====================================
void    _cmd__speed(int argc, char* argv[])
{
    if(argc > 1) {
        int speed = atoi(argv[1]);
        SerialBT.print("speed = ");
        SerialBT.println(speed);
    }
}

void    _cmd__right(int argc, char* argv[])
{
    SerialBT.println("Right turn");
}

//=====================================
// Command table
//=====================================
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

int parse_and_exec_cmd(char* cmdline, T_command_info cmd_table[])
{
    // Parse command line
    char* argv[10];
    int argc = parse_cmdline(cmdline, argv);

    // Search cmd_table and execute the command
    if(argc > 0) {
        for(int i = 0; ; i ++) {
            if(cmd_table[i].cmd != NULL) {
                if(strcmp(argv[0], cmd_table[i].cmd) == 0) {
                    // Find a command in the cmd_table
                    cmd_table[i].proc(argc, argv);
                    break;
                }
            }
            else {
                // The last line
                return 0;
            }
        }
    }
    
    return  1;
}

Stream* g_pSerial=NULL;
void setup() 
{
    // Initialize serial-port (115200bps)
    Serial.begin(115200);

    // Initialize Bluetooth
	SerialBT.begin("ESP32-12135");
	
    g_pSerial = &SerialBT;
}

void loop()
{
    if(SerialBT.available() > 0) { // received data
        char getstr = SerialBT.read(); // Read data from serial-port
        g_command_line[g_cmd_index] = getstr;
        g_cmd_index ++;
        if(getstr == '\n') {
            g_command_line[g_cmd_index-1] = '\0';
            g_cmd_index = 0;
            
            // Parse and execute command
            int err = parse_and_exec_cmd(g_command_line, g_command_table);
            if(err == 0) {
                SerialBT.println("[Error] cannot find command.");
            }

	        SerialBT.print("> ");
        }
    }

}


