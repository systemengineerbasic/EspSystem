#include "BluetoothSerial.h"
#include "my_cmd.h"

BluetoothSerial SerialBT;
Stream* g_pSerial=&Serial;

char    g_command_line[256];
int     g_cmd_index = 0;


//=====================================
// Command procedures
//=====================================
void _cmd__test(int argc, char* argv[])
{
    g_pSerial->println("Test OK.");
}

void _cmd__add(int argc, char* argv[])
{
    if(argc > 2) {
        int val1 = atoi(argv[1]);
        int val2 = atoi(argv[2]);
        char txt[128];
        sprintf(txt, "%d + %d = %d", val1, val2, val1+val2);
        g_pSerial->println(txt);
    }
    else {
        g_pSerial->println("argument error");
    }
}

void _cmd__serial(int argc, char* argv[])
{
    if(argc > 1) {
        if(strcmp(argv[1], "bt")==0) {
            g_pSerial = &SerialBT;
        }
        else if(strcmp(argv[1], "usb")==0) {
            g_pSerial = &Serial;
        }
    }
        
}

//=====================================
// Command table
//=====================================
T_command_info  g_command_table[] = {
    {"test",        _cmd__test},
    {"add",         _cmd__add},
    {"serial",      _cmd__serial},
    // The last line must be NULL
    {NULL,          NULL},
};


void setup() 
{
    // Initialize serial-port (115200bps)
    Serial.begin(115200);

    // Initialize Bluetooth
	SerialBT.begin("ESP32-xxxxx");
	
	// Select standard I/O
    //g_pSerial = &SerialBT;
    g_pSerial = &Serial;
}

void loop()
{
    if(g_pSerial->available() > 0) { // received data
        char getstr = g_pSerial->read(); // Read data from serial-port
        g_command_line[g_cmd_index] = getstr;
        g_cmd_index ++;
        if(getstr == '\n') { // Detect "LF"(enter key)
            g_command_line[g_cmd_index-1] = '\0';
            g_cmd_index = 0;
            
            // Parse and execute command
            int err = parse_and_exec_cmd(g_command_line, g_command_table);
            if(err == 0) {
                g_pSerial->println("[Error] cannot find command.");
            }
        }
    }

}


