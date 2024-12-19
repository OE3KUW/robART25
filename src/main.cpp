/**************************************************************************************
                                                              
                                r o b A R T 2 5
                                                                                 
 3xHELS 24/25                                                              қuran 2025
**************************************************************************************/
#include <Arduino.h>
#include <EEPROM.h>
#define FASTLED_ALL_PINS_HARDWARE_SPI
#include <FastLED.h>

#define TRUE                             1
#define FALSE                            0
#define WAIT_ONE_SEC                 10000
#define ON_BOARD_LED                     5
#define DAC                             25   // Trig
#define WHEEL_L                          2
#define WHEEL_R                         A4
#define WHEEL_L_DIRECTION               15 
#define WHEEL_R_DIRECTION               A5
#define BATTERY_LEVEL                   A3   // GPIO 39
#define REFV                           685.0 // factor
#define DEEP_SLEEP_DURATION           10e6   // 10 Sekunden in Mikrosekunden

#define EEPROM_SIZE                    100
#define EEPROM_ADDR                      0
#define EEPROM_STATE                     0 
#define EEPROM_MOTOR_SYS_ADDR           10
#define EEPROM_SSID_ADDR                20
#define EEPROM_PASSWORD_ADDR            60

#define ESC                             27
#define NUM_LEDS                         4
#define DATA_PIN                        23
#define CLOCK_PIN                       18
#define N                               42

#define STATE_SLEEP                      0
#define STATE_START                      1
#define STATE_DRIVE                      2
#define STATE_MAX                        3


void IRAM_ATTR myTimer(void);
void enterDeepSleep();

volatile int flag; 
volatile int state; 
volatile int vL, vR;
volatile int LDir;
volatile int RDir;
volatile float batteryLevel = 0.;
volatile int deltaT = 0;

String receivedText       = ""; // string buffer
String ssidFromEEPROM     = "";
String passwordFromEEPROM = "";
String motorSysFromEEPROM = "";
String receivedWord       = "";

volatile int motorSys = 0;


hw_timer_t *timer = NULL;
CRGB leds[NUM_LEDS];

String readFromEEPROM(int address);
void store2EEPROM(String word, int address);


void setup() 
{

    pinMode(ON_BOARD_LED, OUTPUT);
    pinMode(DAC, OUTPUT);
    pinMode(WHEEL_L, OUTPUT);
    pinMode(WHEEL_R, OUTPUT);
    pinMode(WHEEL_L_DIRECTION, OUTPUT);
    pinMode(WHEEL_R_DIRECTION, OUTPUT);
    pinMode(BATTERY_LEVEL, INPUT);

    digitalWrite(ON_BOARD_LED, LOW); // invers logic!
    digitalWrite(WHEEL_L_DIRECTION, LOW );
    digitalWrite(WHEEL_R_DIRECTION, HIGH);
    digitalWrite(WHEEL_L, LOW); // stop !
    digitalWrite(WHEEL_R, LOW); // stop !

    vR = vL = 0;
    
    LDir = 0;
    RDir = 1;




    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &myTimer, true);
    timerAlarmWrite(timer, 100, true);  // 0.1 msec
    timerAlarmEnable(timer);

    Serial.begin(115200);


    // Fast Leds:  switch off all Leds!

    FastLED.addLeds<SK9822, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);

    leds[0] = CRGB{0, 0, 0}; // R B G
    leds[1] = CRGB{0, 0, 0};
    leds[2] = CRGB{0, 0, 0};
    leds[3] = CRGB{0, 0, 0};

    FastLED.show();

    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("EEPROM initialisieren fehlgeschlagen!");
        return;
    }

    state = EEPROM.read(EEPROM_STATE);
    
    if (state > STATE_MAX) state = STATE_START;

    ssidFromEEPROM = readFromEEPROM(EEPROM_SSID_ADDR);
    printf("im EEPROM gefunden: .%s.\n", ssidFromEEPROM);
    passwordFromEEPROM = readFromEEPROM(EEPROM_PASSWORD_ADDR);
    printf("im PASSWORD gefunden: .%s.\n", passwordFromEEPROM);
    motorSysFromEEPROM = readFromEEPROM(EEPROM_MOTOR_SYS_ADDR);
    motorSys = (char)motorSysFromEEPROM[0] - '0';
    
    printf("MotorSystem: .%d.\n", motorSys);
    printf("battery level: %f\n", batteryLevel);
//    printf("SSID: .%s.\n", ssidFromEEPROM);
//    printf("PWD: .%s.\n", passwordFromEEPROM);


    printf("-----Don't forget to set the cursor in the monitor window----------\n");
}  

void loop() 
{
    static int led = 0;
    char c;
    int i, j;
    String data;
    int startIndex, endIndex;

    
    switch (state)
    {
        case STATE_SLEEP:
            data = ""; 
            batteryLevel = 0;
            for(i = 0; (i < N); i++) 
            {   
                for(j = N - i; (j > 0); j--) printf("z");
                printf(" press ESC to wake me up and wait!    "); 
                for(j = 0;      (j < i); j++) printf("z");
                printf("\r");
                batteryLevel += analogRead(BATTERY_LEVEL) / REFV;
                delay(60);
            }           
            batteryLevel /= N; 

            if (Serial.available() > 0) {
                data = Serial.readString();
                data.trim();
                printf("received: %s (=%d)\n", data, data[0]);  
            }
            if (data[0] != ESC)
            {
//                batteryLevel = analogRead(BATTERY_LEVEL) / REFV;
                printf("battery:%1.3fV\n", batteryLevel);
                enterDeepSleep();
            }    
            else
            {   
                printf("---- wake up! ----\n");
                state = STATE_START;
                EEPROM.write(EEPROM_STATE, state);
                EEPROM.commit();  // Änderungen speichern
                esp_restart();
            }
        break;
        case STATE_START:
             printf("\n!stART!\n\n");
             printf("To enter sleep mode, write SLEEP via U-ART!\n");
             printf("First, set the cursor in the monitor window!\n");
             vL = vR = 0;
             state = STATE_DRIVE;
        break;
        case STATE_DRIVE:
        break; 
    }

    if (flag)
    {
        flag = 0; 
        batteryLevel = analogRead(BATTERY_LEVEL) / REFV;
//###     printf("robART25 vL= %03d vR = %03d battery: %1.3fV\n", vL, vR, batteryLevel);
    }

// add sleep!
    while (Serial.available() > 0) 
    {
        char receivedChar = Serial.read(); // Einzelnes Zeichen lesen

        // Wenn ein Zeilenumbruch empfangen wird: Ausgabe und Text zurücksetzen
        if (receivedChar == '\n') 
        {
            Serial.println("Empfangene Daten: ." + receivedText); 
            receivedText = ""; // Textfeld zurücksetzen
        } 
        else 
        {
            receivedText += receivedChar; // Zeichen an das Textfeld anhängen
            receivedText.trim(); // Leerzeichen am Anfang und am Ende werden 
                                 // entfernt
            startIndex = 0; 
            startIndex = receivedText.indexOf("\"ssid\":\"");
                //"ssid":"   das sind 8 Zeichen!
            if (startIndex != -1)
            {
                startIndex += 8;
                endIndex = receivedText.indexOf("\"", startIndex);
                
                if (endIndex != -1)
                {
                    receivedWord = receivedText.substring(startIndex, endIndex);
                    printf("\nUserWort erkannt .%s.", receivedWord);
                    store2EEPROM(receivedWord, EEPROM_SSID_ADDR);
                }
            }

            startIndex = receivedText.indexOf("\"password\":\"");
                //"password":"   das sind 12 Zeichen!
            if (startIndex != -1)
            {
                startIndex += 12;
                endIndex = receivedText.indexOf("\"", startIndex);
            
                if (endIndex != -1)
                {
                    receivedWord = receivedText.substring(startIndex, endIndex);
                    printf("\nPassword erkannt .%s.", receivedWord);
                    store2EEPROM(receivedWord, EEPROM_PASSWORD_ADDR);
                }
            }


            startIndex = receivedText.indexOf("\"motor-system\":\"");
                //"motor-system":"   das sind 15 Zeichen!
            if (startIndex != -1)
            {
                startIndex += 16; // wegen dem '-' Zeichen ? 
                endIndex = receivedText.indexOf("\"", startIndex);
            
                if (endIndex != -1)
                {
                    receivedWord = receivedText.substring(startIndex, endIndex);
                    printf("\nMotorsystem erkannt .%s.", receivedWord);
                    store2EEPROM(receivedWord, EEPROM_MOTOR_SYS_ADDR);
                }
            }

            if (receivedText == "SYS") 
            {
                printf("MotorSystem: .%d.\n", motorSys);
                printf("battery level: %f\n", batteryLevel);
                printf("SSID: .%s.\n", ssidFromEEPROM);
                printf("PWD: .%s.\n", passwordFromEEPROM);
                deltaT = 200; 
                while(deltaT);
                printf(".\n");

            }


            if (receivedText == "RESET") 
            {
                printf("----> Reboot!\n");
                deltaT = 200; 
                while(deltaT);
                ESP.restart(); // Neustart des ESP32
            }
            if (receivedText == "SLEEP") 
            {
                printf(" z z z \n");
                state = STATE_SLEEP;
                EEPROM.write(EEPROM_STATE, state);
                EEPROM.commit();  // Änderungen speichern
                enterDeepSleep();
            }
        }
    }
}



/// @brief The systems reduces all current cunsumtion to a minimum!
void enterDeepSleep() {
    // Alle LEDs ausschalten
    digitalWrite(ON_BOARD_LED, HIGH);  // Negative Logik
    timerEnd(timer);  // stop Overflow Timer Interrupt
    // Deep Sleep einleiten
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION);
    esp_deep_sleep_start();
}

String readFromEEPROM(int address)
{
  String word = "";
  char c;
  while ((c = EEPROM.read(address++)) != '\0') 
  { // Lies Zeichen bis zur Null-Terminierung
    word += c;
  }
  return word;
}

/// @brief store a string into the EEPROM Area
/// @param word = any string
/// @param address = start address
void store2EEPROM(String word, int address)
{
    int i;

    for (i = 0; i < word.length(); i++)
    {
        EEPROM.write(address + i, word[i]);
    }
    EEPROM.write(address + word.length(), '\0');    
    EEPROM.commit();  // Änderungen speichern
}


/// @brief Timer Overflow Interrupt: expires each 0.1 msec
void IRAM_ATTR myTimer(void)   
{
    static int32_t count  = 0;
    static unsigned char ramp = 0;

    count++;
    ramp++;

    // dacWrite(DAC, ramp);

    if (deltaT) deltaT--;

    if (count >= WAIT_ONE_SEC) 
    {

        flag = TRUE;
        count = 0;
    }



 // PWM:
    if (motorSys == 0) 
    {
        if (ramp > vL) digitalWrite(WHEEL_L, LOW);  else digitalWrite(WHEEL_L, HIGH);
        if (ramp > vR) digitalWrite(WHEEL_R, LOW);  else digitalWrite(WHEEL_R, HIGH);
    }
    if (motorSys == 1)
    { 
        if (LDir) if (ramp < vL) digitalWrite(WHEEL_L, LOW);  else digitalWrite(WHEEL_L, HIGH);
        else      if (ramp > vL) digitalWrite(WHEEL_L, LOW);  else digitalWrite(WHEEL_L, HIGH);

        if (RDir) if (ramp < vR) digitalWrite(WHEEL_R, LOW);  else digitalWrite(WHEEL_R, HIGH);
        else      if (ramp > vR) digitalWrite(WHEEL_R, LOW);  else digitalWrite(WHEEL_R, HIGH);

    }
}