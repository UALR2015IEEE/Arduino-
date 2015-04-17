/* -*- C++ -*- */

//Includes
#include <NewPing.h>
#include <TFT.h>
#include <math.h>
#include <TouchScreen.h>
#include "HughesyShiftBrite.h"

// setup TFT pins
#define YP A2   // must be an analog pin, use "An" notation!
#define XM A1   // must be an analog pin, use "An" notation!
#define YM 54   // can be a digital pin, this is A0
#define XP 57   // can be a digital pin, this is A3

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);



//function declarations
void exec_command();
int get_int(int chars = 1);
boolean read_serial();

void boot();
void wait4start();


bool button = False;


byte readbyte;
byte writebyte;
boolean command_stat;
byte dir;
byte move_blocks;



NewPing* sonlf;
NewPing* sonrf;
NewPing* sonlr;
NewPing* sonrr;
NewPing* sonff;

NewPing* sensors[5];

HughesyShiftBrite* sb;

void setup(){
    Tft.init();  //init TFT library
    boot();
    Serial1.begin(115200);
    Serial2.begin(115200);
    Serial.begin(115200);
    Serial.flush();
    Serial1.flush();
    Serial2.flush();
    set_speed(zero, zero);

    pinMode(44, OUTPUT);
    digitalWrite(44, HIGH);
    sb = new HughesyShiftBrite(52, 50, 48, 46);
    sb->sendColour(0, 0, 1023);

    sonlf = new NewPing(23, 22, 50);
    sonrf = new NewPing(25, 24, 50);
    sonlr = new NewPing(27, 26, 50);
    sonrr = new NewPing(31, 30, 50);
    sonff = new NewPing(29, 28, 215);
    
    sensors[slf] = sonlf;
    sensors[srr] = sonrr;
    sensors[sff] = sonff;
    sensors[slr] = sonlr;
    sensors[srf] = sonrf;
    wait4start();
}

void loop() {
    button = ts.pressure() < ts.pressureThreshhold;
    if(button) {
        set_speed(2048, 2048);
        while (true);
    }
    //set_speed_angle(80.0, 2.0);
    if (Serial.available()){
        command_stat = read_serial();
    }
}
void boot(){
    Tft.drawString("UALR",0,25,4,RED);
    Tft.drawString("Robotics",25,80,3,GREEN);
    Tft.drawString("^_^",30 ,150,8,BLUE);
    Tft.drawSting("booting..." 0, 210, 5, YELLOW);
    delay(1000);
}


void wait4start(){
    for(int x = 0; i < 240; x++)
        for(int y = 210; i < 320; y++){
            Tft.setPixel(x, y, YELLOW);
        }
    Tft.drawString("START", 35, 220, 5, GREEN);
    while(ts.pressure() < ts.pressureThreshhold);
    for(int x = 0; i < 240; x++)
        for(int y = 210; i < 320; y++){
            Tft.setPixel(x, y, YELLOW);
        }
    Tft.drawString("STOP", 50, 220, 5, RED);
    button = true;
}



void exec_command(){
    set_speed(cruise, cruise);
    while(true){
        time = millis();
        stablize();
        set_speed(target_left, target_right);
    }
}

int get_int(int chars){
    while(!Serial.available());
    char encoded = Serial.read();
    return encoded - 48;
}



bool read_serial(){
    if (Serial.available()){
        readbyte = Serial.read();
        Serial.println(readbyte);

        if (readbyte == 63){
          Serial.println("get pos");
            return false;
        }
        if (readbyte == 33) {
            move_blocks = 0;
            dir = 0;
            if (Serial.available()) {
                move_blocks = get_int();
            }
            Serial.print("move:");
            Serial.println(move_blocks);
            return true;
        }
        if (readbyte == 64){
            if (Serial.available()){
                dir = get_int();
                Serial.print("dir:");
                Serial.println(dir);
            }
            return true;
        }
        if (readbyte == 35){
            Serial.print("button");
            Serial.print(button);
          return false;
        }
        if (readbyte == 94){
            int com = get_int();
            Serial.print("light");
	        Serial.println(com);
            if (com == 0)
                sb->sendColour(0, 0, 1023);
            if (com == 1)
                sb->sendColour(0, 1023, 0);
            if (com == 2)
                sb->sendColour(1023, 0, 0);
            return false;
        }
        else{
            return false;
        }
    }
}
