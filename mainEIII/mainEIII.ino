/* -*- C++ -*- */

//Includes
#include <NewPing.h>
#include <millis>;
#include <Serial>
//#include <TFT.h>;
#include <math.h>
#include "HughesyShiftBrite.h"

// setup TFT pins
#define YP A2   // must be an analog pin, use "An" notation!
#define XM A1   // must be an analog pin, use "An" notation!
#define YM 54   // can be a digital pin, this is A0
#define XP 57   // can be a digital pin, this is A3

#define slf 0
#define srr 1
#define slr 2
#define srf 3
#define sff 4

//function declarations
float ping_median(NewPing* sensor, float avg, int i, int j);
void stablize();
void rotate();
void exec_command();
void announce_sensors();
int get_int(int chars = 1);
void set_speed(int left, int right);
boolean read_serial();
int bound(int num, int zero, int dead, int top);

//variable declarations
unsigned long time = millis();
unsigned long old_time = time;
unsigned long elapsed = 0;

byte readbyte;
byte writebyte;
boolean command_stat;
byte dir;
byte move_blocks;
const int unit = 12;

unsigned int cruise = 2600;
unsigned int dead_zone = 0;
unsigned int max_speed = 400;
unsigned int zero = 2048;

float r_acc = 0.0;
float r_vel = 0.0;
float r_pos = 0.0;

float l_acc = 0.0;
float l_vel = 0.0;
float l_pos = 0.0;

float a_max = 50.0;

word target_right = 0;  //only pass this ints, i tried doing math in this and the remainder error screwed something up
word target_left = 0;

NewPing* sonlf;
NewPing* sonrf;
NewPing* sonlr;
NewPing* sonrr;
NewPing* sonff;

NewPing* sensors[5];

HughesyShiftBrite* sb;

void setup(){
  
    Serial1.begin(115200);
    Serial2.begin(115200);
    Serial.begin(115200);
    Serial.flush();
    Serial1.flush();
    Serial2.flush();
    set_speed(zero, zero);
  
//    pinMode(37, OUTPUT);
//    pinMode(39, OUTPUT);
//    digitalWrite(37, HIGH);
//    digitalWrite(39, HIGH);
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
    
//    Tft.init();  //init TFT library
//    Tft.drawString("UALR",0,25,4,WHITE);
//    Tft.drawString("Robotics",25,80,3,WHITE);
//    Tft.drawString("^_^",30 ,200,8,WHITE);

    Serial.println("Setup complete");
}

void loop() {
    if (Serial.available()){
        command_stat = read_serial();
        //announce_sensors();
        if (command_stat){
            exec_command();
        }
    }
}

float ping_median(NewPing* sensor, float avg, int i, int n)
{
  
    float pini = sensor->ping();
    if(pini == 0){
        Serial.print("F1: ");
        Serial.print(n);
        Serial.print(" ");
        pini = sensor->ping();
        if(pini == 0){
            Serial.print("F2: ");
            Serial.print(n);
            Serial.print(" ");
            pini = sensor->ping();
        }
    }
    avg = ((avg*i) + pini)/(i+1);
    return avg;
}

void stablize(){
    unsigned long start = millis();
    
    float avg[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 4; j++)
        {
            //unsigned long t0 = millis();
            avg[j] = ping_median(sensors[j], avg[j], i, j);
            //unsigned long t1 = millis();
            //Serial.print(t1-t0);
            //Serial.print("\t");
            delay(2);
        }
        //Serial.println();
    }

    //Serial.println();

    float rfc = avg[srf] / US_ROUNDTRIP_CM;
    float rrc = avg[srr] / US_ROUNDTRIP_CM;
    float lfc = avg[slf] / US_ROUNDTRIP_CM;
    float lrc = avg[slr] / US_ROUNDTRIP_CM + 0.5; 

    float rangle = atan((rrc-rfc)/8.0)*180/3.14159;
    float langle = -1*atan((lrc-lfc)/8.25)*180/3.14159;
    float avg_angle = (rangle+langle)/2.0;
    
    float correction_speed = 0.0;
    
    Serial.println(avg_angle);
    
    if(abs(avg_angle)>3.0)
    {
      correction_speed = (avg_angle/30.0)*(cruise-zero)*0.5;
      
      l_vel = (cruise-zero)-correction_speed;
      r_vel = (cruise-zero)+correction_speed;
      
//    if(abs(avg_angle)>15.0)
//    {
//        if(avg_angle > 0.0) //need to rotate left
//        {
//            Serial.println("Correcting to go left");
//            l_acc -= 5.0;
//            r_acc += 5.0;
//            //target_left = cruise-max_speed*0.05;
//            //target_right = cruise+max_speed*0.05;
//        }
//        else if(avg_angle < 0.0) //need to rotate right
//        {
//            Serial.println("Correcting to go right");
//            l_acc += 5.0;
//            r_acc -= 5.0;
//            //target_left = cruise+max_speed*0.05;
//            //target_right = cruise-max_speed*0.05;
//        }
//    }

//        if(avg_angle > 0.0)
//        {
//            Serial.println("Turning left");
//            l_acc -= avg_angle/15.0;
//            r_acc += avg_angle/15.0;
//        }  
//        else {
//            Serial.println("Turning right");
//            l_acc -= avg_angle/15.0;
//            r_acc += avg_angle/15.0;
//        }
        //target_left = cruise-max_speed*(avg_angle/180.0);
        //target_right = cruise+max_speed*(avg_angle/180.0);
    }
    else
    {
      l_acc = 0.0;
      r_acc = 0.0;      
      l_vel = max_speed / 3.0;
      r_vel = max_speed / 3.0;      
    }

//    if(r_vel > cruise-zero) r_acc -= 5.0;
//    if(l_vel > cruise-zero) l_acc -= 5.0;
//    if(r_vel < cruise-zero) r_acc += 5.0;
//    if(l_vel < cruise-zero) l_acc += 5.0;
//
//    if(l_acc > a_max) l_acc = a_max;
//    if(l_acc < -a_max) l_acc = -a_max;
//    if(r_acc > a_max) r_acc = a_max;
//    if(r_acc < -a_max) r_acc = -a_max;
    
//    Serial.print("right: \t");    
//    Serial.print(rangle);
//    Serial.print("\t left: \t");
//    Serial.print(langle);
//    Serial.print("\t average: \t");
//    Serial.println((rangle+langle)/2.0);
//    Serial.print("left speed: \t");
//    Serial.print(l_vel);
//    Serial.print(" \tleft acc: \t");
//    Serial.println(l_acc);
//    Serial.print("right speed: \t");
//    Serial.print(r_vel);
//    Serial.print(" \tright acc: \t");
//    Serial.println(r_acc);
//    Serial.println();

//    if( abs(tolf)>0.1 && abs(tolr)>0.1 && abs(torf)>0.1 && abs(torr)>0.1)
//    {
//        float aLeft = tolr/tolf;
//        float aRight = torr/torf;
//        float pLeft = (torf+torr)/(tolf+tolr);
//        float pRight = (tolf+tolr)/(torf+torr);
//        float dLeft = aLeft*aLeft*pLeft*pLeft;
//        float dRight = aRight*aRight*pRight*pRight;
//
//        dLeft = 1-(dLeft+(1-dLeft)*0.95);
//        dRight = 1-(dRight+(1-dRight)*0.95);
//
//        dLeft = dLeft * -1;
//        dRight = dRight * -1;
//
//        Serial.print("dLeft: ");
//        Serial.println(dLeft);
//        Serial.print("dRight: ");
//        Serial.println(dRight);
//
//        target_left = 2048;
//        target_right = 2048;
//
//        //target_left = cruise+800*dLeft;
//        //target_right = cruise+800*dRight;
//
//    }

    l_vel = 0.0;
    r_vel = 0.0;

}

void rotate(){
    stablize();
}


void exec_command(){
    set_speed(cruise, cruise);
    while(true){
        time = millis();
        stablize();
        set_speed(target_left, target_right);
    }
}

void announce_sensors(){
//    Serial.print("lf ");
//    Serial.print(ping_median(sonlf));
//    Serial.print(" lr ");
//    Serial.print(ping_median(sonlr));
//    Serial.print(" rf ");
//    Serial.print(ping_median(sonrf));
//    Serial.print(" rr ");
//    Serial.print(ping_median(sonrr));
//    Serial.print(" ff ");
//    Serial.print(ping_median(sonff));
//    Serial.print("\n");
}

int get_int(int chars){
    char encoded = Serial.read();
    return encoded - 48;
}

void set_speed(int left, int right) {
    old_time = millis();
    elapsed = old_time - time;
    r_vel = r_vel + r_acc*elapsed/1000.0;
    l_vel = l_vel + l_acc*elapsed/1000.0;
    right = bound(r_vel+zero, zero, dead_zone, max_speed);
    left = bound(l_vel+zero, zero, dead_zone, max_speed);
//    Serial.print(left);
//    Serial.print(" ");
//    Serial.print(right);
//    Serial.print("\n");
    target_right = right;
    target_left = left;
    Serial1.write(0xAA); //tells the controller we're starting to send it commands
    Serial2.write(0xAA); //tells the controller we're starting to send it commands
    Serial1.write(0xB);   //This is the pololu device # you're connected too that is found in the config utility(converted to hex). I'm using #11 in this example
    Serial2.write(0xB);   //This is the pololu device # you're connected too that is found in the config utility(converted to hex). I'm using #11 in this example
    Serial1.write(0x40 + (target_right & 0x1F)); //first half of the target, see the pololu jrk manual for more specifics
    Serial2.write(0x40 + (target_left & 0x1F)); //first half of the target, see the pololu jrk manual for more specifics
    Serial1.write((target_right >> 5) & 0x7F);   //second half of the target, " " "
    Serial2.write((target_left >> 5) & 0x7F);   //second half of the target, " " "
 }

int bound(int num, int zero, int dead, int top)
{
    if(num == zero) return zero;

    int difference = abs(num - zero);
    int factor = 0;
    if(num > zero) factor = 1;
    else if(num < zero) factor = -1;

    if(difference < dead)
    {
        return zero+(dead*factor);
    }

    if(difference > top)
    {
        return zero+(top*factor);
    }

    return num;

}

boolean read_serial(){
    if (Serial.available()){
        readbyte = Serial.read();
        Serial.println(readbyte);

        if (readbyte == 63){
          announce_sensors();
            return false;
        }
        if (readbyte == 33){
          move_blocks = 0;
          dir = 0;
            if (Serial.available()){
                move_blocks = get_int();
            }
            if (Serial.available()){
                dir = get_int();
            }
            return true;
        }
        if (readbyte == 35){
          set_speed(2150, 2150);
          return false;
        }
    }
    else{
        return false;
    }
}
