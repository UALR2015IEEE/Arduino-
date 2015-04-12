/* -*- C++ -*- */

//Includes
#include <NewPing.h>
#include <millis>;
#include <Serial>;

//function declarations
float ping_median(NewPing* sensor);
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
byte readbyte;
byte writebyte;
boolean command_stat;
byte dir;
byte move_blocks;
const int unit = 12;

unsigned int cruise = 2200;
unsigned int dead_zone = 0;
unsigned int max_speed = 800;
unsigned int zero = 2048;

word target_right = 0;  //only pass this ints, i tried doing math in this and the remainder error screwed something up
word target_left = 0;

NewPing* sonlf;
NewPing* sonrf;
NewPing* sonlr;
NewPing* sonrr;
NewPing* sonff;

void setup(){

    sonlf = new NewPing(23, 22, 50);
    sonrf = new NewPing(25, 24, 50);
    sonlr = new NewPing(27, 26, 50);
    sonrr = new NewPing(31, 30, 50);
    sonff = new NewPing(29, 28, 50);

    Serial1.begin(115200);
    Serial2.begin(115200);
    Serial.begin(115200);
    Serial.flush();
    Serial1.flush();
    Serial2.flush();

    set_speed(zero, zero);
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

float ping_median(NewPing* sensor){
    float avg = 12;
    int pini;
    for(int i = 0; i < 3; i++){
        pini = sensor->ping();
        delay(15);
        if(pini == 0){
            pini = sensor->ping();
            if(pini == 0){
                pini = sensor->ping();
            }
        }
        avg = ((avg*i) + pini)/(i+1);
    }
    return avg;
}

void stablize(){
    float tolf = ping_median(sonlf);
    float tolr = ping_median(sonlr);
    float torf = ping_median(sonrf);
    float torr = ping_median(sonrr);

    if( abs(tolf)>0.1 && abs(tolr)>0.1 && abs(torf)>0.1 && abs(torr)>0.1)
    {
        float aLeft = tolr/tolf;
        float aRight = torr/torf;
        float pLeft = (torf+torr)/(tolf+tolr);
        float pRight = (tolf+tolr)/(torf+torr);
        float dLeft = aLeft*aLeft*pLeft*pLeft;
        float dRight = aRight*aRight*pRight*pRight;
        
        dLeft = 1-(dLeft+(1-dLeft)*0.95);
        dRight = 1-(dRight+(1-dRight)*0.95);  
        
        dLeft = dLeft * -1;
        dRight = dRight * -1;
        
        Serial.print("dLeft: ");
        Serial.println(dLeft);
        Serial.print("dRight: ");
        Serial.println(dRight);
        
        target_left = cruise+800*dLeft;
        target_right = cruise+800*dRight;

    }

}

void rotate(){
    stablize();
}


void exec_command(){
    set_speed(cruise, cruise);
    while(true){
        stablize();
        set_speed(target_left, target_right);
    }
}

void announce_sensors(){
    Serial.print("lf ");
    Serial.print(ping_median(sonlf));
    Serial.print(" lr ");
    Serial.print(ping_median(sonlr));
    Serial.print(" rf ");
    Serial.print(ping_median(sonrf));
    Serial.print(" rr ");
    Serial.print(ping_median(sonrr));
    Serial.print(" ff ");
    Serial.print(ping_median(sonff));
    Serial.print("\n");
}

int get_int(int chars){
    char encoded = Serial.read();
    return encoded - 48;
}

void set_speed(int left, int right) {
    right = bound(right, zero, dead_zone, max_speed);
    left = bound(left, zero, dead_zone, max_speed);
    Serial.print(left);
    Serial.print(" ");
    Serial.print(right);
    Serial.print("\n");
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
