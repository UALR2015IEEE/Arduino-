/* -*- C++ -*- */

//Includes
#include <NewPing.h>
#include <millis>;
#include <Serial>;

//function declarations
float ping_median(NewPing* sensor);
void stablize(unsigned long time_factor = 0);
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

    set_speed(2048, 2048);
    stablize(1000);
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

void stablize(unsigned long time_factor){
    boolean stable = false;
    float tolf;
    float tolr;
    float torf;
    float torr;
    float deltal;
    float deltar;
    float delta;
    float deltafactor;
    int initial_right = target_right;
    int initial_left = target_left;
    unsigned long start_time = millis();
    Serial.print("stable: ");
    Serial.println(!stable);
    while(stable == false){
//        count += 1;
//        Serial.print("Count: ");
//        Serial.println(count);
//        Serial.print("millis: " );
//        Serial.println(millis());
//        Serial.print("time diff ");
//        Serial.println(millis() - start_time);
//        Serial.print("stable: ");
//        Serial.println(!stable);
        if (time_factor != 0 && millis() - start_time > time_factor){
            break;
        }
        tolf = ping_median(sonlf);
        torr = ping_median(sonrr);
        tolr = ping_median(sonlr);
        torf = ping_median(sonrf);

        if( abs(tolf)<0.1 || abs(tolr)<0.1 || abs(torf)<0.1 || abs(torr)<0.1)
        {
          set_speed(initial_left, initial_right);
        }
        else {
          float tLeft = tolr/tolf;
          float tRight = torr/torf;
          float dLeft = tLeft*tLeft*tLeft*(torf+torr)/(tolf+tolr);
          float dRight = tRight*tRight*tRight*(tolf+tolr)/(torf+torr);
          
          dLeft = 1-(dLeft+(1-dLeft)*0.95);
          dRight = 1-(dRight+(1-dRight)*0.95);  
          
          dLeft = dLeft * -1;
          dRight = dRight * -1;
          
          Serial.print("dLeft: ");
          Serial.println(dLeft);
          Serial.print("dRight: ");
          Serial.println(dRight);
          
          if(abs(dLeft - dRight) < 0.1)
          {
              stable = true;
              set_speed(initial_left, initial_right);
          }
          else set_speed(initial_left+800*dLeft, initial_right+800*dRight);
        }
        target_left = initial_left;
        target_right = initial_right;
    }

}

void rotate(){
    stablize();
}

void exec_command(){
    set_speed(2400, 2400);
    while(true){
        stablize(150);
        Serial.println("Exit Loop");
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
    right = bound(right, 2048, 0, 800);
    left = bound(left, 2048, 0, 800);
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
