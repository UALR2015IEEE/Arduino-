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

#define slf 0
#define srr 1
#define srf 2
#define slr 3
#define sff 4
#define PI 3.14159
#define D_TO_R 3.14159/180.0
#define R_TO_D 180.0/3.14159

struct Point {
	float x;
	float y;
	float r;
	Point(float x, float y, float r) : x(x), y(y), r(r) {}
	float Length(){ return sqrt(x*x+y*y); }
};

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
void p_curve(float t0);
void draw();
void set_speed_angle(float v, float a);
float cm_to_speed(float v);
float distance(float x, float y);
float distance(Point p1, Point p2);
Point position(float t0);
Point velocity(float t0);
float get_curve_parameter(float s);
void run();

//variable declarations
int mode = 0;
int total_loops = 200;
int loops = 0;
int opx;
int opy;

Point tracking = Point(0.0, 0.0, 0.0);

Point p = Point(0.0, 0.0, 0.0);
Point p0 = Point(0.0, 0.0, 0.0);
Point p1 = Point(0.0, 0.0, 0.0);
Point v = Point(0.0, 0.0, 0.0);
Point a = Point(0.0, 0.0, 0.0);

float t = 0.0;
float t_min = 0.0;
float t_max = 1.0;

unsigned long time = millis();
unsigned long old_time = time;
unsigned long elapsed = 0;

byte readbyte;
byte writebyte;
boolean command_stat;
byte dir;
byte move_blocks;
const int unit = 12;

int cruise = 2200;
int dead_zone = 0;
int zero = 2048;
int max_speed = cruise-zero;

float r_acc = 0.0;
float r_vel = 0.0;
float r_pos = 0.0;

float l_acc = 0.0;
float l_vel = 0.0;
float l_pos = 0.0;

float a_max = max_speed/4;

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
    
    Tft.init();  //init TFT library
//    Tft.drawString("UALR",0,25,4,WHITE);
//    Tft.drawString("Robotics",25,80,3,WHITE);
//    Tft.drawString("^_^",30 ,200,8,WHITE);

	p0 = Point(1.0, 1.0, 45*D_TO_R);
	p1 = Point(35.0, 1.0, 0.0);
	tracking = Point(p0.x, p0.y, p0.r);

    opx = p0.x*20;
    opy = p0.y*20;

    Tft.drawVerticalLine(220,0,320,WHITE);
    Tft.drawHorizontalLine(0,300,240,WHITE);
    for(int x = 0; x < 240; x += 20)
    {
        Tft.setPixel(x, 299, WHITE);
    }
    for(int y = 0; y < 320; y += 20)
    {
        Tft.setPixel(219, y, WHITE);
    }
    
    Tft.drawCircle(220-(p0.y*20), 300-(p0.x*20), 5, RED);
    Tft.drawCircle(220-(p1.y*20), 300-(p1.x*20), 5, BLUE);
    Tft.drawLine(220-(p0.y*20), 300-(p0.x*20), 220-((p0.y+2*sin(p0.r))*20), 300-((p0.x+2*cos(p0.r))*20), RED);
    Tft.drawLine(220-(p1.y*20), 300-(p1.x*20), 220-((p1.y+2*sin(p1.r))*20), 300-((p1.x+2*cos(p1.r))*20), BLUE);    

    Serial.println("Setup complete");
    while(ts.pressure() < ts.pressureThreshhold);
    
    p_curve(t_min);
    set_speed_angle(v.Length(), p.r);
    old_time = millis();    
    
}

void loop() {
//    t += 0.01;
//    if(t >= 1) t = 0.0;
//    p_curve();
//    draw();
//    set_speed_angle(15, 3.1415926/8.0);
    run();
    if (Serial.available()){
        command_stat = read_serial();
//        announce_sensors();
//        
        if (command_stat){
//            p0x += 1;
//            Tft.paintScreenBlack();
//            Tft.drawVerticalLine(20,0,320,WHITE);
//            Tft.drawHorizontalLine(0,160,240,WHITE);
//            for(int x = 0; x < 240; x += 20)
//            {
//                Tft.setPixel(x, 159, WHITE);
//            }
//            for(int y = 0; y < 320; y += 20)
//            {
//                Tft.setPixel(21, y, WHITE);
//            }
//            Tft.drawCircle(20+(p0x*20), 160-(p0y*20), 5, RED);
//            Tft.drawCircle(20+(p1x*20), 160-(p1y*20), 5, BLUE);
//            Tft.drawLine(20+(p0x*20), 160-(p0y*20), 20+((p0x+2*cos(r0))*20), 160-((p0y+2*sin(r0))*20), RED);
//            Tft.drawLine(20+(p1x*20), 160-(p1y*20), 20+((p1x+2*cos(r1))*20), 160-((p1y+2*sin(r1))*20), BLUE);               
//            Tft.drawString("UALR",0,25,4,WHITE);
//            Tft.drawString("Robotics",25,80,3,WHITE);
//            Tft.drawString("^_^",30 ,200,8,WHITE);
//            mode += 1;
//            if(mode > 1) mode = 0;
//            //exec_command();
        }
    }
}

void run()
{
  
    //10s run time
    time = millis();
    
    if(time - old_time > 10 && loops <= total_loops)
    {
        t += 1.0/total_loops;
        p_curve(t);        
        //set_speed_angle(v.Length(), p.r);
        draw();
        loops++;
		unsigned int elapsed = (time - old_time)/1000.0;
		tracking = Point( (tracking.x+v.x*elapsed), (tracking.y+v.y*elapsed), (tracking.r+v.r*elapsed) );
		Point n = Point(v.x*elapsed, v.y*elapsed, 0.0);
		float m = get_curve_parameter(n.Length());
        Serial.print("t: ");
        Serial.println(elapsed);
        Serial.print("TX: ");
        Serial.print(tracking.x);
        Serial.print("\tTY: ");
        Serial.println(tracking.y);      
        Serial.print("PX: ");
        Serial.print(p.x);
        Serial.print("\tPY: ");
        Serial.print(p.y);          
		Serial.print("\tS: ");
		Serial.println(m);
        old_time = time;
    }
    
    if(loops == total_loops)
    {
        set_speed(zero, zero);
        loops++;
    }
    
}

float distance(float x, float y)
{
	return sqrt(x*x+y*y);
}

float distance(Point p1, Point p2)
{
	return sqrt( (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y) );
}

float cm_to_speed(float v)
{
    return (v-10.0735)/0.029695;
}

void set_speed_angle(float v, float a)
{
    float r = abs(v/a); 
    float r_inner = r - 8.75;
    float r_outer = r + 8.75;
    float v_inner = r_inner * abs(a);
    float v_outer = r_outer * abs(a);
    
    Serial.print("v: ");
    Serial.print(v);
    Serial.print("\ta: ");
    Serial.print(a*180/3.14159);
    Serial.print("\tr: ");
    Serial.print(r);
    Serial.print("\tr_inner ");
    Serial.print(r_inner);
    Serial.print("\tr_outer ");
    Serial.print(r_outer);
    Serial.print("\tv_inner ");
    Serial.print(v_inner);
    Serial.print("\tv_outer ");
    Serial.print(v_outer);
    Serial.print("\tcm_inner ");
    Serial.print(cm_to_speed(v_inner)+zero);
    Serial.print("\tcm_outer ");
    Serial.print(cm_to_speed(v_outer)+zero);
    Serial.println();

    if(a > 0.0) //rotate ccw, left is r_inner
    {
        set_speed(cm_to_speed(v_inner)+zero, cm_to_speed(v_outer)+zero);
    }
    else
    {
        set_speed(cm_to_speed(v_outer)+zero, cm_to_speed(v_inner)+zero);
    }
}

void draw()
{
    if(mode == 0)
    {
        int x = p.x*20;
        int y = p.y*20;
        Tft.drawLine(220-(y), 300-(x), 220-(opy), 300-(opx), GREEN);
        opx = x;
        opy = y;        
//        int x = px*20;
//        int y = py*20;
//        int x1 = 
//        Tft.setPixel(x+20, 160-y, GREEN);
    }
    if(mode == 1)
    {
//        Serial.print("t:\t");
//        Serial.print(t);
//        Serial.print("\tdpx:\t");
//        Serial.print(dpx);
//        Serial.print("\tdpy:\t");
//        Serial.println(dpy);
        int x = v.x*20;
        int y = v.y*20;
        Tft.setPixel(x+20, 160-y, GREEN);        
    }
//    Tft.fillRectangle(25, 200, 150, 100, BLACK);
//    int t = v;
//    String s;
//    s = String(t);
//    char b[3];
//    s.toCharArray(b, 3);
//    Tft.drawString(b, 30, 200, 5, WHITE);
}

float get_curve_parameter(float s)
{
	float t0 = t_min;
	int n = 30;
	float h = s/n;

	for(int i = 0; i <= n; i++)
	{
		float k1 = h/velocity(t0).Length();
		float k2 = h/velocity(t0+k1/2.0).Length();
		float k3 = h/velocity(t0+k2/2.0).Length();
		float k4 = h/velocity(t0+k3/2.0).Length();
		t0 += (k1+2.0*(k2+k3)+k4)/6.0;
	}
	return t0;
}

Point position(float t0)
{
	float t2 = t0*t0;
	float t3 = t2*t0;
    float h0 = (2*t3)-(3*t2)+1;
    float h1 = (t3)-(2*t2)+t0;
    float h2 = (-2*t3)+(3*t2);
    float h3 = (t3)-(t2);
    float d = distance(p0, p1);
	float x = h0*p0.x+h1*d*cos(p0.r)+h2*p1.x+h3*d*cos(p1.r);
    float y = h0*p0.y+h1*d*sin(p0.r)+h2*p1.y+h3*d*sin(p1.r);
	float r = atan(y/x);
	return Point(x, y, r);
}

Point velocity(float t0)
{

    float t2 = t0*t0;
    float dh0 = (6*t2)-(6*t0);
    float dh1 = (3*t2)-(4*t0)+1;
    float dh2 = (-6*t2)+(6*t0);
    float dh3 = (3*t2)-(2*t0);
    float d = distance(p0, p1);
    float x = dh0*p0.x+dh1*d*cos(p0.r)+dh2*p1.x+dh3*d*cos(p1.r);
    float y = dh0*p0.y+dh1*d*sin(p0.r)+dh2*p1.y+dh3*d*sin(p1.r);
    float r = atan(y/x);
	return Point(x, y, r);
}

Point acceleration(float t0)
{
    float ddh0 = (12*t0)-6;
    float ddh1 = (6*t0)-4;
    float ddh2 = (-12*t0)+6;
    float ddh3 = (6*t0)-2;    
    float d = distance(p0, p1);
    float x = ddh0*p0.x+ddh1*d*cos(p0.r)+ddh2*p1.x+ddh3*d*cos(p1.r);
	float y = ddh0*p0.y+ddh1*d*sin(p0.r)+ddh2*p1.y+ddh3*d*sin(p1.r);
	float r = atan(y/x);
	return Point(x, y, r);
}

void p_curve(float t0)
{
	p = position(t0);
	v = velocity(t0);
	a = acceleration(t0);
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
    float dist[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    
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

//    unsigned long t1 = millis();
//    Serial.print(t1-start);
//    Serial.print("\t");
//
    for(int i = 0; i < 4; i++)
    {
        dist[i] = avg[i] / US_ROUNDTRIP_CM;
        //Serial.print(dist[i]);
        //Serial.print("\t");
    }
    
    float l_dist = (dist[slf] + dist[slr])/2.0;
    float r_dist = (dist[srf] + dist[srr])/2.0;
    float r_angle = atan((dist[srr]-dist[srf])/8.0)*180/3.14159;
    float l_angle = -1*atan((dist[slr]-dist[slf])/8.25)*180/3.14159;
    
    float angle = 0.0;
    
    if(abs(r_dist - l_dist) > 2)
    {    
        if(r_dist > l_dist) angle = r_angle;
        else angle = l_angle;
    }   
    else angle = (r_angle+l_angle)/2.0;
    
    //float correction_speed = 0.0;
    
    //Serial.println(angle);
    //Serial.println("\n");
    
    //if(abs(angle)>3.0)
    //{
      //correction_speed = (angle/30.0)*(cruise-zero)*0.5;
      
      //l_vel = (cruise-zero)-correction_speed;
      //r_vel = (cruise-zero)+correction_speed;
        
    Serial.print("l_dist:\t");
    Serial.print(l_dist);
    Serial.print("\tr_dist:\t");
    Serial.print(r_dist);
    Serial.print("\tdifference:\t");
    Serial.println(abs(r_dist-l_dist));
    
    if(abs(angle)>3.0)
    {
      
        //if(angle > 0.0) Serial.println("Correction to go left");
        //if(angle < 0.0) Serial.println("Correction to go right");
        
        //Serial.print("acc delta: ");
        //Serial.print((angle/30.0)*a_max*abs((l_vel+1)/max_speed)); 
        //Serial.println();    
        
        l_acc -= (angle/30.0)*a_max*abs((l_vel+1)/max_speed);
        r_acc += (angle/30.0)*a_max*abs((r_vel+1)/max_speed);
      
        //target_left = cruise-max_speed*(angle/180.0);
        //target_right = cruise+max_speed*(angle/180.0);
    }
    else if( (l_dist - r_dist) > 2.0 && angle > -2.0) //need to move more to the left
    {
        //Serial.println("\nAngle's fine, moving toward the left");
        //Serial.print("acc delta: ");
        Serial.print((abs(angle))*a_max*abs((l_vel+1)/max_speed)); 
        Serial.println();
        
        l_acc -= (abs(angle))*a_max*abs((l_vel+1)/max_speed);
        r_acc += (abs(angle))*a_max*abs((r_vel+1)/max_speed);        
    } else if( (r_dist - l_dist) > 2.0 && angle < 2.0) //need to move more to the right
    {
        //Serial.println("Angle's fine, moving toward the right");      
        //Serial.print("\nacc delta: ");
        Serial.print((abs(angle))*a_max*abs((l_vel+1)/max_speed));   
        Serial.println();
        
        l_acc += (abs(angle))*a_max*abs((l_vel+1)/max_speed);
        r_acc -= (abs(angle))*a_max*abs((r_vel+1)/max_speed);        
    }
    else
    {
        if(r_vel > max_speed) r_acc -= 15.0;
        else r_acc += 15.0;
        if(l_vel > max_speed) l_acc -= 15.0;
        else l_acc += 15.0;   
    }

    if(r_vel > max_speed) r_vel = max_speed;
    if(r_vel < 0) r_vel = 0;
    if(l_vel > max_speed) l_vel = max_speed;
    if(l_vel < 0) l_vel = 0;
    
    if(l_acc > a_max) l_acc = a_max;
    if(l_acc < -a_max) l_acc = -a_max;
    if(r_acc > a_max) r_acc = a_max;
    if(r_acc < -a_max) r_acc = -a_max;
    
    Serial.print("\nright: \t");    
    Serial.print(r_angle);
    Serial.print("\t left: \t");
    Serial.print(l_angle);
    Serial.print("\t average: \t");
    Serial.println(angle);
    Serial.print("left speed: \t");
    Serial.print(l_vel);
    Serial.print(" \tleft acc: \t");
    Serial.println(l_acc);
    Serial.print("right speed: \t");
    Serial.print(r_vel);
    Serial.print(" \tright acc: \t");
    Serial.println(r_acc);
    Serial.println();

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
//    old_time = time;
//    time = millis();
//    elapsed = time - old_time;
//    Serial.print("\nelapsed:\t");
//    Serial.println(elapsed);
//    r_vel = r_vel + r_acc*elapsed/1000.0;
//    l_vel = l_vel + l_acc*elapsed/1000.0;
//    right = bound(r_vel+zero, zero, dead_zone, max_speed);
//    left = bound(l_vel+zero, zero, dead_zone, max_speed);
//    Serial.print(left);
//    Serial.print(" ");
//    Serial.print(right);
//    Serial.print("\n");
//    right = bound(right, zero, dead_zone, max_speed);
//    left = bound(left, zero, dead_zone, max_speed);
    target_right = right;
    target_left = left;
    Serial.print("Left:\t");
    Serial.print(left);
    Serial.print("\tRight:\t");
    Serial.println(right);
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
