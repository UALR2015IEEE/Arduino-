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
#define D_TO_R PI/180.0
#define R_TO_D 180.0/PI
#define DEBUG false

struct Point {
	float x;
	float y;
	float r;
	Point(float x, float y, float r) : x(x), y(y), r(r) {}
	float Length(){ return sqrt(x*x+y*y); }
};

struct Curve {
	float da;
	float r;
	float v;
	Curve(float r, float da, float v) : da(da), r(r), v(v) {}
};

//function declarations
float ping_median(NewPing* sensor, float avg, int i, int j);
void stablize();
void rotate(int i);
void exec_command();
void announce_sensors();
int get_int(int chars = 1);
void set_speed(int left, int right);
boolean read_serial();
int bound(int num, int zero, int dead, int top);
void p_curve(float t0);
void draw();
Curve set_speed_angle(float vel, float ang);
float cm_to_speed(float v);
float distance(float x, float y);
float distance(Point p1, Point p2);
Point position(float t0);
Point velocity(float t0);
Point acceleration(float t0);
float get_curve_parameter(float s);
float get_curve_length();
float get_x_length(float a0, float da, float tf, float r);
float get_y_length(float a0, float da, float tf, float r);
void padding( int number, byte width );
void padding( float number, byte width );
void padding( double number, byte width );
void run();

//variable declarations
int mode = 1;
int total_loops = 200;
int loops = 0;
int opx;
int opy;
int ovx;
int ovy;
int d_scale = 5;
float old_a;

Point tracking = Point(0.0, 0.0, 0.0);
Point old_tracking = Point(0.0, 0.0, 0.0);
Point current = Point(0.0, 0.0, 0.0);
Point old_current = Point(0.0, 0.0, 0.0);

Point p = Point(0.0, 0.0, 0.0);
Point p0 = Point(0.0, 0.0, 0.0);
Point p1 = Point(0.0, 0.0, 0.0);
Point v = Point(0.0, 0.0, 0.0);
Point a = Point(0.0, 0.0, 0.0);
Curve ss = Curve(0.0, 0.0, 0.0);

float l = 0.0;
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

	p0 = Point(0.0, 0.0, 0.0);
	p1 = Point(121.5, 31.5, 0.0);
	tracking = Point(0.0, 0.0, 0.0);
	current = p0;

    opx = p0.x*d_scale;
    opy = p0.y*d_scale;

    Tft.drawVerticalLine(220,0,320,WHITE);
    Tft.drawHorizontalLine(0,300,240,WHITE);
    for(int x = 0; x < 240; x += d_scale)
    {
        Tft.setPixel(x, 299, WHITE);
    }
    for(int y = 0; y < 320; y += d_scale)
    {
        Tft.setPixel(219, y, WHITE);
    }
    
    Tft.drawCircle(220-(p0.y*d_scale), 300-(p0.x*d_scale), 5, RED);
    Tft.drawCircle(220-(p1.y*d_scale), 300-(p1.x*d_scale), 5, BLUE);
    Tft.drawLine(220-(p0.y*d_scale), 300-(p0.x*d_scale), 220-((p0.y+2*sin(p0.r))*d_scale), 300-((p0.x+2*cos(p0.r))*d_scale), RED);
    Tft.drawLine(220-(p1.y*d_scale), 300-(p1.x*d_scale), 220-((p1.y+2*sin(p1.r))*d_scale), 300-((p1.x+2*cos(p1.r))*d_scale), BLUE);    

    if (DEBUG) Serial.println("Setup complete");

    while(ts.pressure() < ts.pressureThreshhold);

	t = t_min;
	/*while( t < t_max )
	{
		unsigned long t0 = millis();
		p_curve(t);
		t += 0.01;
		draw();
		Serial.println( millis() - t0 );
	}

	t = t_min;*/

	//l = distance(p0, p1);
    p_curve(t_min);
	ovx = v.x*d_scale;
	ovy = v.y*d_scale;
	//l = get_curve_length();
	old_a = v.r;
    ss = set_speed_angle(v.Length(), v.r);
    old_time = millis();   
	if (DEBUG) Serial.println();
	old_tracking = tracking;
	old_current = current;
}

void loop() {
//    t += 0.01;
//    if(t >= 1) t = 0.0;
//    p_curve();
//    draw();
//    set_speed_angle(15, 3.1415926/8.0);
    run();
    //set_speed_angle(80.0, 2.0);
    if (Serial.available()){
        command_stat = read_serial();
//        announce_sensors();
//        
//        if (command_stat){
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
        //}
    }
}

void run()
{
  
    //10s run time
    time = millis();
    
    if(t < 1.0)
    {
        float elapsed = (time - old_time)/1000.0;
        if (DEBUG) Serial.print("t ");
        if (DEBUG) padding(elapsed, 1);
	    /*tracking.x += ss.x*elapsed;
		tracking.y += ss.y*elapsed;
		tracking.r += atan2((tracking.y - old_tracking.y),(tracking.x - old_tracking.x + 0.0000001))*elapsed;
		old_tracking.x = tracking.x;
		old_tracking.y = tracking.y;
		old_tracking.r = tracking.r;*/
		float theta;
		float x_d;
		float y_d;
		/*if(abs(ss.da) < 0.1)
		{
			x_d = ss.v * cos(current.r) * elapsed;
			y_d = ss.v * sin(current.r) * elapsed;

		}
		else
		{*/
		if(ss.da > 0) theta = current.r - PI/2.0;
		else if(ss.da < 0) theta = current.r + PI/2.0;
		x_d = get_x_length(theta, ss.da, elapsed, ss.r);
		y_d = get_y_length(theta, ss.da, elapsed, ss.r);
		//}
		tracking.x += x_d;
		tracking.y += y_d;
		tracking.r = atan2( (tracking.y-old_tracking.y), (tracking.x-old_tracking.x) );
		old_tracking = tracking;
		/*if(abs(ss.da < 0.1))
		{*/
		current.x += ss.r*(cos(theta+ss.da*elapsed) - cos(theta));
		current.y += ss.r*(sin(theta+ss.da*elapsed) - sin(theta));
		/*}
		else
		{
			current.x += ss.v * cos(current.r) * elapsed;
			current.y += ss.v * sin(current.r) * elapsed;
		}*/
		if (DEBUG) Serial.print(" |TH ");
		if (DEBUG) padding(theta*R_TO_D, 4);
		if (DEBUG) Serial.print(" |CTH ");
	    if (DEBUG) padding((theta+ss.da*elapsed)*R_TO_D, 4);
		//current.x += ss.v*cos(current.r+ss.da*elapsed)*elapsed;
		//current.y += ss.v*sin(current.r+ss.da*elapsed)*elapsed;
		current.r = atan2( (current.y - old_current.y), (current.x - old_current.x) );
		old_current = current;
		t = get_curve_parameter(tracking.Length());
		if (DEBUG) Serial.print(" |S ");
		if (DEBUG) padding(t, 1);
        p_curve(t);
        if (DEBUG) Serial.print(" |PX ");
        if (DEBUG) padding(p.x, 4);
        if (DEBUG) Serial.print(" |PY ");
        if (DEBUG) padding(p.y, 4);       
		if (DEBUG) Serial.print(" |PR ");
		if (DEBUG) padding(p.r*R_TO_D, 4);
        if (DEBUG) Serial.print(" |VX ");
        if (DEBUG) padding(v.x, 4);
        if (DEBUG) Serial.print(" |VY ");
        if (DEBUG) padding(v.y, 4);       
		if (DEBUG) Serial.print(" |VR ");
		if (DEBUG) padding(v.r*R_TO_D, 4);
		//tracking = Point( (tracking.x+x_d), (tracking.y+y_d), (tracking.r+ss.da*elapsed) );
        if (DEBUG) Serial.print(" |CX ");
        if (DEBUG) padding(current.x, 3);
        if (DEBUG) Serial.print(" |CY ");
        if (DEBUG) padding(current.y, 3);
		if (DEBUG) Serial.print(" |CR ");
		if (DEBUG) padding(current.r*R_TO_D, 4);
		if (DEBUG) Serial.print(" |X: ");
		if (DEBUG) Serial.print(x_d);
		if (DEBUG) Serial.print(" |Y: ");
	    if (DEBUG) Serial.print(y_d);	
        ss = set_speed_angle(v.Length(), v.r);
        old_time = time;
        draw();
		if (DEBUG) Serial.println();
    }
    
    if(t >= 1.0) set_speed(zero, zero);
        
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

void rotate(int r)
{
	//d = 17.5 => r = 8.75
	//r > 0 -> rotate ccw
	if(r > 0)
	{
		for(int i = 0; i < r; i++)
		{
			set_speed((-1*cm_to_speed(27.4889))+zero, cm_to_speed(27.4889)+zero);
			float t0 = millis();
			while(millis() - t0 < 500) {}
			set_speed(zero, zero);
		}
	}
	if(r < 0)
	{
		for(int i = 0; i < r; i++)
		{
			set_speed(cm_to_speed(27.4889)+zero, (-1*cm_to_speed(27.4889))+zero);
			float t0 = millis();
			while(millis() - t0 < 500) {}
			set_speed(zero, zero);
		}
	}
}

Curve set_speed_angle(float vel, float ang)
{
	/*if( abs (ang-old_a) > 0.001) 
	{
		Serial.print(" O/W ");
		ang = old_a * -1;
	}
	ang = ang / 2.0;*/
    float r = abs(vel/(ang+0.00000001)); 
    float r_inner = r - 8.75;
    float r_outer = r + 8.75;
    float v_inner = r_inner * abs(ang);
    float v_outer = r_outer * abs(ang);
		
	while(cm_to_speed(v_inner)+zero > 2700 || cm_to_speed(v_outer)+zero > 2700)
	{
		vel = vel * 0.99;
		ang = ang * 0.99;
		v_inner = v_inner * 0.99;
		v_outer = v_outer * 0.99;
	}

	while(cm_to_speed(v_inner)+zero < 2400 || cm_to_speed(v_outer)+zero < 2400)
	{
		vel = vel * 1.01;
		ang = ang * 1.01;
		v_inner = v_inner * 1.01;
		v_outer = v_outer * 1.01;
	}

    
    if (DEBUG) Serial.print(" |v ");
    if (DEBUG) padding(vel, 3);
    if (DEBUG) Serial.print(" |a ");
    if (DEBUG) padding(ang*R_TO_D, 4);
    //Serial.print("\tr: ");
    //Serial.print(r);
    //Serial.print("\tr_inner ");
    //Serial.print(r_inner);
    //Serial.print("\tr_outer ");
    //Serial.print(r_outer);
    if (DEBUG) Serial.print(" |v_i ");
    if (DEBUG) padding(v_inner, 3);
    if (DEBUG) Serial.print(" |v_o ");
    if (DEBUG) padding(v_outer, 3);
    //Serial.print(" |cm_i ");
    //padding(cm_to_speed(v_inner)+zero, 4);
    //Serial.print(" |cm_o ");
    //padding(cm_to_speed(v_outer)+zero, 4);

    if(ang > 0.0) //rotate ccw, left is r_inner
    {
		if (DEBUG) Serial.print(" |L ");
        set_speed(cm_to_speed(v_inner)+zero, cm_to_speed(v_outer)+zero);
    }
    else
    {
		if (DEBUG) Serial.print(" |R ");
        set_speed(cm_to_speed(v_outer)+zero, cm_to_speed(v_inner)+zero);
    }

	return Curve(r, ang, vel);

}

void draw()
{
    //if(mode == 0)
    {
        int x = p.x*d_scale;
        int y = p.y*d_scale;
		//Serial.print("X: ");
		//Serial.print(x);
		//Serial.print("\tY: ");
		//Serial.println(y);
		if( y > 220 ) y = y%220;
		if( x > 300 ) x = x%300;
		//while(220 - y < 0) { y -= 220; }
		//while(300 - x < 0) { x -= 300; }
        Tft.drawLine(220-(y), 300-(x), 220-(opy), 300-(opx), GREEN);
        opx = x;
        opy = y;        
//        int x = px*20;
//        int y = py*20;
//        int x1 = 
//        Tft.setPixel(x+20, 160-y, GREEN);
    }
    //if(mode == 1)
    {
        int x = v.x*d_scale;
        int y = v.y*d_scale;
		//Serial.print("X: ");
		//Serial.print(x);
		//Serial.print("\tY: ");
		//Serial.println(y);
		if( y > 220 ) y = y%220;
		if( x > 300 ) x = x%300;
		while(y < 0) { y += 220; }
		while(x < 0) { x += 300; }
        Tft.drawLine((y), (x), (ovy), (ovx), GREEN);
        ovx = x;
        ovy = y;        
//        int x = px*20;
//        int y = py*20;
//        int x1 = 
//        Tft.setPixel(x+20, 160-y, GREEN);
	}
}

float get_curve_length()
{
	float t0 = t_min;
	int n = 200;
	float h = t_max/n;

	float l = 0.0;

	for(int i = 0; i <= n; i++)
	{		
		l += velocity(t0).Length();
		t0 += h;
	}
	return l;

}

float get_x_length(float a0, float da, float tf, float r)
{
	
	if(tf < 0.00001) return 0;

	float t0 = a0;
	int n = 20;
	float h = tf/n;

	a0 += da*h;

	float l = 0;
	for(int i = 1; i <= n; i++)
	{
		l += abs(r*(cos(a0)-cos(t0)));
		t0 = a0;
		a0 += da*h;
	}

	return l;

}

float get_y_length(float a0, float da, float tf, float r)
{

	if(tf < 0.00001) return 0;

	float t0 = a0;
	int n = 20;
	float h = tf/n;
	
	a0 += da*h;

	float l = 0;
	for(int i = 1; i <= n; i++)
	{
		l += abs(r*(sin(a0)-sin(t0)));
		t0 = a0;
		a0 += da*h;
	}

	return l;

}

float get_curve_parameter(float s)
{
	float t0 = t_min;
	int n = 20;
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
	Point v = velocity(t0);
	float r = atan2(v.y, v.x);
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
	Point a = acceleration(t0);
    float r = atan2(a.y, a.x);
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
	float r = atan2(y, x);
	return Point(x, y, r);
}

void p_curve(float t0)
{
	p = position(t0);
	v = velocity(t0);
	a = acceleration(t0);
}


void padding( double number, byte width ) {
	byte i = 1;
	if( number < 0 ) 
	{
		if (DEBUG) Serial.print("-");
		i = 2;
		number = -number;
	}

	int currentMax = 10;

	for (i; i<width; i++){
		if (number < currentMax) {
			if (DEBUG) Serial.print("0");
		}
		currentMax *= 10;
	} 
	if (DEBUG) Serial.print(number);
}

void padding( float number, byte width ) {
	byte i = 1;
	if( number < 0 ) 
	{
		if (DEBUG) Serial.print("-");
		i = 2;
		number = -number;
	}

	int currentMax = 10;

	for (i; i<width; i++){
		if (number < currentMax) {
			if (DEBUG) Serial.print("0");
		}
		currentMax *= 10;
	} 
	if (DEBUG) Serial.print(number);
}

void padding( int number, byte width ) {
	byte i = 1;
	if( number < 0 ) 
	{
		if (DEBUG) Serial.print("-");
		i = 2;
		number = -number;
	}

	int currentMax = 10;

	for (i; i<width; i++){
		if (number < currentMax) {
			if (DEBUG) Serial.print("0");
		}
		currentMax *= 10;
	} 
	if (DEBUG) Serial.print(number);
}

float ping_median(NewPing* sensor, float avg, int i, int n)
{
  
    float pini = sensor->ping();
    if(pini == 0){
        if (DEBUG) Serial.print("F1: ");
        if (DEBUG) Serial.print(n);
        if (DEBUG) Serial.print(" ");
        pini = sensor->ping();
        if(pini == 0){
            if (DEBUG) Serial.print("F2: ");
            if (DEBUG) Serial.print(n);
            if (DEBUG) Serial.print(" ");
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
        
    if (DEBUG) Serial.print("l_dist:\t");
    if (DEBUG) Serial.print(l_dist);
    if (DEBUG) Serial.print("\tr_dist:\t");
    if (DEBUG) Serial.print(r_dist);
    if (DEBUG) Serial.print("\tdifference:\t");
    if (DEBUG) Serial.println(abs(r_dist-l_dist));
    
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
        if (DEBUG) Serial.print((abs(angle))*a_max*abs((l_vel+1)/max_speed)); 
        if (DEBUG) Serial.println();
        
        l_acc -= (abs(angle))*a_max*abs((l_vel+1)/max_speed);
        r_acc += (abs(angle))*a_max*abs((r_vel+1)/max_speed);        
    } else if( (r_dist - l_dist) > 2.0 && angle < 2.0) //need to move more to the right
    {
        //Serial.println("Angle's fine, moving toward the right");      
        //Serial.print("\nacc delta: ");
        if (DEBUG) Serial.print((abs(angle))*a_max*abs((l_vel+1)/max_speed));   
        if (DEBUG) Serial.println();
        
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
    
    if (DEBUG) Serial.print("\nright: \t");    
    if (DEBUG) Serial.print(r_angle);
    if (DEBUG) Serial.print("\t left: \t");
    if (DEBUG) Serial.print(l_angle);
    if (DEBUG) Serial.print("\t average: \t");
    if (DEBUG) Serial.println(angle);
    if (DEBUG) Serial.print("left speed: \t");
    if (DEBUG) Serial.print(l_vel);
    if (DEBUG) Serial.print(" \tleft acc: \t");
    if (DEBUG) Serial.println(l_acc);
    if (DEBUG) Serial.print("right speed: \t");
    if (DEBUG) Serial.print(r_vel);
    if (DEBUG) Serial.print(" \tright acc: \t");
    if (DEBUG) Serial.println(r_acc);
    if (DEBUG) Serial.println();

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
    while(!Serial.available());
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
    if(right > 4095) right = 4095;
    if(left > 4095) left = 4095;
    target_right = right;
    target_left = left;
    if(left != 2048 && right != 2048)
    {
      //Serial.print("Left: ");
      //Serial.print(left);
      //Serial.print("\tRight: ");
      //Serial.println(right);
    }
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
        if (readbyte == 33) {
            move_blocks = 0;
            dir = 0;
            if (Serial.available()) {
                move_blocks = get_int();
            }
            return true;
        }
        if (readbyte == 64){
            if (Serial.available()){
                dir = get_int();
            }
            return true;
        }
        if (readbyte == 35){
            Serial.print(ts.pressure() < ts.pressureThreshhold);
          return false;
        }
        if (readbyte == 94){
            int com = get_int();
	    Serial.println(com);
            Tft.drawChar(char(com), 150, 100, 5, WHITE);
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
