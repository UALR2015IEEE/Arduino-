#include <NewPing.h>
#include <millis>;
#include <Serial>;


unsigned long time = millis();
byte readbyte;
byte writebyte;
boolean command_stat;
byte dir;
byte move_blocks;
const int unit = 12;

word targetx = 0;  //only pass this ints, i tried doing math in this and the remainder error screwed something up
word targety = 0;

NewPing sonlf(23, 22, 50);
NewPing sonrf(25, 24, 50);
NewPing sonlr(27, 26, 50);
NewPing sonrr(31, 30, 50);
NewPing sonff(29, 28, 50);

void setup(){
    Serial1.begin(9600);
    Serial2.begin(9600);
    Serial.begin(115200);
    Serial.flush();
    Serial1.flush();
    Serial2.flush();
    
    SetSpeed(2048, 2048);
}

void loop() {
  if (Serial.available()){
    command_stat = readserial();
    //annoucesence();
    if (command_stat){
      exec_command();
     }
  }
}
float pingmedian(NewPing &senc){
  float newname = 12;
  int pini;
  for(int i = 0; i < 3; i++){
    pini = senc.ping();
    delay(15);
    if(pini == 0){
      pini = senc.ping();
      if(pini == 0){
        pini = senc.ping();
      }
    }
    newname = ((newname*i)+ pini)/(i+1);
  }
  return newname; 
}
    

void stableize(){
  boolean stable = false;
  float tolf;
  float tolr;
  float torf;
  float torr;
  float deltal;
  float deltar;
  float delta;
  float deltafactor;
  int initialx = targetx;
  int initialy = targety;
  while(stable == false){
    tolf = pingmedian(sonlf);
    torr = pingmedian(sonrr);
    tolr = pingmedian(sonlr);
    torf = pingmedian(sonrf);
    deltal = tolr - tolf;
    deltar = torf - torr;
    delta = (deltal + deltar) / 10;
    deltafactor = (delta*delta)/5;
//      Serial.print(tolf);
//      Serial.print(" ");
//      Serial.print(tolr);
//      Serial.print(" ");
//      Serial.print(torf);
//      Serial.print(" ");
//      Serial.print(torr);
//      Serial.print(" ");
//      Serial.print(deltafactor);
//      Serial.print("\n");
    if(delta == 0 || deltafactor < 3){
      stable = true;
    }
    if(delta > 0){
      SetSpeed(initialx - deltafactor, initialy + deltafactor);
    }
    if(delta < 0){
      SetSpeed(initialx + deltafactor, initialy - deltafactor);
    }
  }
  SetSpeed(initialx, initialy);

}


void rotate(){
  stableize();
}

void exec_command(){
  rotate();
}

void annoucesence(){
  Serial.print("lf ");
  Serial.print(sonlf.ping_median(5));
  Serial.print(" lr ");
  Serial.print(sonlr.ping_median(5));
  Serial.print(" rf ");
  Serial.print(sonrf.ping_median(5));
  Serial.print(" rr ");
  Serial.print(sonrr.ping_median(5));
  Serial.print(" ff ");
  Serial.print(sonff.ping_median(5));

  Serial.print("\n");
}

int getint(int chars = 1){
      char encoded = Serial.read();
      int decoded = encoded - 48;
      
      return decoded;
}

void SetSpeed(int x, int y) {
  if (x > 2198){
    x = 2198;
  }
   if (x < 1898){
     x = 1898;
   }
   if (x < 2120 && x > 2048){
     x = 2120;
   }
   if (x > 1976 && x < 2048)
   {
     x = 1976;
   }
   if (y > 2198){
    y = 2198;
  }
   if (y < 1898){
     y = 1898;
   }
   if (y < 2120 && y > 2048){
     y = 2120;
   }
   if (y > 1976 && y < 2048)
   {
     y = 1976;
   }
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print("\n");
  targetx = x;
  targety = y;
  Serial1.write(0xAA); //tells the controller we're starting to send it commands
  Serial2.write(0xAA); //tells the controller we're starting to send it commands
  Serial1.write(0xB);   //This is the pololu device # you're connected too that is found in the config utility(converted to hex). I'm using #11 in this example
  Serial2.write(0xB);   //This is the pololu device # you're connected too that is found in the config utility(converted to hex). I'm using #11 in this example
  Serial1.write(0x40 + (targetx & 0x1F)); //first half of the target, see the pololu jrk manual for more specifics
  Serial2.write(0x40 + (targety & 0x1F)); //first half of the target, see the pololu jrk manual for more specifics
  Serial1.write((targetx >> 5) & 0x7F);   //second half of the target, " " " 
  Serial2.write((targety >> 5) & 0x7F);   //second half of the target, " " " 
 }

boolean readserial(){
    if (Serial.available()){
        readbyte = Serial.read();
        Serial.println(readbyte);

        if (readbyte == 63){
          annoucesence();
            return false;
        }
        if (readbyte == 33){
          move_blocks = 0;
          dir = 0;
            if (Serial.available()){
                move_blocks = getint();
            }
            if (Serial.available()){
                dir = getint();
            }
            return true;
        }
        if (readbyte == 35){
          SetSpeed(2150, 2150);
          return false;
        }
    }
    else{
        return false;
    }
}



