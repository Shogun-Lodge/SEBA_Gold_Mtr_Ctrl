/*
 * Motor Controller Seba Gold Controller Test
 * 
 * OSC Control for -
 *  > Speed
 *  > Accel
 *  > Direction
 */

#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>

#include <OSCMessage.h>

#include <Encoder.h>

#define SPD_OUT A22               // DAC Output 0-3.3V - Speed Control
#define ACC_OUT A21               // DAC Output 0-3.3V - Accel/Decel Control
#define DIR_PIN 39                // Direction control

// Stop and Limit Pins
#define SOFT 17
#define HARD 16
#define REV_LIM 23
#define FWD_LIM 22

EthernetUDP Udp;

uint8_t unitMac[] = {0xAB, 0xBA, 0xBE, 0xEF, 0xFE, 0xED}; // you can find this written on the board of some Arduino Ethernets or shields

// Unit's IP
IPAddress unitIp(10, 101, 40, 20);

// Interface Controller IP
uint8_t remoteIp[4] = {10, 101, 40, 10};

//port numbers
const unsigned int inPort = 7777;       // Unit's Port        
uint32_t port = 8888;                   // Interface Controller Port

// Current Speed and Accel Settings
volatile uint32_t oldSpd = 0;
uint32_t newSpd = 0;
volatile uint32_t oldAcc = 50;

// Maths
float dacRes = (3.3/4095);
float tenVolt = (10/3.3);
float fiveVolt = (5/3.3);

char dirBuff[4];

// GO flag
volatile uint8_t goFlg = 0;

// Limit flags
uint8_t revLim = 0;
uint8_t fwdLim = 0;

// Timer vals
uint32_t timeFlg = 0;
uint32_t timeVal = 20;

// Interrupt Stuff
volatile bool way;
volatile bool eStopFlg = 0;
volatile bool slowStopFlg = 0;
//volatile uint32_t timeHold;
volatile uint8_t howMany = 0;
volatile bool once = 0;

IntervalTimer spdTimer;

// Encoder things
int16_t newEnc = 0;
int16_t oldEnc = 0;
uint8_t encRes = 128;
Encoder enc(25,24);

//-----Functions-----
// Change Speed level during timed Accel/Decel (IntervalTimer)
void timedSpeed(){  
  if(way){
    analogWrite(SPD_OUT, uint32_t((oldSpd+howMany)*40.95));
    Serial.print("oldSpd+x = ");
    Serial.println(oldSpd+howMany);
    Serial.println(uint32_t((oldSpd+1)*40.95));        
  }
  else{
    analogWrite(SPD_OUT, uint32_t((oldSpd-howMany)*40.95));
    Serial.print("oldSpd-x = ");
    Serial.println(oldSpd-howMany);
    Serial.println(uint32_t((oldSpd-howMany)*40.95));  
  }
  howMany++;
}

// Check stops and limits - Yes this lazy and should be moved to interupts
void stops(){
  // E-Stop?
  //if(!digitalRead(HARD)){
    //eStop();
  //} 
  // Soft stop?
  //if(!digitalRead(SOFT)){
    //slowStop(); 
  //}
  // Forward limit switched
  if(!digitalRead(FWD_LIM) && !fwdLim){
    fwdLimit(); 
  }
  // Forward limit released
  else if(digitalRead(FWD_LIM)){
    fwdLim = 0;
  }  
  // Reverse limtit switched
  if(!digitalRead(REV_LIM) && !revLim){
    revLimit(); 
  }
  // Reverse limit releases
  else if(digitalRead(REV_LIM)){
    revLim = 0;
  }
}

// Update Interface Controller settings
void oscStatus(bool which){
  OSCMessage msg("/temp");
  if(!which){
    msg.setAddress("/mrtctrl/setUp"); 
  }
  else{
    msg.setAddress("/mrtctrl/status");
  }
  msg.add(goFlg);
  msg.add(int(oldSpd));
  msg.add(int(oldAcc));
  msg.add((bool)digitalRead(DIR_PIN));
  msg.add(fwdLim);
  msg.add(revLim);
  msg.add((bool)digitalRead(SOFT));
  msg.add((bool)digitalRead(HARD));
  msg.add(int(timeVal));
  msg.add(int(timeFlg));
  msg.add(int(oldEnc));
  Udp.beginPacket(remoteIp, port);
    msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}


// E-Stop triggered
void ISReStop(){
  //detachInterrupt(HARD);
  //oscStatus(1);
  //Serial.println("---------------");
  //Serial.println("E-Stop...");
  spdTimer.end();                                 // Turn off timed speed change
  analogWrite(ACC_OUT, 0);
  analogWrite(SPD_OUT, 0);
  //while(!digitalRead(HARD)){
    //analogWrite(SPD_OUT, 0);
    //analogWrite(ACC_OUT, 0);
  //}
  //Serial.println("Released");
  //Serial.println("---------------");
  eStopFlg = 1;
  goFlg = 0;
  oldSpd = 0;
  oldAcc = 0;
  once = 1;
  //attachInterrupt(HARD, ISReStop, FALLING);  
}

// Soft Stop triggered
void ISRslowStop(){
  spdTimer.end();                                 // Turn off timed speed change
  analogWrite(SPD_OUT, 0);
  analogWrite(ACC_OUT, 1000);
  oldSpd = 0;
  oldAcc = 50;
  slowStopFlg = 1;
  goFlg = 0;
}
/*
// Forward Limit switched
void ISRfwdLimit(){
  cli();
  if(!fwdLim){
    //Serial.println("---------------");
    //Serial.println("Forward Limit!");
    if(digitalRead(DIR_PIN)){
      analogWrite(SPD_OUT, 0);
      analogWrite(ACC_OUT, 0);
      spdTimer.end();                                 // Turn off timed speed change
      fwdLim = 1;
    }
    oscStatus(1);
    //Serial.println("---------------");
  }   
  sei();
}

// Reverse Limit switched
void ISRrevLimit(){
  Serial.println("---------------");
  Serial.println("Reverse Limit!");
  cli();
  if(!digitalRead(DIR_PIN)){
    analogWrite(SPD_OUT, 0);
    analogWrite(ACC_OUT, 0);
    spdTimer.end();                                 // Turn off timed speed change
  }
  revLim = 1;
  oscStatus(1);
  sei();
  Serial.println("---------------"); 
}
*/
/*
// E-Stop triggered
void eStop(){
  Serial.println("---------------");
  Serial.println("E-Stop...");
  spdTimer.end();                                 // Turn off timed speed change
  analogWrite(SPD_OUT, 0);
  analogWrite(ACC_OUT, 0);
  while(!digitalRead(HARD)){
    analogWrite(SPD_OUT, 0);
    analogWrite(ACC_OUT, 0);
  }
  Serial.println("Released");
  Serial.println("---------------");
  oldSpd = 0;
  oldAcc = 50; 
}
*/
/*
// Soft Stop triggered
void slowStop(){
  Serial.println("---------------");
  Serial.println("Soft Stop...");
  spdTimer.end();                                 // Turn off timed speed change
  analogWrite(SPD_OUT, 0);
  analogWrite(ACC_OUT, 495);
  oscStatus(1);
  while(!digitalRead(SOFT)){
    analogWrite(SPD_OUT, 0);
    //analogWrite(ACC_OUT, 495);  
  }
  Serial.println("Released");
  Serial.println("---------------");
  oldSpd = 0;
  oldAcc = 50;
}
*/
// Forward Limit switched
void fwdLimit(){
  if(!fwdLim){
    Serial.println("---------------");
    Serial.println("Forward Limit!");
    if(digitalRead(DIR_PIN)){
      analogWrite(SPD_OUT, 0);
      analogWrite(ACC_OUT, 0);
      spdTimer.end();                                 // Turn off timed speed change
      fwdLim = 1;
    }
    oscStatus(1);
    Serial.println("---------------");
  }   
}

// Reverse Limit switched
void revLimit(){
  if(!revLim){
    Serial.println("---------------");
    Serial.println("Reverse Limit!");
    if(!digitalRead(DIR_PIN)){
      analogWrite(SPD_OUT, 0);
      analogWrite(ACC_OUT, 0);
      spdTimer.end();                                 // Turn off timed speed change
      revLim = 1;
    }
    oscStatus(1);
    Serial.println("---------------"); 
  }
}

// Change Direction
void dirSelect(){
  Serial.println("---------------");
  Serial.println("New Direction...");
  Serial.print("Requested = ");
  Serial.println(dirBuff);
  if(dirBuff[0] == 'f' && !fwdLim){
    digitalWrite(DIR_PIN, HIGH);
    Serial.println("FORWARD!");  
  }
  else if(dirBuff[0] == 'r' && !revLim){
    digitalWrite(DIR_PIN, LOW);
    Serial.println("REVERSE!");  
  }
  else{
    Serial.println("What??? Limit???");
  }
  Serial.println("---------------");
  oscStatus(1);
}

// Set new Speed level
void spdChange(){
  if(newSpd > 100){
  newSpd = 100;
  }
  Serial.println("---------------");
  Serial.print("New Speed = ");
  Serial.println(newSpd);
  Serial.print("Old Speed = ");
  Serial.println(oldSpd);
  if(newSpd != oldSpd){
    if(goFlg){
      if(!timeFlg){
        analogWrite(SPD_OUT, uint32_t(newSpd*40.95));
      }
      else{
        timedChange(1);
      }
    }
    Serial.println("New and Sparkly Speed");
    Serial.print("New Speed = ");
    Serial.println(newSpd);
    Serial.print("DAC Write = ");
    Serial.println(uint32_t(newSpd*40.95));
    Serial.print("Volts Out = ");
    Serial.println((newSpd*40.95)*dacRes);
    Serial.print("0-10 Volts = ");
    Serial.println(((newSpd*40.95)*dacRes)*tenVolt);
    Serial.print("0-5 Volts = ");
    Serial.println(((newSpd*40.95)*dacRes)*fiveVolt);      
  }
  if(newSpd == oldSpd){
    Serial.println("That's old Speed... Try again");
  }
  //if(timeFlg && !goFlg){
    //delay(1);
  //}
  //else{
    //oldSpd = newSpd;
  //}
  oldSpd = newSpd;
  Serial.println("---------------"); 
  oscStatus(1);
}

// Set new Accel/Decel level
void accChange(uint32_t newAcc){
  if(newAcc > 100){
  newAcc = 100;
  }
  Serial.println("---------------");
  if(newAcc != oldAcc){
    analogWrite(ACC_OUT, uint32_t(newAcc*40.95));
    Serial.println("New and Sparkly Accel/Decel");
    Serial.print("New Accel = ");
    Serial.println(newAcc);
    Serial.print("DAC Write = ");
    Serial.println(uint32_t(newAcc*40.95));
    Serial.print("Volts Out = ");
    Serial.println((newAcc*40.95)*dacRes);
    Serial.print("0-10 Volts = ");
    Serial.println(((newAcc*40.95)*dacRes)*tenVolt);
    Serial.print("0-5 Volts = ");
    Serial.println(((newAcc*40.95)*dacRes)*fiveVolt);      
  }
  if(newAcc == oldAcc){
    Serial.println("That's old Accel/Decel... Try again");
  }
  oldAcc = newAcc;
  Serial.println("---------------"); 
  oscStatus(1);
}

// Timed speed change
void timedChange(bool when){
  //uint32_t size;
  Serial.println("Timed Accel/Decel");
  Serial.print("when = ");
  Serial.println(when);
  analogWrite(ACC_OUT, 0);
  //bool way = 0;
  way = 0;
  uint32_t timeSpd;
  if(!when){
    oldSpd = 0;
  }
  if(oldSpd < newSpd){
    timeSpd = newSpd-oldSpd;
    way = 1;
  }
  else{
    timeSpd = oldSpd-newSpd;
  }   
  Serial.print("New Speed = ");
  Serial.println(newSpd);
  Serial.print("Old Speed = ");
  Serial.println(oldSpd);
  Serial.print("timeSpd = ");
  Serial.println(timeSpd);
  float temp = ((float(timeVal)/float(timeSpd))*1000000.);
  uint32_t wait = temp;
  Serial.print("wait = ");
  Serial.println(wait);
  Serial.print("way = ");
  Serial.println(way);
  Serial.print("oldSpd = ");
  Serial.println(oldSpd);
  Serial.print("newSpd = ");
  Serial.println(newSpd);
  howMany = 0;
  //for(uint8_t x = 0; x <= timeSpd; x++){   
  spdTimer.begin(timedSpeed, wait);
  while(howMany <= timeSpd){ 
    stops();                                            // Any stops or limits triggered?  
    oscRx(); 
/*
    if(way){
      analogWrite(SPD_OUT, uint32_t((oldSpd+x)*40.95));
      Serial.print("oldSpd+x = ");
      Serial.println(oldSpd+x);
      Serial.println(uint32_t((oldSpd+x)*40.95));        
    }
    else{
      analogWrite(SPD_OUT, uint32_t((oldSpd-x)*40.95));
      Serial.print("oldSpd-x = ");
      Serial.println(oldSpd-x);
      Serial.println(uint32_t((oldSpd-x)*40.95));  
    }
*/    
    //delay(wait);
  }
  spdTimer.end();
  Serial.println("---------------");
}
// Checks for UDP data and matches OSC addresses
void oscRx(){
  OSCMessage msg;
  uint32_t size;

  if((size = Udp.parsePacket())>0){
  //if(size){  
    while(size--){
      msg.fill(Udp.read());
    }
    if(!msg.hasError()){ 
      // Go command received
      if(msg.fullMatch("/mtrctrl/GO")){
        bool dir = digitalRead(DIR_PIN);
        // Don't go - in fwd direction and on fwd limit
        if(fwdLim && dir){
          Serial.println("Change Direction! On Fwd Limit");
        }
        // Don't go - in rev direction and on rev limit
        else if(revLim && !dir){
          Serial.println("Change Direction! On Rev Limit");
        }
        else if(eStopFlg || slowStopFlg){
          Serial.println("Ummm... E-Stop? Soft Stop?");
        }
        // All good to GO!
        else{
          analogWrite(ACC_OUT, uint32_t(oldAcc*40.95));           // Return old Accel value that was changed with STOP
          goFlg = 1;
          if(!timeFlg){                                           // 'oldSpd == newSpd' skip timedChange if values the same as it locks micro
            analogWrite(SPD_OUT, uint32_t(newSpd*40.95));
          }
          else{
            timedChange(0);
            oldSpd = newSpd;  
          }
          Serial.println("GO!");
          oscStatus(1);
        }
      }
      // Stop command received (Softish Stop)
      else if(msg.fullMatch("/mtrctrl/STOP")){
        analogWrite(ACC_OUT, msg.getInt(0));            // Set Accel level to value sent
        analogWrite(SPD_OUT, 0);                        // Turn off motor
        spdTimer.end();                                 // Turn off timed speed change
        goFlg = 0;                    
        //newSpd = oldSpd;
        //oldSpd = 0;                                     
        Serial.print("New Speed = ");
        Serial.println(newSpd);
        Serial.print("Old Speed = ");
        Serial.println(oldSpd);
        Serial.println("STOP!");
        oscStatus(1);
      }      
      // New Speed level received
      else if(msg.fullMatch("/mtrctrl/speed")){
        newSpd = msg.getInt(0);
        spdChange();
      }
      // New Accel level received
      else if(msg.fullMatch("/mtrctrl/accel")){
        uint32_t data = msg.getInt(0);
        accChange(data);
      }
      // New Direction request received
      else if(msg.fullMatch("/mtrctrl/direction")){
        msg.getString(0,dirBuff,3);
        dirSelect();
      }
      // Accel Timed/Level Time
      else if(msg.fullMatch("/mtrctrl/accelTime")){
        timeVal = msg.getInt(0);
        Serial.print("Accel Time = ");
        Serial.println(timeVal);
      }
      // Accel Timed/Level received
      else if(msg.fullMatch("/mtrctrl/timed")){
        timeFlg = msg.getInt(0);
        Serial.print("Timed = ");
        Serial.println(timeFlg);
      }
      // Heart Beat received
      else if(msg.fullMatch("/mtrctrl/thump")){
        thump();
      }
      else{
        Serial.println("Hmmm.. IDK? Check spelling.");
      }
    }
  }  
}

// Heart Beat
void thump(){
  OSCMessage msg("/reply/thump");
  Udp.beginPacket(remoteIp, port);
    msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}
  
void setup() {
  
  //Stop and ESTOP pins
  pinMode(SOFT, INPUT_PULLUP);
  pinMode(HARD, INPUT_PULLUP);
  attachInterrupt(HARD, ISReStop, FALLING);
  attachInterrupt(SOFT, ISRslowStop, FALLING);

  //Limit Pins
  pinMode(FWD_LIM, INPUT_PULLUP);
  pinMode(REV_LIM, INPUT_PULLUP);
  //attachInterrupt(FWD_LIM, ISRfwdLimit, LOW);
  //attachInterrupt(REV_LIM, ISRrevLimit, LOW);
   
  Serial.begin(9600);
  
  // SPI set up for Teensy 3.6 and WIZ850io
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);    // begin reset the WIZ820io
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);  // de-select WIZ820io
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);   // de-select the SD Card
  digitalWrite(9, HIGH);   // end reset pulse
  
  Ethernet.begin(unitMac,unitIp);
  Udp.begin(inPort);

  // DAC Resolution? 12 bit?
  analogWriteResolution(12);

  // Set start up levels
  analogWrite(SPD_OUT, 0);
  analogWrite(ACC_OUT, uint32_t(oldAcc*40.95));

  // Set up direction (fwd)
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);

  // Hack
  dirBuff[3] = '\0';

  Serial.println("Controller GO!");

  // Set interface controller settings
  for(uint8_t x = 0; x < 5; x++){
     oscStatus(0);
     //thump();
    delay(500);    
  }

  //Set up encoder
  enc.write(0); 
}

void loop(){
  stops();                        // Any stops or limits triggered?                   
  if(digitalRead(FWD_LIM)){
    fwdLim = 0;
  }
  if(digitalRead(REV_LIM)){
    revLim = 0;
  }
  if(digitalRead(HARD)){
    eStopFlg = 0;
  }
  else if(once){
    oscStatus(1);
    once = 0;  
  }
  if(digitalRead(SOFT)){
    slowStopFlg = 0;
  }
  else if(once){
    oscStatus(1);
    once = 0;  
  }
  oscRx();                      // Any OSC commands?
  if(!eStopFlg || !slowStopFlg){
    oscStatus(1);                 // Update interface controller settings
  }
  newEnc = (enc.read()/encRes);
  if(newEnc != oldEnc){
    //Serial.print("Enc value = ");
    //Serial.println(newEnc);
    oldEnc = newEnc;
    oscStatus(1);
  }

}
