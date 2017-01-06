//=================================================
// Shield L293D pour Arduino
// tiptopboards.com 23 11 2013
// Adpapté de  :
// Adafruit program et arduino.cc user "Krodal".  June 2012
// Open Source / Public Domain, pour Arduino 1.0.1
//=================================================
// Sorties pont en H -> moteurs DC, relais, éclairage
// Pas de moteur pas à pasavec de programme
// Les 2 servos utilisent la librairie Servo.h
// 4 moteurs DC au maximum (ou 8 sorties)
//==================================================
#include <Servo.h>   //Librairie pour les 2 servomoteurs
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <String.h>

#define DEBUG true

// Capteur Ultrason
#define CAPTEUR_TRIG 2
#define CAPTEUR_ECHO 7
#define CAPTEUR_PROPO 58

// Pins Arduino pour le registre à décalage 4 7 8 12
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

// Bus 8-bit en sortie du registre à décalage 74HC595 
// Utilisés pour fixer la direction des ponts de commande
#define MOTOR1_A 2  //ce ne sont apsdes pins Arduino
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4

// Pins Arduino pour les signaux PWM (moteurs et servos) 3 5 6 9 10 11
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define SERVO1_PWM 10

// Codes pour les fonctions de moteur
#define FORWARD 1     //4 modes de commande 
#define BACKWARD 2    //avant arrière frein stop
#define BRAKE 3
#define RELEASE 4

// BT
#define BT_RX 3
#define BT_TX 5
SoftwareSerial bt(BT_TX, BT_RX);

// Declaration classes de Servo 
Servo servo_1;
int current_angle;

// RFID Stuff
#define RST_PIN         9  //Port lecteur RFID           
#define SS_PIN          10
MFRC522 mfrc522(SS_PIN, RST_PIN); 
MFRC522::MIFARE_Key key;
unsigned long currentTime = 0;
unsigned long timeOut = 100;

// LCD
LiquidCrystal_I2C lcd(0x20, 18, 19);

//Initialisations
void setup()
{
  Serial.begin(9600);
  if(DEBUG)
    Serial.println("[DEBUG] Starting the systems.");
  bt.begin(9600);
  if(DEBUG)
    Serial.println("[DEBUG] Bluetooth systems setup complete.");
  SPI.begin(); 
  mfrc522.PCD_Init();
  if(DEBUG)
    Serial.println("[DEBUG] RFID systems setup complete.");
  pinMode(CAPTEUR_TRIG, OUTPUT);
  pinMode(CAPTEUR_ECHO, INPUT);
  if(DEBUG)
    Serial.println("[DEBUG] Ultrasound systems setup complete.");
  lcd.clear();
  lcd.init();
  lcd.print("'NotWorkingTech'");
  lcd.setCursor(0,1);
  lcd.print(" IMT Atlantique");
  lcd.backlight(); // lcd.backlight();
}

//Programme principal
void loop(){
  updateClosestObject();
  tryRFIDCardRead(4,18);
  listenToRemoteCommands();
}

//#############################################################
//#                  RFID Functionationality                  #
//#############################################################

void tryRFIDCardRead(unsigned char block, unsigned char bufSize){  
   currentTime = millis();
   while ( ! mfrc522.PICC_IsNewCardPresent() ) {
    if (millis() > currentTime + timeOut ) 
      return ;
    //Serial.println("[DEBUG] RFID: Not Present");
    mfrc522.PCD_Init();
  }
   String res = readRFIDCard(block, bufSize);
   Serial.print("RFID: Block 4 read");
   res = res+ readRFIDCard(9, bufSize);
   writeToCard(12, "NotWorkingTech");
   if(res.length()!=0)
    sendOverBT(res);
}


void writeToCard(unsigned char block, unsigned char data[])
{
 
  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
  currentTime = millis();
   while ( ! mfrc522.PICC_IsNewCardPresent() ) {
    if (millis() > currentTime + timeOut ) 
      return Serial.println("NO1");
    //Serial.println("[DEBUG] RFID: Not Present");
    mfrc522.PCD_Init();
  }
  if ( ! mfrc522.PICC_ReadCardSerial()) {  
    Serial.print("No"); 
    return;
    }
  MFRC522::StatusCode status;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print("PCD_Authenticate() failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }
  else Serial.println("PCD_Authenticate() success: ");

  status = mfrc522.MIFARE_Write(block, data, 16);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("MIFARE_Write() failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }
  else {
    Serial.println("MIFARE_Write() success: ");
    Serial.println(block);
    }
  
}

String readRFIDCard(unsigned char block, unsigned char bufSize)
{  
  String res = "";
  unsigned char buf[bufSize];
  memset(buf, 0xFF, bufSize);
  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
  // Look for new cards
  if ( ! mfrc522.PICC_ReadCardSerial())    return res;
  MFRC522::StatusCode status;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print("[ERROR] RFID: PCD_Authenticate() failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return res;
  }
  else Serial.println("[DEBUG] RFID: PCD_Authenticate() success: ");
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(block, buf, &bufSize);
  if (status != MFRC522::STATUS_OK) {
      Serial.print(F("[ERROR] RFID: MIFARE_Read() failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
  }
  String data = "";
  for(int i=0;i<bufSize;i++){
    if(isalpha((char)buf[i]))
      data=data+String((char)buf[i]);
  }
  unsigned long hex_num;
  hex_num =  mfrc522.uid.uidByte[0] << 24;
  hex_num += mfrc522.uid.uidByte[1] << 16;
  hex_num += mfrc522.uid.uidByte[2] <<  8;
  hex_num += mfrc522.uid.uidByte[3];
  res = "I"+String(hex_num)+";\nT"+data+";\n";
  return res;
}

//#############################################################
//#               Bluetooth Functionationality                #
//#############################################################
char MOVE_FOREWARD = 'F';
char MOVE_BACKWARD = 'B';
char TURN_LEFT = 'L';
char TURN_RIGHT = 'R';
char STOP = 'S';
int SPEED_PERC = 50;
unsigned long BT_TIMEOUT = 100;

bool mot_direction;

/**
 * Envoie une chaine de caractère avec bluetooth.
 */
void sendOverBT(String str){
  Serial.println("Entering sendOverBT : "+str);
  for(int i=0;i<str.length();i++){    
    bt.write(str.charAt(i));
  }
}

/**
* Ecoute sur le port Bluetooth et traite la commande.
* Time out après BT_TIMEOUT millisecondes.
**/
void listenToRemoteCommands(){
    unsigned long initial_time = millis();
    //Serial.println(String("[DEBUG] Entering listenToRemoteCommands()"));
    String incoming = "";
    bool keep_going = true;
    char current;
    long diff = 0;
    while(keep_going){
        diff = millis()-initial_time;
        if(diff>BT_TIMEOUT){
          //Serial.println(String("[DEBUG] listenToRemoteCommands() Timed-out"));
          break;
        }
        current = bt.read();
        if((int)current == -1)
          continue;
        if(current=='\n')
          break;
        incoming = incoming + current;        
     }
    if(incoming.length()!=0)
      parseRemoteCommands(incoming);
}

/**
* Traiter la commande.
**/
void parseRemoteCommands(String comm){
  if(DEBUG)
    Serial.println("[DEBUG] RC: Parsing command: "+comm);
  switch(comm.charAt(0)){
    case 'F':
        move_foreward_indefinetly();
    break;
    case 'B':
        move_backwards_indefinetly();
    break;
    case 'S':
      motion_stop();
    break;
    case 'L':
      if(comm.charAt(1)=='+')
        start_turn_left();
      else
        end_turn();
    break;
    case 'R':
      if(comm.charAt(1)=='+')
        start_turn_right();
      else
        end_turn();
    break;
    case 'O':
    {
      int endIndex = comm.indexOf(';');
      int pitch = comm.substring(1, endIndex-1).toInt();
      int roll = comm.substring(endIndex+1).toInt();
      Serial.println("[DEBUG] RC: Pitch "+String(pitch)+" Roll "+String(roll));
      parseOrientationMsg(pitch, roll);
    }
    break;
    case 'V':
    {
      /*SPEED_PERC = comm.substring(2).toInt();
      if(mot_direction)
        move_foreward_indefinetly();
      else
        move_backwards_indefinetly();
       */
    }
    break;
    default:
      //motion_stop();
      if(DEBUG)
          Serial.println(String("[DEBUG] Unidentified command.") + comm);
    break;
  }
}

void parseOrientationMsg(int pitch, int roll){
    if(roll < 45){
      Serial.println("TURN LEFT");
      //start_turn_left();
    }
    if(pitch >= 45 && roll <= -45){
      Serial.println("TURN END");
     // end_turn();
    }
    if(pitch > 25){
      //start_turn_right();
      Serial.println("TURN RIGHT");
    }
    if(roll < -25){
      Serial.println("MV BACKWARDS");
      //move_backwards_indefinetly();
    }
    if(roll >= -25 && pitch <= 25){
      //motion_stop();
      Serial.println("MV STOP");
    }
    if(roll > 25){
      //move_foreward_indefinetly();
      Serial.println("MV FOREWARD");
    }
      
}

//#############################################################
//#                Motoring Functionationality                #
//#############################################################

void updateClosestObject(){
  int dist = (int)sensor_pulse();
  sendOverBT("D"+String(dist)+";\n");
}

void start_turn_right(){
  if (mot_direction){
    Serial.println((int)(255*SPEED_PERC/100));
    motor(2, FORWARD, 255);
    motor(1, FORWARD, 100);
  }  
  else{
    motor(2, BACKWARD, 255);
    motor(1, BACKWARD, 100);   
    
  }
}

void start_turn_left(){
  if (mot_direction){
    motor(2, FORWARD, 100);
    motor(1, FORWARD, 255);
  }  
  else{
    motor(2, BACKWARD, 100);
    motor(1, BACKWARD, 255);  
  }
}

void end_turn(){
  if(mot_direction){
    move_foreward_indefinetly();
  }
  else{
    move_backwards_indefinetly(); 
  }
}

/**
* Retourne la position du capteur en degré.
* @return int La position du capteur.
**/
int sensor_position(){
  return current_angle;
}
/**
* Faire tourner le capteur d'un angle @(angle).
* @param angle L'angle en degrés.
**/
void sensor_turn(int angle){
  servo_1.write(angle);
  current_angle += angle;
  delay(10);
}

/**
* Lance une détection par le biais du capteur.
* @return float La distance à l'objet devant la voiture.
**/
float sensor_pulse(){
  long duration;
  float distance;
  digitalWrite(CAPTEUR_TRIG, LOW);  
  delayMicroseconds(2); 
  digitalWrite(CAPTEUR_TRIG, HIGH);
  delayMicroseconds(10); //Trig déclenché 10ms sur HIGH
  digitalWrite(CAPTEUR_TRIG, LOW);
  // Calcul de l'écho
  duration = pulseIn(CAPTEUR_ECHO, HIGH);
  // Distance proportionnelle à la durée de sortie
  distance = duration/58;
  return distance;
}

/**
* Faire avancer la voiture indéfiniment, jusqu'à appel de @(motion_stop).
**/
void move_foreward_indefinetly(){
  mot_direction = true;  
  motor(1, FORWARD, 255);
  motor(2, FORWARD, 255);
}

/**
* Faire reculer la voiture indéfiniment, jusqu'à l'appel de @(motion_stop).
**/
void move_backwards_indefinetly(){
  mot_direction = false;
  motor(1, BACKWARD, 255);
  motor(2, BACKWARD, 255);
}

/**
* Arrêter le mouvement de la voiture.
**/
void motion_stop(){
  motor(1, BRAKE, 255);
  motor(2, BRAKE, 255);
  motor(1, RELEASE, 0);
  motor(2, RELEASE, 0);
}

/**
* Avancer de X centimètres puis arrêter.
* @param centimeters Nbr. de cm à parcourir.
**/
void move_foreward(float centimeters){
  // Speed in cm/s
  const float forward_speed = 41.5;
  int movement_time = (int)((centimeters/forward_speed)*1000);
  motor(1, FORWARD, 255);
  motor(2, FORWARD, 255);
  delay(movement_time);
  motor(1, BRAKE, 255);
  motor(2, BRAKE, 255);
  motor(1, RELEASE, 0);
  motor(2, RELEASE, 0);
}

/**
* Reculer de X centimètres puis arrêter.
* @param centimeters Nbr. de cm à parcourir.
**/
void move_backward(int centimeters){
  const int backward_speed = 41.5;
  int movement_time = centimeters/backward_speed;
  motor(1, BACKWARD, 255);
  motor(2, BACKWARD, 255);
  delay(movement_time);
  motor(1, BRAKE, 255);
  motor(2, BRAKE, 255);
  motor(1, RELEASE, 0);
  motor(2, RELEASE, 0);
}

/**
* Tourner à droite de l'angle @(angle).
* @param angle Entier représentant l'angle en degré
**/
void turn_right(int angle){
  // Degré/ms
  const float right_speed = 0.18;
  motor(2, FORWARD, 255);
  delay(angle*right_speed);
  motor(2, BRAKE, 200);
  motor(2, RELEASE, 0);
}

/**
* Tourner à gauche de l'angle @(angle).
* @param angle Entier représentant l'angle en degré
**/
void turn_left(int angle){
  // Degré/ms
  const float left_speed = 0.18;
  motor(1, FORWARD, 255);
  delay(angle*left_speed);
  motor(1, BRAKE, 200);
  motor(1, RELEASE, 0);
}


//#############################################################
//#               Low Level Functionationality                #
//#############################################################

//=== Fonction motor
// Choisir le moteur (1-2), la commande et la vitesse (0-255).
// Les commandes sont : FORWARD, BACKWARD, BRAKE, RELEASE.
void motor(int nMotor, int command, int speed)
{
  int motorA, motorB;

  if (nMotor >= 1 && nMotor <= 4)
  {  
    switch (nMotor)
    {
    case 1:
      motorA   = MOTOR1_A;
      motorB   = MOTOR1_B;
      break;
    case 2:
      motorA   = MOTOR2_A;
      motorB   = MOTOR2_B;
      break;
    default:
      break;
    }

    switch (command)
    {
    case FORWARD:   //Tourner en avant
      motor_output (motorA, HIGH, speed);
      motor_output (motorB, LOW, -1);     // -1: no PWM set
      break;
    case BACKWARD:  //Tourner en arrière
      motor_output (motorA, LOW, speed);
      motor_output (motorB, HIGH, -1);    // -1: no PWM set
      break;
    case BRAKE:   //Freiner
      motor_output (motorA, LOW, 255); // 255: fully on.
      motor_output (motorB, LOW, -1);  // -1: no PWM set
      break;
    case RELEASE:   //Stop
      motor_output (motorA, LOW, 0);  // 0: output floating.
      motor_output (motorB, LOW, -1); // -1: no PWM set
      break;
    default:
      break;
    }
  }
}


//=== Fonction motor_output
// Utilise le driver pour piloter des sorties
// Mettre la  variable high_low sur HIGH / LOW pour des lampes
// On une speed = 0
// speed varie de 0-255 pour les 2 pins (0 = arrêt, 255 = maxi)
// à mettre sur -1 pour ne pas régler de PWM du tout
void motor_output (int output, int high_low, int speed)
{
  int motorPWM;

  switch (output)
  {
  case MOTOR1_A:
  case MOTOR1_B:
    motorPWM = MOTOR1_PWM;
    break;
  case MOTOR2_A:
  case MOTOR2_B:
    motorPWM = MOTOR2_PWM;
    break;
  default:
    // Utilise speed comme flag d'erreur, -3333 = invalid output.
    speed = -3333;
    break;
  }

  if (speed != -3333)   //La valeur speed est valide
  {
    // Set the direction with the shift register 
    // on the MotorShield, even if the speed = -1.
    // In that case the direction will be set, but not the PWM.
    shiftWrite(output, high_low);

    // Ajuster le PWM seulemernt s il est valide
    if (speed >= 0 && speed <= 255)    
    {
      analogWrite(motorPWM, speed);
    }
  }
}


// Fonction shiftWrite
// The parameters are just like digitalWrite().
// The output is the pin 0...7 (the pin behind the shift register).
// The second parameter is HIGH or LOW.
void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;

  // Do the initialization on the fly, 
  // at the first time it is used.
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);

    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);

    // start with all outputs (of the shift register) low
    latch_copy = 0;

    shift_register_initialized = true;
  }

  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);

  // Use the default Arduino 'shiftOut()' function to
  // shift the bits with the MOTORCLK as clock pulse.
  // The 74HC595 shiftregister wants the MSB first.
  // After that, generate a latch pulse with MOTORLATCH.
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}
