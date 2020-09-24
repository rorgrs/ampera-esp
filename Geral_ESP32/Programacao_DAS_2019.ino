#include <SD.h>//inclui a biblioteca do sd
#include <DS3231.h>//inclui a biblioteca do Modulo RTC
#include <Amp_CAN.h>//biblioteca de comunicação CAN
#include <SPI.h>//inclui a biblioteca do protocolo SPI
#include <Wire.h>//inclui a biblioteca Wire

#define SPI_SS_CAN 10//Endereço do CAN
#define SPI_SS_SD 9//Endereço do SD
#define LED_DAQ 5//LED de falhas, ou outras avisos
#define MPU_addr 0x69//endereço I2C do MPU
#define falhaBMS 2 //Definição de pino de entrada de falha do BMS
#define falhaIMD 3 //Definição de pino de entrada de falha do IMD
#define falhaBSPD 4 //Definição de pino de entrada de falha do BSPD

File txt;//ponteiro txt para texto do SD criado
DS3231  RTC(SDA, SCL);// config do RTC qual pino data e clock
UNO CAN(SPI_SS_CAN);//cria ponteiro de msg CAN so sistema
Frame msg_CAN_read, msg_DIF_send, msg_PAINEL_send;//cria variável para comunicação CAN
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //Variaveis adquiridas pelo MPU6050 acelerometro, temperatura e giroscópio


void setup() {
  Serial.begin(9600);

  //SPI.setClockDivider(2);
  
  RTC.begin(); // inicializa o módulo de data e hora
  
  CAN.setup();//Inicializa o módulo CAN
  
  pinMode(LED_DAQ, OUTPUT);//led como saida


  Wire.begin();//Inicializa o MPU (acelerômetro, temperatura e giroscópio)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);


  Serial.print("Initializando SD card...");
  if (!SD.begin(SPI_SS_SD)) {//serve pra ver se o sd esta tudo certo
    Serial.println("initialization FAILED!");
    return;
  }
  
  txt = SD.open("dados.txt", FILE_WRITE); //O nome do arquivo deve conter apenas letras // sintaxe = arquivo, acao) com numeros nao funciona
  txt.print("Dados a seguir sao referentes aos dia: ");
  txt.println(RTC.getDateStr()); // dia
  
  txt.println("Hora: \t ID CAN: \t Bytes 1 e 2: \t Bytes 3 e 4: \t Bytes 5 e 6: \t Bytes 7 e 8:  \t  AcX: \t AcY: \t AcZ: \t Temperatura: \t GyX: \t GyY: \t GyZ: \t Falha BMS: \t Falha IMD: \t Falha BSPD: \t");
  txt.println();
  txt.close();

  pinMode(falhaBMS,INPUT);
  pinMode(falhaIMD,INPUT);
  pinMode(falhaBSPD,INPUT);
  
  Serial.println("initialização COMPLETA.");
}


void loop() {
  lerCAN();// Chama function lerCAN (abaixo)que adquire o que estiver disponível no barramento CAN
  lerMPU();// Chama function lerMPU (abaixo)
  escreverSD(); // Chama function escreverSD (abaixo)
}

void lerCAN(){
  CAN.read(&msg_CAN_read); //Guarda o valor de leitura no endereco do frame "msg"
  if (msg_CAN_read.data != 0){//se a mensagem for diferente de 0 liga o led, led pisca por que as vezes o arduino tenta pegar a msg em qualquer momento e em alguns momentos o mcp ainda nao recebeu outra mensagem e ja apagou a antiga
    digitalWrite(LED_DAQ,HIGH);
  }
}

void lerMPU(){
  Wire.beginTransmission(MPU_addr); //aceletrometro e giroscopio
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)// escolhe qual o primeiro registrador que a gente quer
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers / aqui pega os 14 registradores subsequentes
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    //aceleracao nos 3 eixos
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
//  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void escreverSD (){
  txt = SD.open("dados.txt", FILE_WRITE); //O nome do arquivo deve conter apenas letras // sintaxe = arquivo, acao) com numeros nao funciona
  if(txt){
    txt.print(RTC.getTimeStr()); // hora
    txt.print("\t");
    txt.print(msg_CAN_read.id, HEX); //ID CAN
    txt.print("\t");
    txt.print( (int)(msg_CAN_read.data >> 48));
    txt.print("\t");
    txt.print( (int)(msg_CAN_read.data >> 32));
    txt.print("\t");
    txt.print( (int)(msg_CAN_read.data >> 16));
    txt.print("\t");
    txt.print( (int)(msg_CAN_read.data >> 0));
    txt.print("\t");
    
    txt.print(AcX);
    txt.print("\t");
    txt.print(AcY);
    txt.print("\t");
    txt.print(AcZ);
    txt.print("\t");
    txt.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
    txt.print("\t");
    txt.print(GyX);
    txt.print("\t");
    txt.print(GyY);
    txt.print("\t");
    txt.print(GyZ);
    txt.print("\t");
/*    txt.print(int (digitalRead(falhaBMS))); 
    txt.print("\t");
    txt.print(int (digitalRead(falhaIMD))); 
    txt.print("\t");
    txt.println(int (digitalRead(falhaBSPD)));
  */  
    txt.close();
  }
  else{  //  Verifica se abriu o arquivo
    digitalWrite(LED_DAQ,LOW);
    Serial.println ("Erro no SD");
  }
}