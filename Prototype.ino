#include <Adafruit_INA219.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

Adafruit_INA219 sensor;

/*
[INA219&BH1750]
SCL - 20
SDA - 21

[SD]
MOSI - 51
MISO - 50
SCK - 52
CS - 53

[Motor 1]
DIR - 4
STEP - 5
EN - 15
MS1 - 12
MS2 - 11
MS3 - 10

[Motor 2]
DIR - 24
STEP - 25
EN - 35
MS1 - 32
MS2 - 31
MS3 - 30
*/

int BH1750address = 0x23; //芯片地址为16位23
int x; //Motor转动
byte buff[2];

String INA219() { //INA219 测量电压和电流
    float busVoltage = 0;
    float current = 0; // 以毫安为单位
    float power = 0;
    String resu = "";
    
    busVoltage = sensor.getBusVoltage_V();//获取电压值
    current = sensor.getCurrent_mA();// 获取电流值
    power = busVoltage * (current/1000);// 计算功率
    
    Serial.print("Bus Voltage:"); 
    Serial.print(busVoltage);
    Serial.print(" V;");
    
    
    Serial.print("Current:");
    Serial.print(current);
    Serial.print(" mA;");
    
    Serial.print("Power:");
    Serial.print(power);
    Serial.println(" W");
  
    resu += (String(busVoltage) + " " + String(current) + " " + String(power));
    return resu;
    
    delay(2000);
}

int BH1750_Read(int address){ //读光强度
    int i=0;
    Wire.beginTransmission(address);
    Wire.requestFrom(address, 2);
    while(Wire.available()){
        buff[i] = Wire.read();
        i++;
    }
    Wire.endTransmission();
    return i;
}

void BH1750_Init(int address){ //写入光强度
    Wire.beginTransmission(address);
    Wire.write(0x10);
    Wire.endTransmission();
}

int BH1750(){ //BH1750测量光强度值
    int i;
    uint16_t value=0;
    BH1750_Init(BH1750address);
    delay(200);
  
    if(2==BH1750_Read(BH1750address)){
        value=((buff[0]<<8)|buff[1])/1.2;
        Serial.println(value);
    }
    
    return value;
    delay(150);
}

int SDReadWrite(String dataString){ //SD卡读写
    File dataFile = SD.open("datalog.txt", FILE_WRITE);  
  
    if (dataFile){
        dataFile.println(dataString);
        dataFile.close();
        Serial.println(dataString);
    }
    else {
        Serial.println("error opening datalog.txt");
    }
}

int Motor1(){ //Motor1 转动
    digitalWrite(4,HIGH); // Set Dir high
    
    for(x = 0; x < 200; x++){ // Loop 200 times
        digitalWrite(5,HIGH); // Output high
        delayMicroseconds(800); // Wait 1/2 a ms
        digitalWrite(5,LOW); // Output low
        delayMicroseconds(800); // Wait 1/2 a ms
    }
    delay(1000); // pause one second
    
    digitalWrite(4,LOW); // Set Dir low
    
    for(x = 0; x < 200; x++){ // Loop 200 times
        digitalWrite(5,HIGH); // Output high
        delayMicroseconds(800); // Wait 1/2 a ms
        digitalWrite(5,LOW); // Output low
        delayMicroseconds(800); // Wait 1/2 a ms
    }
    delay(1000); // pause one second
}

int Motor2() { //Motor2 转动
    digitalWrite(24,HIGH); // Set Dir high
    
    for(x = 0; x < 200; x++){ // Loop 200 times
        digitalWrite(25,HIGH); // Output high
        delayMicroseconds(800); // Wait 1/2 a ms
        digitalWrite(25,LOW); // Output low
        delayMicroseconds(800); // Wait 1/2 a ms
    }
    delay(1000); // pause one second
    
    digitalWrite(24,LOW); // Set Dir low
    
    for(x = 0; x < 200; x++){ // Loop 200 times
        digitalWrite(25,HIGH); // Output high
        delayMicroseconds(800); // Wait 1/2 a ms
        digitalWrite(25,LOW); // Output low
        delayMicroseconds(800); // Wait 1/2 a ms
    }
    delay(1000); // pause one second
}

void setup() {
    Serial.begin(9600);
    sensor.begin(); //INA219 begin
  
    Wire.begin(); //I2C begin
    
    pinMode(15,OUTPUT); // Enable MTR1
    pinMode(5,OUTPUT); // Step MTR1
    pinMode(4,OUTPUT); // Dir MTR1
    digitalWrite(15,LOW); // Set Enable low MTR1
  
    pinMode(35,OUTPUT); // Enable MTR2
    pinMode(25,OUTPUT); // Step MTR2
    pinMode(24,OUTPUT); // Dir MTR2
    digitalWrite(35,LOW); // Set Enable low MTR2
  
    Serial.println("Initiating...");
}

void loop() {
    SDReadWrite(INA219());
    String lux = "Light intensity:";
    lux += BH1750();
    SDReadWrite(lux);
}
