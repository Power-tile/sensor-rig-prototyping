/// Mark needed library dependency here
/// #include <...>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <SPI.h>
#include <SD.h>
/**
 * The difference of the return value of getCurrentTime() function between one minute.
 */
const double MINUTE = 60000;

/**
 * Variable marking the starting time of the program.
 */
unsigned long startingTime;

/**
 * Ending time of the last cycle. (1 cycle per minute.)
 */
unsigned long prevCycleTime;

/**
 * Number of cycles run.
 */
int cycleCount = 0;

/**
 * The change in the stepper motor degree for one rotation.
 * This is NOT the step of the stepper motor. It is the resolution of rotation.
 */
const int DEG_DELTA = 3.6;

const int HMAX = 0, HMIN = -180;
const int VMAX = 0, VMIN = -120; 

const int chipSelect = 53; // pin of SD card // TODO need confirmation

int horizontalDegree = 0, verticalDegree = 0; // current horizontal/vertical degree. Init position marks (0, 0); clockwise < 0, counterclockwise > 0.
int maxIntensity, maxHDeg, maxVDeg, maxTime; // maximum light intensity, the corresponding degrees, and the corresponding time.
int horizontalDirection = -1, verticalDirection = -1; // rotation direction
bool flagH = false, flagV = false; // marks if a horizontal-/vertical-rotation is finished. true - finished; false - not finished.

/* TODO put all other constants / pins / variables here */
int BH1750address = 0x23; // 芯片地址为16位23
Adafruit_INA219 currentSensor; // current & voltage sensor
byte buff[2]; // array from light intensity sensor

/**
 * Initializes the rotation variables.
 */
void initRotate() {
    if (horizontalDegree) rotateMotor(HMAX - horizontalDegree, 1);
    if (verticalDegree) rotateMotor(VMAX - verticalDegree, 2);
    horizontalDegree = HMAX; verticalDegree = VMAX;
    horizontalDirection = verticalDirection = -1;
    maxIntensity = 0; maxHDeg = -1000; maxVDeg = -1000;
    flagH = false; flagV = false;
}

/**
 * Initializes the vertical rotation variables.
 */
void initRotateV() {
    if (verticalDegree) rotateMotor(VMAX - verticalDegree, 2);
    flagV = false;
    verticalDegree = VMAX;
    verticalDirection = -1;
}

/**
 * Gets the current time in unsigned long.
 * @return Current time in unsigned long, relatively.
 */
unsigned long getCurrentTime() {
    return millis();
}

/**
 * Rotates the desired motor.
 * @param id The number of the desired motor. 1 refers to the horizontal rotation, and 2 refers to the vertical rotation.
 * @param deg The degree of rotation for the motor, in degree notation, integer. Counterclockwise: deg > 0, clockwide: deg < 0.
 */
void rotateMotor(int deg, int id) {
    /**
     * Full step (MS1, MS2, MS3 = LOW) 1.8 degrees = 1 step
     */
    int dirPin = (id == 1) ? 4 : 24;
    int stepPin = (id == 1) ? 5 : 25;

    if (deg >= 0) {
        digitalWrite(dirPin, HIGH); // Set Dir high
    } else {
        digitalWrite(dirPin, LOW); // Set Dir low
    }

    for (int x = 0; x < deg / 1.8; x++){ // Loop 200 times
        digitalWrite(stepPin, HIGH); // Output high
        delayMicroseconds(800); // Wait 1/2 a ms
        digitalWrite(stepPin, LOW); // Output low
        delayMicroseconds(800); // Wait 1/2 a ms
    }
    delay(1000); // pause one second
}

/**
 * BH1750前置
 * BH1750_init Send measurement command
 * BH1750_Read Read intensity
 */
void BH1750Init(int address) { //发送测量命令 0x10
    Wire.beginTransmission(address);
    Wire.write(0x10);
    Wire.endTransmission();
}

int BH1750Read(int address) { //读光强度
    int i = 0;
    Wire.beginTransmission(address);
    Wire.requestFrom(address, 2);
    while (Wire.available()){
        buff[i] = Wire.read();
        i++;
    }
    Wire.endTransmission();
    return i;
}

/**
 * Read the intensity value by the id of the intensity sensor.
 * @param id The id of the desired intensity sensor (1~4).
 * @return The current intensity value of the desired sensor.
 */
double readIntensity(int id) {
    uint16_t value = 0;
    delay(200);
  
    if (BH1750Read(BH1750address) == 2) { // TODO clarify condition
        value = ((buff[0] << 8) | buff[1]) / 1.2;
        Serial.println(value);
    }
    
    return value;
}

/**
 * Read the current from the current sensor.
 * @return Current value.
 */
double readCurrent() {
    float current = currentSensor.getCurrent_mA(); // 获取电流值 unit: mA 
    
    Serial.print("Current:");
    Serial.print(current);
    Serial.println(" mA;");
  
    return (double)current;
}

/**
 * Read the voltage from the current sensor.
 * @return Voltage magnitude.
 */
double readVoltage() {
    float busVoltage = currentSensor.getBusVoltage_V(); // 获取电压值
    
    Serial.print("Bus Voltage:"); 
    Serial.print(busVoltage);
    Serial.println(" V;");

    return (double)busVoltage;
}

/**
 * Saves the value to the SD card along with the index of sensor, corresponding time, and cycle number.
 * @param value The value to be saved.
 * @param index Index of sensor. The numbers 1, 2, 3 mark inert intensity sensors;
 * 4 marks the moving sensor; 5, 6 marks the degrees of different stepper motors; 7 marks current; 8 marks voltage;
 * 9 marks error.
 * @param time Corresponding time of the value.
 * @param cycle Id of cycle this turn.
 */
void saveValue(double value, double index, unsigned long time, int cycle) {
    /// !! IMPORTANT !! Remember to store all four parameters. (-1, value, index, time, cycle)
    File dataFile = SD.open("datalog.csv", FILE_WRITE);
    String dataString = "";
  
    if (dataFile) {
        dataString += String(value) + "," + String(index) + "," + String(time) + "," + String(cycle);
        dataFile.println(dataString);
        dataFile.close();
        Serial.println(dataString);
    } else {
        Serial.println("error opening datalog.csv");
    }
}

void setup() {
    /// TODO setup all pinModes
    Serial.begin(9600);
    currentSensor.begin(); //INA219 begin
  
    Wire.begin(); //I2C begin
    
    pinMode(15, OUTPUT); // Enable MTR1
    pinMode(5, OUTPUT); // Step MTR1
    pinMode(4, OUTPUT); // Dir MTR1
    digitalWrite(15, LOW); // Set Enable low MTR1
  
    pinMode(35, OUTPUT); // Enable MTR2
    pinMode(25, OUTPUT); // Step MTR2
    pinMode(24, OUTPUT); // Dir MTR2
    digitalWrite(35, LOW); // Set Enable low MTR2

    BH1750Init(BH1750address); // initialize light intensity sensor

    // TODO 判断是否已经有datalog.csv 如果已经有了 把原来的file重命名为old_data.csv 新建一个datalog.csv
  
    Serial.println("Initiating...");

    startingTime = getCurrentTime(); // Initialize the starting time
    prevCycleTime = startingTime; // Cycle begins.

    delay(10); // delay 10ms before first cycle
}

void loop() {    
    if (getCurrentTime() - prevCycleTime >= MINUTE) { // difference of time > 1 minute
        cycleCount++; // this cycle ends

        saveValue(readIntensity(1), 1, getCurrentTime(), cycleCount); // sensor 1 saving data
        saveValue(readIntensity(2), 2, getCurrentTime(), cycleCount); // sensor 2 saving data
        saveValue(readIntensity(3), 3, getCurrentTime(), cycleCount); // sensor 3 saving data
        saveValue(readCurrent(), 7, getCurrentTime(), cycleCount); // save current data
        saveValue(readVoltage(), 8, getCurrentTime(), cycleCount); // save voltage data
        if (maxHDeg != -1000 && maxVDeg != -1000 && maxIntensity != 0) {
            saveValue(maxIntensity, 4, maxTime, cycleCount); // rotating sensor saving max data, along with time
            saveValue(maxHDeg, 5, maxTime, cycleCount); // horizontal stepper motor saving degree data, along with time
            saveValue(maxVDeg, 6, maxTime, cycleCount); // vertical stepper motor saving degree data, along with time
        } else {
            saveValue(-1, 9, getCurrentTime(), cycleCount);
        }

        initRotate(); // initialize rotation values
        prevCycleTime = getCurrentTime(); // mark new cycle start time
    }

    if (!flagH) { // full rotation not complete
        if (!flagV) { // vertical rotation not completed
            rotateMotor(DEG_DELTA * verticalDirection, 2); // rotates v-motor
            verticalDegree += DEG_DELTA * verticalDirection; // update v-degree
            if (verticalDegree < VMIN) verticalDirection = 1; // rotation exceeds VMIN, rotate back
            else if (verticalDegree > VMAX) flagV = true; // vertical rotation complete
        } else { // vertical rotation completed
            initRotateV(); // initialize vertical motor
            rotateMotor(DEG_DELTA * horizontalDirection, 1); // rotates h-motor
            horizontalDegree += DEG_DELTA * horizontalDirection; // update h-degree
            if (horizontalDegree < HMIN) horizontalDirection = 1; // rotation exceeds HMIN, rotate back
            else if (horizontalDegree > HMAX) flagH = true; // rotation complete
        }

        double intensity = readIntensity(4); // read the intensity of the rotating sensor
        if (intensity > maxIntensity) { // recorded a maximum intensity
            maxIntensity = intensity;
            maxHDeg = horizontalDegree; // corresponding h-degree
            maxVDeg = verticalDegree; // corresponding v-degree
            maxTime = getCurrentTime(); // corresponding time
        }
    }

    delay(10); // delays 10ms after each rotation
}
