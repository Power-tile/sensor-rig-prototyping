/// Mark needed library dependency here
/// #include <...>

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
 * The change in the stepper motor degree for rotation.
 * This is NOT the step of the stepper motor. It is the resolution of rotation.
 */
const int DEG_DELTA = 3;
// TODO need confirmation

const int VMAX = 0, VMIN = -120; 
const int HMAX = 0, HMIN = -180;

int horizontalDegree = 0, verticalDegree = 0; // current horizontal/vertical degree. Init position marks (0, 0); clockwise < 0, counterclockwise > 0.
int maxIntensity, maxHDeg, maxVDeg, maxTime; // maximum light intensity, the corresponding degrees, and the corresponding time.
int horizontalDirection = -1, verticalDirection = -1; // rotation direction
bool flagH = false, flagV = false; // marks if a horizontal-/vertical-rotation is finished. true - finished; false - not finished.

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
 * @param deg The degree of rotation for the motor, in degree notation, integer. Counterclockwise: deg > 0, clockwide: `deg < 0`.
 */
void rotateMotor(int deg, int id) {
    /* TODO Implementation */ 
}

/**
 * Read the intensity value by the id of the intensity sensor.
 * @param id The id of the desired intensity sensor.
 * @return The current intensity value of the desired sensor.
 */
double readIntensity(int id) {
    /* TODO Implementation */
}

/**
 * Saves the value to the SD card along with the index of sensor, corresponding time, and cycle number.
 * @param value The value to be saved.
 * @param index Index of sensor. The numbers 1, 2, 3 mark inert intensity sensors;
 * 4 marks the moving sensor; 5, 6 marks the degrees of different stepper motors; 7 marks error.
 * @param time Corresponding time of the value.
 * @param cycle Id of cycle this turn.
 * 
 */
void saveValue(double value, double index, unsigned long time, int cycle) {
    /* TODO Implementation */
    /// !! IMPORTANT !! Remember to store all four parameters.
}

void setup() {
    /// TODO setup all pinModes

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
        if (maxHDeg != -1000 && maxVDeg != -1000 && maxIntensity != 0) {
            saveValue(maxIntensity, 4, maxTime, cycleCount); // rotating sensor saving max data, along with time
            saveValue(maxHDeg, 5, maxTime, cycleCount); // horizontal stepper motor saving degree data, along with time
            saveValue(maxVDeg, 6, maxTime, cycleCount); // vertical stepper motor saving degree data, along with time
        } else {
            saveValue(-1, 7, getCurrentTime(), cycleCount);
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