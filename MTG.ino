#include <ArduinoBLE.h>
#include <Adafruit_NeoPixel.h>
#define PIXELSPIN 10 // pin neopixels are to be used on
#define NUMPIXELS 16 // amount of neopixels in a single string

Adafruit_NeoPixel pixels(NUMPIXELS, PIXELSPIN, NEO_GRB + NEO_KHZ800);

BLEService deviceConfig("4242");                    // BLE Service for configuration settings
BLEByteCharacteristic deviceMode("0001",            // Setting for which game-mode to start
    BLERead | BLEWrite);                            // remote clients will be able to read or write this
BLEUnsignedLongCharacteristic deviceSessionTime("0002",
    BLERead | BLEWrite);
BLEByteCharacteristic deviceLightTimer("0003",
    BLERead | BLEWrite);
    
BLEService deviceStats("00004243-0000-1000-8000-00805f9b34fb");
BLEByteCharacteristic heartPoll("0001",
    BLERead | BLENotify);
BLEIntCharacteristic punchForceAVG("00000002-0000-1000-8000-00805f9b34fb",
    BLERead | BLENotify);
BLEIntCharacteristic punchForceMin("00000003-0000-1000-8000-00805f9b34fb",
    BLERead | BLENotify);
BLEIntCharacteristic punchForceMax("00000004-0000-1000-8000-00805f9b34fb",
    BLERead | BLENotify);

byte pin[8] = {A0, A1, A2, A3, A4, A5, A6, A7};       // stores pin values for ADC 0-7
byte pressureThreshold[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // stores the modified ADC readings
int analogReading[9];                                // stores raw ADC readings for each sensor
int analogReadingAVG[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int analogReadingMin[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int analogReadingMax[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
byte sequence[8] = {0, 0, 0, 0, 0, 0, 0, 0};         // stores random sequence for pressure pads
byte sequenceLED[8] = {0, 0, 0, 0, 0, 0, 0, 0};      // stores random sequence for LEDs
unsigned long timeStart;                             // tracks when a function starts
unsigned long timeEnd = 0;                           // tracks whenever a function loops

void setup() {
  Serial.begin(9600);                             // initialize serial communication with baud of 9600
  while (Serial);
  randomSeed(analogRead(A0)+analogRead(A1)+       /**/
  analogRead(A2)+analogRead(A3)+analogRead(A4)+   // generate a pseudorandom seed based on analog sensors at startup
  analogRead(A5)+analogRead(A6)+analogRead(A7));  /**/
  analogReadResolution(12);                       // increases ADC bit resolution from 10 to 12

  if (!BLE.begin()) {   // initialize BLE
    Serial.println("starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("Nano33BLE");                              // Set name for connection
  BLE.setAdvertisedService(deviceConfig);                     // Advertise service
  deviceConfig.addCharacteristic(deviceMode);                 // Add characteristic to service
  deviceConfig.addCharacteristic(deviceSessionTime);
  deviceConfig.addCharacteristic(deviceLightTimer);
  BLE.addService(deviceConfig);                               // Add service
  
  BLE.setAdvertisedService(deviceStats);
  deviceStats.addCharacteristic(heartPoll);
  deviceStats.addCharacteristic(punchForceAVG);
  deviceStats.addCharacteristic(punchForceMin);
  deviceStats.addCharacteristic(punchForceMax);
  BLE.addService(deviceStats);
  
  deviceMode.writeValue(3);             // default gamemode of none
  deviceSessionTime.writeValue(30000); // default session time of 3 minutes in milliseconds

  BLE.advertise();             // Start advertising
  Serial.print("Peripheral device MAC: "); Serial.println(BLE.address());
  Serial.println("Waiting for connections...");

  pixels.begin();              // start neopixels
  pixels.clear();              // blank neopixels
  pixels.show();               // updates neopixels
//pixels.setBrightness(50);
}

void loop() {
  BLEDevice central = BLE.central();  // Wait for a BLE central to connect

  if (central) {                                                                    // if a central is connected to the peripheral:
    Serial.print("Connected to central MAC: "); Serial.println(central.address());  // print the central's BT address:
    
    while (central.connected()){            // keep looping while connected through bluetooth
      if (deviceMode.value() == 0) {
        reset();
        Flash();
        compressArrays();
        punchForceAVG.writeValue(analogReadingAVG[8]);
        punchForceMin.writeValue(analogReadingMin[8]);
        punchForceMax.writeValue(analogReadingMax[8]);
        deviceMode.writeValue(3);
        pixels.clear();
        pixels.show();
      }
      else if (deviceMode.value() == 1) {
        reset();
        All9();
        compressArrays();
        punchForceAVG.writeValue(analogReadingAVG[8]);
        punchForceMin.writeValue(analogReadingMin[8]);
        punchForceMax.writeValue(analogReadingMax[8]);
        
        deviceMode.writeValue(3);
        pixels.clear();
        pixels.show();
      }
      else if (deviceMode.value() == 2) {
        reset();
        HandEye();
        compressArrays();
        punchForceAVG.writeValue(analogReadingAVG[8]);
        punchForceMin.writeValue(analogReadingMin[8]);
        punchForceMax.writeValue(analogReadingMax[8]);
        
        deviceMode.writeValue(3);
        pixels.clear();
        pixels.show();
      }
      
      else {
       // rainbow(25);
      }
      
    }
    Serial.print("Disconnected from central MAC: "); Serial.println(central.address());
  }
}

void Flash(){
  timeStart = millis();
  unsigned long blinkDelay = 500;
  for (byte i=2; i<8; i++) { // starts with a 3 long sequence and increments after every successful completion up to a max of 8
    seqStart: // get sent back here if failing to hit correct sensor
    sequenceGenerator(i);
    for (byte j=0; j<=i; j++) {   // loop that blinks simon sequence
      pixels.setPixelColor(sequenceLED[j], pixels.Color(0, 0, 255));
      pixels.setPixelColor(sequenceLED[j]+1, pixels.Color(0, 0, 255));
      pixels.show();
      delay(blinkDelay);
      pixels.clear();
      pixels.show();
      delay(blinkDelay/2);
    }

    byte j = 0;
    while (timeEnd <= timeStart + deviceSessionTime.value()) {  // loops checking sensors until either time runs out or successful completion of sequence
      for (byte k=0; k<8; k++) { // check and store pressure readings
        pressureSensor(k);
      }
      for (byte k=0; k<8; k++) {
        if ((sequence[j] != k) && (pressureThreshold[k] > 0)) { // checks if incorrect sensor was hit
          pixels.setPixelColor(k*2, pixels.Color(255, 0, 0));     //
          pixels.setPixelColor((k*2)+1, pixels.Color(255, 0, 0)); //
          pixels.show();                                          // this block sets leds for incorrect button
          delay(blinkDelay/2);                                    // to flash red very quickly once
          pixels.clear();                                         //
          pixels.show();                                          //
          goto seqStart;  // goes back to generate new sequence due to incorrect hit
        }
      }
      if ((j<=i) && (pressureThreshold[sequence[j]] > 0)) {         // checks if correct sensor was hit
        pixels.setPixelColor(sequenceLED[j], pixels.Color(0, 255, 0));   //
        pixels.setPixelColor(sequenceLED[j]+1, pixels.Color(0, 255, 0)); //
        pixels.show();                                                   // this block sets leds for correct button
        delay(blinkDelay/2);                                             // to flash green very quickly once
        pixels.clear();                                                  //
        pixels.show();                                                   //
        j++;
      }
      else if (j>i) {
        break;
      }
      timeEnd = millis(); 
    }
    if (timeEnd >= timeStart + deviceSessionTime.value()) {
      break;
    }
  }
  pixels.clear();
  pixels.show();
}

void All9(){
  timeStart = millis();
  while(1) {
    for (byte i=0; i<NUMPIXELS; i++) {  // sets all pixels to green
      pixels.setPixelColor(i, pixels.Color(0, 255, 0));
    }
    pixels.show();

    byte n = 8; // number of initial LEDs that are on
    while (timeEnd <= timeStart + deviceSessionTime.value()) {  // loops checking sensors until time runs out
      for (byte i=0; i<8; i++) { // check and store pressure readings
        pressureSensor(i);
      }
      for (byte i=0; i<8; i++) {
        if ((pressureThreshold[i] > 0) && (pixels.getPixelColor(i*2) != 0)) {    // checks if sensor was hit and if LED is still on
          pixels.setPixelColor(i*2, pixels.Color(0, 0, 0));                      //
          pixels.setPixelColor((i*2)+1, pixels.Color(0, 0, 0));                  // sets lights to off for hit sensor
          pixels.show();                                                         // this block sets leds for correct button
          delay(5);      // update LEDs a second time because of Hardware bug? that
          pixels.show(); // causes LED to not turn off until next sensor is hit
          n--; // decrement number of on LEDs
        }
      }
      if (n==0) { //checks if all LEDs are off
        break;
      }
      timeEnd = millis();
    }
    if (timeEnd >= timeStart + deviceSessionTime.value()) {
      break;
    }
  }
  pixels.clear();
  pixels.show();
}

void HandEye(){
  timeStart = millis();
  unsigned long blinkDelay = 500;
  for (byte i=2; i<8; i++) { // starts with a 3 long sequence and increments after every successful completion up to a max of 8
    seqStart: // get sent back here if failing to hit correct sensor
    sequenceGenerator(i);
    for (byte j=0; j<=i; j++) {   // loop that blinks simon sequence
      pixels.setPixelColor(sequenceLED[j], pixels.Color(0, 0, 255));
      pixels.setPixelColor(sequenceLED[j]+1, pixels.Color(0, 0, 255));
      pixels.setPixelColor(sequenceLED[j]*2), pixels.Color(0, 255, 0));
      pixels.setPixelColor(sequenceLED[j]*2+1), pixels.Color(0, 255, 0));
      pixels.show();
      delay(blinkDelay);
      pixels.clear();
      pixels.show();
      delay(blinkDelay/2);
    }

    byte j = 0;
    while (timeEnd <= timeStart + deviceSessionTime.value()) {  // loops checking sensors until either time runs out or successful completion of sequence
      for (byte k=0; k<8; k++) { // check and store pressure readings
        pressureSensor(k);
      }
      for (byte k=0; k<8; k++) {
        if ((sequence[j] != k) && (pressureThreshold[k] > 0)) { // checks if incorrect sensor was hit
          pixels.setPixelColor(k*2, pixels.Color(255, 0, 0));     //
          pixels.setPixelColor((k*2)+1, pixels.Color(255, 0, 0)); //
          pixels.show();                                          // this block sets leds for incorrect button
          delay(blinkDelay/2);                                    // to flash red very quickly once
          pixels.clear();                                         //
          pixels.show();                                          //
          goto seqStart;  // goes back to generate new sequence due to incorrect hit
        }
      }
      if ((j<=i) && (pressureThreshold[sequence[j]] > 0)) {         // checks if correct sensor was hit
        pixels.setPixelColor(sequenceLED[j], pixels.Color(0, 255, 0));   //
        pixels.setPixelColor(sequenceLED[j]+1, pixels.Color(0, 255, 0)); //
        pixels.show();                                                   // this block sets leds for correct button
        delay(blinkDelay/2);                                             // to flash green very quickly once
        pixels.clear();                                                  //
        pixels.show();                                                   //
        j++;
      }
      else if (j>i) {
        break;
      }
      timeEnd = millis(); 
    }
    if (timeEnd >= timeStart + deviceSessionTime.value()) {
      break;
    }
  }
  pixels.clear();
  pixels.show();
}
void sequenceGenerator(byte j){
  for (byte i=0; i<=j; i++){
    sequence[i] = random(0, 8);
    sequenceLED[i] = sequence[i]*2;
  }
}

void pressureSensor(byte i) {
  analogReading[i] = analogRead(pin[i]);
  
  if (analogReading[i] < 100) {        // from 0 to 49
    pressureThreshold[i] = 0;
  }
  else if (analogReading[i] < 2000) {  // from 50 to 1999
    pressureThreshold[i] = 1;
    forceToBluetooth(i);
  }
  else {                               // from 1999 to 4095
    pressureThreshold[i] = 2;
    forceToBluetooth(i);
  }
}

void forceToBluetooth(byte i) {
  if (analogReadingAVG[i] == 0) {
    analogReadingAVG[i] = analogReading[i];
  }
  else {
    analogReadingAVG[i] = (analogReading[i] + analogReadingAVG[i])/2;
  }
  if (analogReadingMin[i] == 0) {
    analogReadingMin[i] = analogReading[i];
  }
  else if (analogReadingMin[i] > analogReading[i]) {
    analogReadingMin[i] = analogReading[i];
  }
  if (analogReadingMax[i] == 0) {
    analogReadingMax[i] = analogReading[i];
  }
  else if (analogReadingMax[i] < analogReading[i]) {
    analogReadingMax[i] = analogReading[i];
  }
}

void compressArrays() {
  byte amount = 0;
  byte minimum = 0;
  for (byte i=0; i<8; i++) {
    if (analogReadingAVG[i] != 0) {
      analogReadingAVG[8] += analogReadingAVG[i];
      amount++;
    }
    analogReadingAVG[8] = analogReadingAVG[8] / amount;
    
    if (analogReadingMin[i] != 0) {
      if (minimum == 0) {
        analogReadingMin[8] = analogReadingMin[i];
        minimum++;
      }
      analogReadingMin[8] = min(analogReadingMin[i], analogReadingMin[8]);
    }
    if (analogReadingMax[i] != 0) {
      analogReadingMax[8] = max(analogReadingMax[i], analogReadingMax[8]);
    }
  }
}

void reset() {
  pixels.clear();
  pixels.show();
  timeEnd = 0;
  for (byte i=0; i<9; i++) {
    analogReading[i] = 0;
    analogReadingAVG[i] = 0;
    analogReadingMin[i] = 0;
    analogReadingMax[i] = 0;
  }
}

void rainbow(int wait) {
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    pixels.rainbow(firstPixelHue);
    pixels.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}
