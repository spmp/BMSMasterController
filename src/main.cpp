/** BMS Master Module
 * This code is the firmware for the BMS master module which performs
 * the following operations
 * - Communicates with Cell Top Modules (CTM's) on Serial 1
 * - Communicates with external charger on Serial 2
 * - Provides debug output on Serial0
 * - Displays cell information on an Adafruit_GFX display
 *
 * The hardware at this time uses the following resources
 * - Serail0 - default pins
 * - Serial1
 *   - RX Pin 33
 *   - TX Pin 32
 * - Serial2
 *   - RX Pin 27
 *   - TX Pin 26
 * - I2C (Wire) (1)
 *   - SDA Pin 0
 *   - SCL Pin 4
 *
 **/

#include <Arduino.h>
#include <CellComms.h>
#include <FEC.h>
#include <BMSCommsSender.h>
#include <CellCommsDisplay.h>
#include <Adafruit_SSD1306.h>


/** Cell Comms - communication with the Cell Top Modules **/
#define NUM_CELLS                     28

/** I2C port used for Adafruit display **/
#define I2C_DISPLAY_SDA_PIN           0
#define I2C_DISPLAY_SCL_PIN           4
#define I2C_DISPLAY_FREQUENCY         100000

/** Display - Adafruid_GFX display **/
#define ADAFRUIT_SSD1306_ADDRESS      0x3C
#define ADAFRUID_SSD1306_DEVICE_ID    SSD1306_128_64_ID
#define OLED_RESET                    4

/** CellCommsDisplay settings **/
#define CELLCOMMSDISPLAY_HEADER_PROP 0.25
#define CELLCOMMSDISPLAY_MV_MAX      3700
#define CELLCOMMSDISPLAY_MV_MIN      2700

/** Serial 0 - debug serial **/
#define SERIAL0_BAUD                  38400

/** Serial 1 - Cell Top Module communitcations **/
#define SERIAL1_TXPIN                 32
#define SERIAL1_RXPIN                 33
#define SERIAL1_BAUD                  4800

/** Serial 2 - inter micro BMS comms output **/
#define SERIAL2_TXPIN                 26
#define SERIAL2_RXPIN                 27

/** Instantiate Serials **/
HardwareSerial Serial1(1);
HardwareSerial Serial2(2);

/** Instantiate CellComms, BMSCommsSender, Adafruit_SSD1306, CellCommsDisplay **/
CellComms cells(
  NUM_CELLS,
  Serial2
);
BMSCommsSender bmsCommsSender(
  Serial1,
  cells
);
Adafruit_SSD1306 display(
  SSD1306_128_64_ID,
  OLED_RESET
);
CellCommsDisplay ccDisplay(
  display,
  cells,
  CELLCOMMSDISPLAY_HEADER_PROP,
  CELLCOMMSDISPLAY_MV_MIN,
  CELLCOMMSDISPLAY_MV_MAX
);

/** Timer and process control **/
uint32_t timestamp = 0;
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  timestamp++;
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

/** debug functions **/
void debugPrint(String debugMessage) {
  #ifdef debug
    Serial.println(debugMessage);
  #endif
}

/** Local variables and functions **/
uint8_t toggleReadWrite = 0;

uint8_t checkDisplayAlive(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission();
}

/***************************Setup**********************************************/
void setup()   {
  // Initialise Serials
  Serial.begin(SERIAL0_BAUD);
  debugPrint("Setup Serial0");

  Serial1.begin(SERIAL1_BAUD, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  debugPrint("Setup Serial1");

  Serial2.begin(CELLCOMMS_BAUD, SERIAL_8N1, SERIAL2_RXPIN, SERIAL2_TXPIN);
  debugPrint("Setup Serial2");

  // Initialise Wire (I2C)
  Wire.begin(I2C_DISPLAY_SDA_PIN, I2C_DISPLAY_SCL_PIN, I2C_DISPLAY_FREQUENCY);
  debugPrint("Setup Wire");
  delay(40);

  // Initilise display
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, ADAFRUIT_SSD1306_ADDRESS);  // initialize with the I2C addr 0x3D (for the 128x64)

  display.clearDisplay();
  debugPrint("Setup Display");

  // Setup Timer 1 on interrupt
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 1000000, true);
  // Start an alarm
  timerAlarmEnable(timer);
  debugPrint("Setup timer");
  delay(500);
}

/***************************Loop***********************************************/
void loop() {
  // If Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    // Do some stuff!
    toggleReadWrite++;
    // Send some data, then recieve it
    if (toggleReadWrite == 1) {

      // Send to cells to initiate response which can be read later
      cells.sendMillivolts(3550);
    }

    if (toggleReadWrite == 3) {
      // Read data
      cells.readCells();

      String printLine =
        String(timestamp)              +
        String(" Vmean: ")             + String(cells.millivoltsMean()) +
        String(" Vmax: ")              + String(cells.millivoltsMax()) +
        String(" Vmin: ")              + String(cells.millivoltsMin()) +
        String(" cell min v: ")        + String(cells.millivoltsMinCell()) +
        String(" cell max v: ")        + String(cells.millivoltsMaxCell()) +
        String(" balancing: ")         + String(cells.balancingNum()) +
        String(" over voltage: ")      + String(cells.overVoltageNum()) +
        String(" under voltage: ")     + String(cells.underVoltageNum()) +
        String(" temperature: ")       + String(cells.temperatureMean() * 0.1, 1) +
        String(" over temperature: ")  + String(cells.overTemperatureNum());

      Serial.print(printLine);
      Serial.print("\r\n");
      for (CellData cd : cells.cellDataVect) {
        Serial.print(cd.millivolts);
        Serial.print(" ");
      }

      // Send message to charger
      bmsCommsSender.send_packet();

      // Update display
      display.clearDisplay();
      ccDisplay.displayBars();
      ccDisplay.displayHeader();

      // Check whether I2C display is responding
      if ( checkDisplayAlive(SSD1306_I2C_ADDRESS) != 0 ) {
        // Reset the esp32 to 'repair' I2C
        esp_restart();
      }
      display.display();

      Serial.print("\r\n");

      toggleReadWrite = 0;
    }
  }
}
