/** BMS Master Module
 * This code is the firmware for the BMS master module which performs
 * the following operations
 * - Communicates with Cell Top Modules (CTM's) on Serial 1
 * - Communicates with external charger on Serial 2
 * - Provides debug output on Serial0
 * - Displays cell information on an Adafruit_GFX display
 * - Logs status to serial and bluetooth serial
 *
 * The hardware at this time uses the following resources
 * - Serail0 - default pins
 * - BMSCommsSenderSerial
 *   - RX Pin 33
 *   - TX Pin 32
 * - CellCommsSerial
 *   - RX Pin 27
 *   - TX Pin 26
 * - I2C (Wire) (1)
 *   - SDA Pin 0
 *   - SCL Pin 4
 * - BT Classic RFCOMM
 *
 **/

#include <Arduino.h>
#include <CellComms.h>
#include <FEC.h>
#include <BMSCommsSender.h>
#include <CellCommsDisplay.h>
#include <Adafruit_SSD1306.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define debug

/** Cell Comms - communication with the Cell Top Modules **/
#define NUM_CELLS                     28
#define CELL_COMMS_SERIAL_BAUD        19200
#define CELL_COMMS_SERIAL_DEV         1
#define CELL_COMMS_SERIAL_TXPIN       26
#define CELL_COMMS_SERIAL_RXPIN       27

/** BMS Comms Sender - inter micro BMS comms output **/
#define BMS_COMMS_SENDER_SERIAL_BAUD  4800
#define BMS_COMMS_SENDER_SERIAL_DEV   2
#define BMS_COMMS_SENDER_SERIAL_TXPIN 32
#define BMS_COMMS_SENDER_SERIAL_RXPIN 33

/** I2C port used for Adafruit display **/
#define I2C_DISPLAY_SDA_PIN           0
#define I2C_DISPLAY_SCL_PIN           4
#define I2C_DISPLAY_FREQUENCY         100000

/** Display - Adafruid_GFX display **/
#define ADAFRUIT_SSD1306_ADDRESS      0x3C
#define ADAFRUID_SSD1306_DEVICE_ID    SSD1306_128_64_ID
#define OLED_RESET                    I2C_DISPLAY_SCL_PIN

/** CellCommsDisplay settings **/
#define CELLCOMMSDISPLAY_HEADER_PROP 0.25
#define CELLCOMMSDISPLAY_MV_MAX      3700
#define CELLCOMMSDISPLAY_MV_MIN      2700

/** Serial 0 - debug serial **/
#define SERIAL0_BAUD                  38400

/** BT Serial device **/
#define BT_SERIAL_NAME                 "CTM BMS"

/** Instantiate Serials **/
HardwareSerial CellCommsSerial(CELL_COMMS_SERIAL_DEV);
HardwareSerial BMSCommsSenderSerial(BMS_COMMS_SENDER_SERIAL_DEV);
BluetoothSerial SerialBT;

/** Instantiate CellComms, BMSCommsSender, Adafruit_SSD1306, CellCommsDisplay **/
CellComms cells(
  NUM_CELLS,
  CellCommsSerial
);
BMSCommsSender bmsCommsSender(
  BMSCommsSenderSerial,
  cells
);
Adafruit_SSD1306 display(
  Wire,
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

  CellCommsSerial.begin(
    CELL_COMMS_SERIAL_BAUD,
    SERIAL_8N1,
    CELL_COMMS_SERIAL_RXPIN,
    CELL_COMMS_SERIAL_TXPIN
  );
  debugPrint("Setup CellCommsSerial");

  BMSCommsSenderSerial.begin(
    BMS_COMMS_SENDER_SERIAL_BAUD,
    SERIAL_8N1,
    BMS_COMMS_SENDER_SERIAL_RXPIN,
    BMS_COMMS_SENDER_SERIAL_TXPIN
  );
  debugPrint("Setup BMSCommsSenderSerial");

  SerialBT.begin(BT_SERIAL_NAME);
  debugPrint("Setup SerialBT");

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

      String bankStatus =
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
        String(" over temperature: ")  + String(cells.overTemperatureNum()) +
        String("\r\n");
      Serial.print(bankStatus);
      SerialBT.print(bankStatus);

      // Add cell mv per cell
      // NOTE: BT Serial needs some time between sending packets
      // The following for-loop is enough time.
      String cellVs = String("");
      for (CellData cd : cells.cellDataVect) {
        cellVs += String(cd.millivolts);
        cellVs += String(" ");
      }
      cellVs += String("\r\n");
      Serial.print(cellVs);
      SerialBT.print(cellVs);

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

      toggleReadWrite = 0;
    }
  }
}
