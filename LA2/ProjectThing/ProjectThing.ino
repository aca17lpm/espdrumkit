// ProjectThing.ino

// Lucian Murdin LA2 Submission

// MIDI routines used from example "feather_midi" provided in Adafruit VS1053b library 
// https://github.com/adafruit/Adafruit_VS1053_Library/blob/master/examples/feather_midi/feather_midi.ino

// include main modules, also touch_pad stuff
#include <Arduino.h>
#include <Wire.h>
#include <esp_log.h>
#include "driver/touch_pad.h"

// the wifi and HTTP server libraries
#include <WiFi.h>
#include <WebServer.h>

// the timer stuff
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

// debugging infrastructure; setting different DBGs true triggers prints ////
#define dbg(b, s) if(b) Serial.print(s)
#define dln(b, s) if(b) Serial.println(s)
#define startupDBG      true
#define loopDBG         true
#define monitorDBG      true
#define netDBG          true
#define miscDBG         true
#define analogDBG       true
#define otaDBG          true

// wait_secs macro from exercises
#define WAIT_SECS(n) vTaskDelay((n*1000)/portTICK_PERIOD_MS); // n seconds

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf pg 31
// the instrument banks 
#define VS1053_BANK_DEFAULT 0x00
#define VS1053_BANK_DRUMS1 0x78
#define VS1053_BANK_DRUMS2 0x7F
#define VS1053_BANK_MELODY 0x79

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf pg 32
// the melody instruments from VS1053b
#define VS1053_GM1_OCARINA 80
#define VS1053_GM1_TUBBELLS 15
#define VS1053_GM1_SHANAI 112
#define VS1053_GM1_POLYSYNTH 91
#define VS1053_GM1_GOBLINS 102

// the drums from VS1053b
#define VS1053_GM1_OPENTRIANGLE 81
#define VS1053_GM1_CLOSEDTRIANGLE 80
#define VS1053_GM1_SNARE 38
#define VS1053_GM1_HIHAT 42
#define VS1053_GM1_SHORTWHISTLE 71
#define VS1053_GM1_LONGWHISTLE 72

#define MIDI_NOTE_ON  0x90
#define MIDI_NOTE_OFF 0x80
#define MIDI_CHAN_MSG 0xB0
#define MIDI_CHAN_BANK 0x00
#define MIDI_CHAN_VOLUME 0x07
#define MIDI_CHAN_PROGRAM 0xC0

#define VS1053_MIDI Serial1

// creating an array of drum kits
int kitArray[3][2] = {{VS1053_GM1_SNARE, VS1053_GM1_HIHAT},
                      {VS1053_GM1_SHORTWHISTLE, VS1053_GM1_LONGWHISTLE},
                      {VS1053_GM1_OPENTRIANGLE, VS1053_GM1_CLOSEDTRIANGLE}};
int kitNum = 0;

// creating an array of synth instruments
int synthArray[4] = {VS1053_GM1_OCARINA, VS1053_GM1_TUBBELLS, VS1053_GM1_POLYSYNTH, VS1053_GM1_GOBLINS};
int synthNum = 0;

// I/O definitions
uint8_t synthLED = 16;
uint8_t switchPin = 21;
static void eventsSetup();

// touch pad definitions
// API documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/touch_pad.html
touch_pad_t melodyPad = TOUCH_PAD_NUM0;

//sensor control for slope and voltage
touch_cnt_slope_t maxSlope = TOUCH_PAD_SLOPE_MAX;
touch_cnt_slope_t minSlope = TOUCH_PAD_SLOPE_1;
touch_tie_opt_t voltage = TOUCH_PAD_TIE_OPT_LOW;

// ints for filter, wire reading, and interrupt for touch config
uint32_t filterVal = 100;
uint16_t melodyVal = 0;
uint16_t interrupt = 0;

// synth stuff
// sensor range for touched and untouched melodyPad
const int sensorStart = 70;
const int sensorEnd = 400;
// MIDI range for synth output : chosen in the middle of the MIDI range 0-127 to avoid horrible pitches
const int MIDIStart = 40;
const int MIDIEnd = 80;

// calculating slope for translating sensor data into a MIDI range to be output from the synth
// maths used from https://stackoverflow.com/questions/5731863/mapping-a-numeric-range-onto-another
const double slope = 1.0 * (MIDIEnd - MIDIStart) / (sensorEnd - sensorStart);

int MIDIOutput = 0;
int lastMIDINote = 0;
bool synthOn = false;


// drum stuff
bool drumHit1 = false;
bool drumHit2 = false;
bool drumHitPrevious1 = false;
bool drumHitPrevious2 = false;
touch_pad_t drumPad1 = TOUCH_PAD_NUM7;
touch_pad_t drumPad2 = TOUCH_PAD_NUM5;
uint16_t drumVal1 = 0;
uint16_t drumVal2 = 0;

// globals for a wifi access point and webserver /////////////////////////////
String apSSID;                  // SSID of the AP
WebServer webServer(80);        // a simple web server

// timer thing from https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Timer/RepeatTimer/RepeatTimer.ino
void ARDUINO_ISR_ATTR onTimer(){
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void setup() {
  delay(1000);
  
  Serial.begin(115200);
  VS1053_MIDI.begin(31250); // MIDI uses a 'strange baud rate'

  startAP();            // fire up the AP...
  startWebServer();     // ...and the web server

  pinMode(synthLED, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);

  // interrupts setup
  eventsSetup();

  // setting channel bank for synth on channel 0
  midiSetChannelBank(0, VS1053_BANK_MELODY);
  midiSetChannelVolume(0, 127);
  midiSetInstrument(0, VS1053_GM1_OCARINA);

  // setting channel bank for drums on channel 9 (drums dedicated channel)
  // no need to set instrument as it is determined by pitch for drums
  midiSetChannelBank(9, VS1053_BANK_DRUMS1);
  midiSetChannelVolume(9, 127);

  // configuring touch pad stuff, not using interrupts so set to zero
  touch_pad_init();
  touch_pad_config(melodyPad, interrupt);
  touch_pad_config(drumPad1, interrupt);
  touch_pad_config(drumPad2, interrupt);

  // set pad charge/discharge rates with fast for drumpads and slow for synth
  touch_pad_set_cnt_mode(melodyPad, maxSlope, voltage);
  touch_pad_set_cnt_mode(drumPad1, minSlope, voltage);
  touch_pad_set_cnt_mode(drumPad2, minSlope, voltage);
  touch_pad_filter_start(filterVal);


  // more timer stuff from IDF example
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  // Set alarm to call onTimer function every 1/16th second (value in microseconds).
  timerAlarmWrite(timer, 62500, true);
  // Start an alarm
  timerAlarmEnable(timer);
}

void loop() {  


  webServer.handleClient(); // deal with any pending web requests

  // read in touch pads 
  touch_pad_read_filtered(melodyPad, &melodyVal);
  touch_pad_read_raw_data(drumPad1, &drumVal1);
  touch_pad_read_raw_data(drumPad2, &drumVal2);

  //drum 1 hit once
  if (drumVal1 < 30){
    drumHit1 = true;
  } else {
    drumHit1 = false;
    drumHitPrevious1 = false;
  }

  //drum 2 hit once
  if (drumVal2 < 30){
    drumHit2 = true;
  } else {
    drumHit2 = false;
    drumHitPrevious2 = false;
  }

  // monitoring timer every 1/16th of a second, when notes are detected, play all at once
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    
    // read in filtered synth pad data and print to plotter
    // Serial.printf("%d\n", melodyVal);
    
    // turn off previous synth note, no need to turn off drums
    midiNoteOff(0, lastMIDINote, 0);

    // check drum hit but not a double hit
    if (drumHit1 && !drumHitPrevious1){
      // find hit array, instrument 1
      midiNoteOn(9, kitArray[kitNum][0], 127);
      drumHitPrevious1 = true;
    }

    if (drumHit2 && !drumHitPrevious2) {
      midiNoteOn(9, kitArray[kitNum][1], 127);
      drumHitPrevious2 = true;
    }

    // check if synth is turned on by button : see interrupt handler
    if (synthOn){
    
      // set instrument selected
      midiSetInstrument(0, synthArray[synthNum]);
      
      // map sensor input to MIDI output
      MIDIOutput = MIDIStart + slope * (melodyVal - sensorStart);
      midiNoteOn(0, MIDIOutput, 127);
      
      // remember MIDI note to turn off in next timer callback
      lastMIDINote = MIDIOutput;
    }
  }
}

// using gpio interrupt events from TimingThing
static void IRAM_ATTR gpio_isr_handler(void *arg) { // switch press handler
  uint32_t gpio_num = (uint32_t) arg;
  // toggle LED and synthOn variable
  if (synthOn) {
    synthOn = false;
    digitalWrite(synthLED, LOW);
  } else {
    synthOn = true;
    digitalWrite(synthLED, HIGH);
  }
}

static void eventsSetup() {
  // configure the switch pin (INPUT, falling edge interrupts)
  gpio_config_t io_conf;                        // params for switches
  io_conf.mode = GPIO_MODE_INPUT;               // set as input mode
  io_conf.pin_bit_mask = 1ULL << switchPin;    // bit mask of pin(s) to set
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;      // enable pull-up mode
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // disable pull-down mode
  io_conf.intr_type = GPIO_INTR_NEGEDGE;        // interrupt on falling edge
  (void) gpio_config(&io_conf);                 // do the configuration

  // install gpio isr service & hook up isr handlers
  gpio_install_isr_service(0); // prints an error if already there; ignore!
  gpio_isr_handler_add(        // attach the handler
    (gpio_num_t) switchPin, gpio_isr_handler, (void *) switchPin
  );
}

// routine to change channel, instrument and bank together
void changeInstrument(uint8_t chan,uint8_t inst, uint8_t bank) {
  midiSetChannelBank(chan, bank);
  midiSetChannelVolume(chan, 127);
  midiSetInstrument(chan, inst);
}

// MIDI utilities from feather_midi example https://github.com/adafruit/Adafruit_VS1053_Library/blob/master/examples/feather_midi/feather_midi.ino

// functions construct MIDI messages to send along to synth
void midiSetInstrument(uint8_t chan, uint8_t inst) {
  if (chan > 15) return;
  inst --; // page 32 has instruments starting with 1 not 0 :(
  if (inst > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_PROGRAM | chan);  
  delay(10);
  VS1053_MIDI.write(inst);
  delay(10);
}

void midiSetChannelVolume(uint8_t chan, uint8_t vol) {
  if (chan > 15) return;
  if (vol > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_MSG | chan);
  VS1053_MIDI.write(MIDI_CHAN_VOLUME);
  VS1053_MIDI.write(vol);
}

void midiSetChannelBank(uint8_t chan, uint8_t bank) {
  if (chan > 15) return;
  if (bank > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_MSG | chan);
  VS1053_MIDI.write((uint8_t)MIDI_CHAN_BANK);
  VS1053_MIDI.write(bank);
}

void midiNoteOn(uint8_t chan, uint8_t n, uint8_t vel) {
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  
  VS1053_MIDI.write(MIDI_NOTE_ON | chan);
  VS1053_MIDI.write(n);
  VS1053_MIDI.write(vel);
}

void midiNoteOff(uint8_t chan, uint8_t n, uint8_t vel) {
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  
  VS1053_MIDI.write(MIDI_NOTE_OFF | chan);
  VS1053_MIDI.write(n);
  VS1053_MIDI.write(vel);
}


// webserver stuff from EX06
// web AP startup utilities
void startAP() {
  apSSID = String("Lucian-ESP32");

  if(! WiFi.mode(WIFI_AP_STA))
    Serial.println("failed to set Wifi mode");
  if(! WiFi.softAP(apSSID.c_str(), "dumbpassword"))
    Serial.println("failed to start soft AP");
  printIPs();
}

void printIPs() {
  if(startupDBG) { // easier than the debug macros for multiple lines etc.
    Serial.print("AP SSID: ");
    Serial.print(apSSID);
    Serial.print("; IP address(es): local=");
    Serial.print(WiFi.localIP());
    Serial.print("; AP=");
    Serial.println(WiFi.softAPIP());
  }
  if(netDBG)
    WiFi.printDiag(Serial);
}
void startWebServer() {
  // register callbacks to handle selecting kits and synths
  webServer.on("/", handleRoot);
  webServer.on("/kit1", getKit1);
  webServer.on("/kit2", getKit2);
  webServer.on("/kit3", getKit3);

  webServer.on("/synth1", getSynth1);
  webServer.on("/synth2", getSynth2);
  webServer.on("/synth3", getSynth3);

  webServer.on("/goblins", getGoblins);

  // 404s...
  webServer.onNotFound(handleNotFound);

  webServer.begin();
}

// HTML page creation utilities
String getPageTop() {
  return
    "<html><head><title>COM3506 IoT [ID: " + apSSID + "]</title>\n"
    "<meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
    "\n<style>body{background:#FFF; color: #000; "
    "font-family: sans-serif; font-size: 150%;}</style>\n"
    "</head><body>\n"
  ;
};

String getPageBody() {
  return 
     "<h2>The Mouldy Banana drum kit and synth</h2>\n"
     "\n<p><a href='/kit1'>Kit1</a>&nbsp;&nbsp;&nbsp;</p></body></html>\n"
     "\n<p><a href='/kit2'>Whistle Kit</a>&nbsp;&nbsp;&nbsp;</p></body></html>\n"
     "\n<p><a href='/kit3'>Triangle Kit</a>&nbsp;&nbsp;&nbsp;</p></body></html>\n"
     "\n<p><a href='/synth1'>Ocarina Synth</a>&nbsp;&nbsp;&nbsp;</p></body></html>\n"
     "\n<p><a href='/synth2'>Tubular Bells Synth</a>&nbsp;&nbsp;&nbsp;</p></body></html>\n"
     "\n<p><a href='/synth3'>Synth3</a>&nbsp;&nbsp;&nbsp;</p></body></html>\n"
     "\n<p><a href='/goblins'>Goblins!</a>&nbsp;&nbsp;&nbsp;</p></body></html>\n"
   ;
};

String getPageFooter() {
  return "\n<p><a href='/'>Home</a>&nbsp;&nbsp;&nbsp;</p></body></html>\n";
}

// webserver handler callbacks ///////////////////////////////////////////////
void handleNotFound() {
  webServer.send(200, "text/plain", "URI Not Found");
}

void handleRoot() {
  String toSend = getPageTop();
  toSend += getPageBody();
  toSend += getPageFooter();
  webServer.send(200, "text/html", toSend);
}

void getKit1() {
  kitNum = 0;
  handleRoot();
}

void getKit2() {
  kitNum = 1;
  handleRoot();
}

void getKit3() {
  kitNum = 2;
  handleRoot();
}

void getSynth1() {
  synthNum = 0;
  handleRoot();
}

void getSynth2() {
  synthNum = 1;
  handleRoot();
}

void getSynth3() {
  synthNum = 2;
  handleRoot();
}

void getGoblins() {
  synthNum = 3;
  handleRoot();
}
