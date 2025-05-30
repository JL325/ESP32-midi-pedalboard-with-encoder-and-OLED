// Standard Arduino and Adafruit Libraries
#include <Wire.h>             // For I2C communication (used by OLED)
#include <Adafruit_GFX.h>     // Core graphics library for OLED
#include <Adafruit_SSD1306.h> // Hardware-specific library for SSD1306 OLED

// ESP32 USB MIDI Libraries
#include "USB.h"     // ESP32 USB core library
#include "USBMIDI.h" // ESP32 USB MIDI library

// Create a MIDI object
USBMIDI MIDI;

// --- OLED Display Settings ---
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin, common for ESP32)
#define SCREEN_ADDRESS 0x3C // I2C address for the SSD1306 display (can also be 0x3D)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // OLED display object
unsigned long lastOledUpdateTime = 0;       // Timestamp of the last OLED screen update
const unsigned long OLED_UPDATE_INTERVAL = 100; // Update OLED every 100ms (if changes occurred)
bool oledNeedsUpdate = true;                // Flag to indicate if the OLED needs to be redrawn

// --- Rotary Encoder Pin Definitions ---
#define ENCODER_PIN_A  13 // CLK (Clock) pin of the rotary encoder
#define ENCODER_PIN_B  14 // DT (Data) pin of the rotary encoder
#define ENCODER_PIN_SW 12 // SW (Switch) pin of the rotary encoder (button press)

// --- Rotary Encoder State Variables ---
int encoderPos = 0;             // Current raw position of the encoder
int lastReportedPos = 0;        // Last encoder position that resulted in a BPM/Preset change
byte encoderPinALast = LOW;     // Previous state of encoder pin A (for detecting changes)
byte encoderPinAState = LOW;    // Current state of encoder pin A
byte encoderPinBState = LOW;    // Current state of encoder pin B
bool encoderRotated = false;    // Flag indicating if the encoder has been rotated

// --- Mode Management ---
enum Mode {
  BPM_MODE,     // Mode for adjusting BPM and tap tempo
  PRESET_MODE   // Mode for selecting and sending MIDI Program Changes
};
Mode currentMode = BPM_MODE; // Default mode is BPM mode
int currentPreset = 0;       // Current MIDI program change number (0-127)

// --- Encoder Switch Debouncing Variables ---
const int ENCODER_DEBOUNCE_TIME = 50;   // Debounce time for the encoder switch (milliseconds)
bool lastEncoderSwitchState = HIGH;     // Previous state of the encoder switch (HIGH because of INPUT_PULLUP)
unsigned long lastEncoderSwitchDebounceTime = 0; // Timestamp of the last encoder switch state change

// --- Encoder Acceleration Settings (for BPM adjustment) ---
unsigned long lastEncoderBpmUpdateTime = 0;   // Timestamp of the last BPM update via encoder
const unsigned long FAST_THRESHOLD_MS = 60;   // Time (ms) below which rotation is considered FAST
const unsigned long MEDIUM_THRESHOLD_MS = 120;  // Time (ms) below which rotation is considered MEDIUM
const int ACCEL_FAST_MULTIPLIER = 5;          // BPM change multiplier for FAST rotation
const int ACCEL_MEDIUM_MULTIPLIER = 2;        // BPM change multiplier for MEDIUM rotation
const int ACCEL_NORMAL_MULTIPLIER = 1;        // BPM change multiplier for normal rotation

// --- Button Settings ---
#define NUM_BUTTONS 21 // Total number of buttons
// Pin mapping for the buttons. Order matters for functionality.
const int buttonPins[NUM_BUTTONS] = {
    0,  // Index 0: Tap Tempo button
    18, // Index 1: Trigger Button 1 (CC#69, Value 0)
    33, // Index 2: Trigger Button 2 (CC#69, Value 1)
    35, // Index 3: Trigger Button 3 (CC#69, Value 2)
    37, // Index 4: Trigger Button 4 (CC#69, Value 3)
    39, // Index 5: Trigger Button 5 (CC#69, Value 4)
    8,  // Index 6: Trigger Button 6 (CC#69, Value 5)
    6,  // Index 7: Trigger Button 7 (CC#69, Value 6)
    4,  // Index 8: Trigger Button 8 (CC#69, Value 7)
    2,  // Index 9: Button (CC#1, Value 1)
    1,  // Index 10: Button (CC#2, Value 2)
    40, // Index 11: Toggle Button 1 (CC#4)
    38, // Index 12: Toggle Button 2 (CC#5)
    36, // Index 13: Toggle Button 3 (CC#6)
    34, // Index 14: Toggle Button 4 (CC#7)
    21, // Index 15: Toggle Button 5 (CC#8)
    3,  // Index 16: Toggle Button 6 (CC#9)
    5,  // Index 17: Toggle Button 7 (CC#10)
    7,  // Index 18: Toggle Button 8 (CC#11)
    9,  // Index 19: Button (CC#12, Value 0)
    11  // Index 20: Mode Switch Button (BPM/Preset)
};
const int DEBOUNCE_TIME = 50;                 // Debounce time for buttons (milliseconds)
bool lastButtonStates[NUM_BUTTONS];           // Array to store the last state of each button
unsigned long lastDebounceTimes[NUM_BUTTONS]; // Array to store the last debounce time for each button
bool toggleStates[NUM_BUTTONS];               // Array to store the toggle state (ON/OFF) for toggleable buttons

// --- Tempo Settings ---
int bpm = 120;                // Default Beats Per Minute
const int MIN_BPM = 30;       // Minimum allowed BPM
const int MAX_BPM = 250;      // Maximum allowed BPM
unsigned long lastTapTime = 0; // Timestamp of the last tap for tap tempo
bool tapReady = false;         // Flag indicating if the first tap has occurred (ready for second tap)

// --- LED Settings ---
const int ledPin = 15;            // Pin connected to the tempo LED
unsigned long lastBeatTime = 0;   // Timestamp of the last LED beat flash
bool ledState = false;            // Current state of the LED (ON/OFF)
bool controlLedViaMidi = false;   // Flag indicating if the LED is being controlled by an incoming MIDI CC message

// --- MIDI Input Settings ---
const uint8_t MIDI_CHANNEL_IN = 1;    // MIDI channel to listen for incoming messages (1-16)
const uint8_t CC_LED_CONTROL = 20;    // MIDI CC number to control the LED state

// --- Function Prototypes ---
void handleMidiMessage(midiEventPacket_t rx_packet);
void updateOLED();
void readEncoder();
void sendSimulatedTapTempo();

//=============================================================================
// SETUP FUNCTION - Runs once at startup
//=============================================================================
void setup() {
  Serial.begin(115200); // Initialize serial communication for debugging
  Serial.println("Starting MIDI Controller with OLED and Encoder...");

  // --- Initialize OLED Display ---
  Wire.begin(17, 16); // Initialize I2C communication (SDA=17, SCL=16 for ESP32)
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Halt execution if OLED initialization fails
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("MIDI Controller"); // Splash screen message
  display.display();
  delay(1000); // Pause for splash screen visibility
  oledNeedsUpdate = true; // Force initial BPM display after splash screen

  // --- Initialize Buttons ---
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP); // Set button pins as input with internal pull-up resistors
    lastButtonStates[i] = HIGH;           // Initialize last button state (HIGH due to pull-up)
    lastDebounceTimes[i] = 0;             // Initialize debounce timer
    toggleStates[i] = false;              // Initialize toggle states to OFF
  }

  // --- Initialize Rotary Encoder Pins ---
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);  // Encoder CLK pin
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);  // Encoder DT pin
  pinMode(ENCODER_PIN_SW, INPUT_PULLUP); // Encoder Switch pin
  encoderPinALast = digitalRead(ENCODER_PIN_A); // Read initial state of encoder pin A

  // --- Initialize LED ---
  pinMode(ledPin, OUTPUT);      // Set LED pin as output
  digitalWrite(ledPin, LOW);    // Turn LED off initially

  // --- Initialize USB MIDI ---
  MIDI.begin(); // Initialize USB MIDI communication
  USB.begin();  // Initialize USB stack
  Serial.println("MIDI Controller Ready.");
}

//=============================================================================
// MAIN LOOP - Runs repeatedly
//=============================================================================
void loop() {
  unsigned long currentTime = millis();     // Get current time once at the start of the loop for consistency
  bool triggerButtonPressed = false;        // Flag to track if any of trigger buttons (1-8) were pressed in this loop iteration

  // --- 1. Read Rotary Encoder ---
  readEncoder(); // Updates encoderPos and sets encoderRotated flag if rotation occurred

  // --- 2. Process Encoder Rotation ---
  if (encoderRotated) {
    int currentEncoderPos = encoderPos;
    int change = currentEncoderPos - lastReportedPos; // Calculate raw change in encoder position

    // Encoder typically requires 2 steps for a "click"
    if (abs(change) >= 2) {
      int effectiveChangeBase = change / 2; // Number of "clicks"

      if (currentMode == BPM_MODE) {
        // --- BPM Mode: Adjust BPM with acceleration ---
        unsigned long timeSinceLastUpdate = currentTime - lastEncoderBpmUpdateTime;
        int accelerationMultiplier = ACCEL_NORMAL_MULTIPLIER;

        if (lastEncoderBpmUpdateTime != 0) { // Avoid acceleration on the very first turn
          if (timeSinceLastUpdate < FAST_THRESHOLD_MS) {
            accelerationMultiplier = ACCEL_FAST_MULTIPLIER;
          } else if (timeSinceLastUpdate < MEDIUM_THRESHOLD_MS) {
            accelerationMultiplier = ACCEL_MEDIUM_MULTIPLIER;
          }
        }
        int finalEffectiveChange = effectiveChangeBase * accelerationMultiplier;
        bpm += finalEffectiveChange;

        // Constrain BPM to defined min/max values
        if (bpm < MIN_BPM) bpm = MIN_BPM;
        if (bpm > MAX_BPM) bpm = MAX_BPM;

        lastReportedPos += (effectiveChangeBase * 2); // Update based on raw steps consumed
        lastEncoderBpmUpdateTime = currentTime;       // Record time of this successful BPM update
        Serial.print("BPM Encoder - Raw: "); Serial.print(change);
        Serial.print(", Final: "); Serial.print(finalEffectiveChange);
        Serial.print(" -> BPM: "); Serial.println(bpm);
        oledNeedsUpdate = true;     // Mark OLED for update
        controlLedViaMidi = false;  // Manual BPM adjustment takes LED control priority
      } else if (currentMode == PRESET_MODE) {
        // --- Preset Mode: Adjust current preset number ---
        currentPreset += effectiveChangeBase;
        // Wrap around preset number (0-127)
        if (currentPreset < 0) currentPreset = 127;
        if (currentPreset > 127) currentPreset = 0;

        lastReportedPos += (effectiveChangeBase * 2); // Update based on raw steps consumed
        Serial.print("Preset Encoder -> Preset: "); Serial.println(currentPreset);
        oledNeedsUpdate = true; // Mark OLED for update
      }
    }
    encoderRotated = false; // Reset rotation flag
  }

  // --- 3. Read Encoder Switch (Button Press) ---
  bool currentEncoderSwitchState = digitalRead(ENCODER_PIN_SW);
  if (currentEncoderSwitchState != lastEncoderSwitchState) {
    lastEncoderSwitchDebounceTime = currentTime; // Reset debounce timer on state change
  }

  if ((currentTime - lastEncoderSwitchDebounceTime) > ENCODER_DEBOUNCE_TIME) {
    // If switch state has been stable longer than debounce time
    if (currentEncoderSwitchState == LOW && lastEncoderSwitchState == HIGH) { // Check for falling edge (press)
      Serial.println("Encoder button pressed!");
      if (currentMode == BPM_MODE) {
        sendSimulatedTapTempo(); // In BPM mode, encoder press simulates tap tempo
      } else if (currentMode == PRESET_MODE) {
        // In Preset mode, encoder press sends a MIDI Program Change
        MIDI.programChange(currentPreset, MIDI_CHANNEL_IN); // Send PC on specified channel
        Serial.print("Sent Program Change: "); Serial.println(currentPreset);
        oledNeedsUpdate = true; // Update OLED to reflect action (e.g., "Sent PC" or highlight preset)
      }
    }
  }
  lastEncoderSwitchState = currentEncoderSwitchState; // Save current switch state for next iteration

  // --- 4. Handle MIDI Input ---
  midiEventPacket_t rx_packet;
  if (MIDI.readPacket(&rx_packet)) { // Check if a MIDI packet has been received
      handleMidiMessage(rx_packet);  // Process the received MIDI message
  }

  // --- 5. Handle Button Inputs & Send MIDI Output ---
  for (int i = 0; i < NUM_BUTTONS; i++) {
    bool currentState = digitalRead(buttonPins[i]); // Read current state of the button
    if (currentState != lastButtonStates[i]) {
      // If button state changed, check debounce timer
      if (currentTime - lastDebounceTimes[i] > DEBOUNCE_TIME) {
        if (currentState == LOW) { // Button pressed (LOW due to INPUT_PULLUP)
          // --- Button 0: Tap Tempo ---
          if (i == 0) {
            MIDI.controlChange(3, 127, MIDI_CHANNEL_IN); // Send CC#3 (Tap Tempo Signal)
            Serial.println("Button 0 (Tap - Pin " + String(buttonPins[i]) + "): Sent MIDI CC#3 Value: 127");

            if (tapReady && (currentTime - lastTapTime) < 2000) { // Max 2 sec between taps
              unsigned long tapInterval = currentTime - lastTapTime;
              if (tapInterval >= 250) { // Min interval to avoid too fast taps (240 BPM)
                bpm = 60000 / tapInterval;
                if (bpm < MIN_BPM) bpm = MIN_BPM; // Constrain BPM
                if (bpm > MAX_BPM) bpm = MAX_BPM;
                Serial.print("TAP BPM: "); Serial.println(bpm);
                controlLedViaMidi = false; // Manual tap takes LED control
                oledNeedsUpdate = true;    // Update OLED with new BPM
              }
            } else { // First tap or tap too late
              tapReady = true;
              controlLedViaMidi = false;
              oledNeedsUpdate = true; // Update OLED on first tap (to show "TAP" or similar)
            }
            lastTapTime = currentTime;
          }
          // --- Buttons 1-8: Trigger Buttons (send CC#69 with value 0-7) ---
          else if (i >= 1 && i <= 8) {
            triggerButtonPressed = true; // Set flag to reset toggle buttons later
            MIDI.controlChange(69, i - 1, MIDI_CHANNEL_IN); // CC#69, Value: 0 for button index 1, up to 7 for button index 8
            Serial.print("Button " + String(i) + " (Pin " + String(buttonPins[i]) + "): Sent MIDI CC#69 Value: "); Serial.println(i - 1);
          }
          // --- Buttons 11-18: Toggle Buttons ---
          // These buttons send a CC message (4-11) with value 127 (ON) or 0 (OFF)
          else if (i >= 11 && i <= 18) {
            toggleStates[i] = !toggleStates[i]; // Flip the toggle state
            int ccNumber = i - 7; // Button 11 -> CC#4, Button 12 -> CC#5, ..., Button 18 -> CC#11
            MIDI.controlChange(ccNumber, toggleStates[i] ? 127 : 0, MIDI_CHANNEL_IN);
            Serial.print("Button " + String(i) + " (Pin " + String(buttonPins[i]) + "): Sent MIDI CC#" + String(ccNumber) + " Value: "); Serial.println(toggleStates[i] ? 127 : 0);
          }
          // --- Button 9: Send CC#1, Value 1 ---
          else if (i == 9)  {
            MIDI.controlChange(1, 1, MIDI_CHANNEL_IN);
            Serial.println("Button 9 (Pin " + String(buttonPins[i]) + "): Sent MIDI CC#1 Value: 1");
          }
          // --- Button 10: Send CC#2, Value 2 ---
          else if (i == 10) {
            MIDI.controlChange(2, 2, MIDI_CHANNEL_IN);
            Serial.println("Button 10 (Pin " + String(buttonPins[i]) + "): Sent MIDI CC#2 Value: 2");
          }
          // --- Button 19: Send CC#12, Value 0 ---
          else if (i == 19) {
            MIDI.controlChange(12, 0, MIDI_CHANNEL_IN);
            Serial.println("Button 19 (Pin " + String(buttonPins[i]) + "): Sent MIDI CC#12 Value: 0");
          }
          // --- Button 20: Mode Switch (BPM/Preset) ---
          else if (i == 20) {
            if (currentMode == BPM_MODE) {
              currentMode = PRESET_MODE;
              Serial.println("Mode changed to PRESET_MODE");
            } else {
              currentMode = BPM_MODE;
              Serial.println("Mode changed to BPM_MODE");
            }
            oledNeedsUpdate = true; // Mode change requires OLED update
          }
        } // End of if (currentState == LOW)
        lastDebounceTimes[i] = currentTime; // Update debounce time only on a debounced change
      } // End of if (debounce time passed)
    } // End of if (button state changed)
    lastButtonStates[i] = currentState; // Update button state regardless of debounce success for next comparison
  } // End of button loop

  // --- Reset Toggle Buttons (11-18) if a Trigger Button (1-8) was pressed ---
  if (triggerButtonPressed) {
    Serial.println("Trigger button (1-8) pressed. Checking toggle buttons 11-18...");
    for (int j = 11; j <= 18; j++) { // Iterate through toggle buttons
      if (toggleStates[j] == true) { // If this toggle button is currently ON
        toggleStates[j] = false;     // Set its internal state to OFF
        int ccNumber = j - 7;        // Calculate corresponding CC number (Button 11 -> CC#4, etc.)
        MIDI.controlChange(ccNumber, 0, MIDI_CHANNEL_IN); // Send MIDI CC to turn it OFF (value 0)
        Serial.print("  Button "); Serial.print(j);
        Serial.print(" (Pin "); Serial.print(buttonPins[j]);
        Serial.print(", CC#"); Serial.print(ccNumber);
        Serial.println("): Reset to OFF (Value 0) due to trigger button press.");
      }
    }
    // triggerButtonPressed will be reset to false at the start of the next loop iteration
  }

  // --- 6. Update OLED Display ---
  if (oledNeedsUpdate || (currentTime - lastOledUpdateTime > OLED_UPDATE_INTERVAL)) {
      updateOLED();                 // Redraw the OLED screen
      oledNeedsUpdate = false;      // Reset the update flag
      lastOledUpdateTime = currentTime; // Record the time of this update
  }

  // --- 7. Handle LED Blinking for Tempo ---
  if (!controlLedViaMidi && bpm > 0) { // If LED is not controlled by MIDI and BPM is valid
    unsigned long beatIntervalTime = 60000 / (unsigned long)bpm; // Calculate time per beat in ms
    if (beatIntervalTime > 0 && currentTime - lastBeatTime >= beatIntervalTime) {
      ledState = !ledState; // Toggle LED state
      digitalWrite(ledPin, ledState ? HIGH : LOW); // Update physical LED
      lastBeatTime = currentTime; // Record time of this beat flash
    }
  }
}

//=============================================================================
// HELPER FUNCTIONS
//=============================================================================

// --- Reads Rotary Encoder State ---
// This function polls the encoder pins and updates `encoderPos` and `encoderRotated`.
void readEncoder() {
  encoderPinAState = digitalRead(ENCODER_PIN_A); // Read Encoder Pin A
  // If Pin A's state has changed since last read...
  if (encoderPinAState != encoderPinALast){
    encoderPinBState = digitalRead(ENCODER_PIN_B); // Read Encoder Pin B
    // If Pin B's state is different from Pin A's, rotation is counter-clockwise
    if (encoderPinBState != encoderPinAState) {
      encoderPos--;
    } else {
      // If Pin B's state is the same as Pin A's, rotation is clockwise
      encoderPos++;
    }
    encoderRotated = true; // Set flag indicating rotation occurred
    // Optional: Serial.print("Encoder Pos: "); Serial.println(encoderPos);
  }
  encoderPinALast = encoderPinAState; // Store current Pin A state for the next comparison
}

// --- Updates the OLED Display ---
// This function clears the display and redraws content based on the current mode and values.
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (currentMode == BPM_MODE) {
    // --- BPM Mode Display ---
    // Blinking "BPM:" text (toggles visibility every 500ms)
    if ((millis() / 500) % 2 == 0) {
      display.setCursor(0, 0);
      display.print("BPM:");
    }

    // Display BPM value larger
    display.setTextSize(3);
    display.setCursor(50, 20); // Position for larger BPM text
    display.print(bpm);

    // Display current Preset number (formatted to 3 digits with leading zeros)
    display.setTextSize(1);
    display.setCursor(0, 50); // Position for preset info
    display.print("Preset:");
    char presetStr[4]; // Buffer for formatted string "NNN\0"
    sprintf(presetStr, "%03d", currentPreset);
    display.print(presetStr);

  } else if (currentMode == PRESET_MODE) {
    // --- Preset Mode Display ---
    // Blinking "Preset:" text
    if ((millis() / 500) % 2 == 0) {
      display.setCursor(0, 50);
      display.print("Preset:");
    }

    // Display Preset number larger (formatted to 3 digits)
    display.setTextSize(3);
    display.setCursor(30, 20); // Position for larger preset text
    char presetStr[4];
    sprintf(presetStr, "%03d", currentPreset);
    display.print(presetStr);

    // Display current BPM value smaller
    display.setTextSize(1);
    display.setCursor(0, 0); // Position for BPM info
    display.print("BPM:");
    display.print(bpm);
  }

  display.display(); // Push the buffer to the OLED screen
}

// --- Sends Simulated Tap Tempo MIDI Messages ---
// Called when the encoder switch is pressed in BPM_MODE.
// Sends 4 rapid CC#3 messages to simulate tap tempo input for DAWs.
void sendSimulatedTapTempo() {
    Serial.println("Encoder Switch Pressed: Simulating Tap Tempo (x4)");

    // Calculate delay between taps based on current BPM
    // This makes the simulated taps match the current tempo
    int tapDelay = 60000 / bpm;
    Serial.print("Tap Delay per beat = ");
    Serial.print(tapDelay);
    Serial.println(" ms");

    for (int i = 0; i < 4; i++) {
        MIDI.controlChange(3, 127, MIDI_CHANNEL_IN); // Send CC#3, value 127
        Serial.print("Sent MIDI CC#3 Value: 127 (Tap ");
        Serial.print(i + 1);
        Serial.println(")");
        // Ensure a minimum delay between taps, even if BPM is very high,
        // to prevent overwhelming the MIDI receiver or sending taps too quickly.
        delay(max(tapDelay, 20)); // Use max() to ensure at least 20ms delay
    }
    // Note: This function does not update the internal 'bpm' variable itself.
    // It relies on the connected DAW/software to interpret these taps.
    // If desired, internal BPM could be set here, but typically tap tempo
    // is an *input* to determine BPM, not an *output* based on current BPM.
}

// --- Handles Incoming MIDI Messages ---
// Parses received MIDI packets and acts upon specific messages (e.g., CC for LED control).
void handleMidiMessage(midiEventPacket_t rx_packet) {
  uint8_t msg_type = rx_packet.byte1 & 0xF0;      // Extract message type (e.g., Note On, CC)
  uint8_t channel = (rx_packet.byte1 & 0x0F) + 1; // Extract MIDI channel (0-15 -> 1-16)
  uint8_t data1 = rx_packet.byte2;                // First data byte (e.g., Note Number, CC Number)
  uint8_t data2 = rx_packet.byte3;                // Second data byte (e.g., Velocity, CC Value)

  // Ignore messages not on the configured input channel
  if (channel != MIDI_CHANNEL_IN) { return; }

  Serial.print("Received MIDI:");
  Serial.print(" Type=0x"); Serial.print(msg_type, HEX);
  Serial.print(" Chan="); Serial.print(channel);
  Serial.print(" Data1="); Serial.print(data1);
  Serial.print(" Data2="); Serial.println(data2);

  switch (msg_type) {
    case 0xB0: // Control Change (CC) message
      if (data1 == CC_LED_CONTROL) { // Check if it's the CC for LED control
        ledState = (data2 > 64); // Typically, value > 64 means ON, <= 64 means OFF
        digitalWrite(ledPin, ledState ? HIGH : LOW); // Update physical LED
        controlLedViaMidi = true; // Mark that LED is now controlled by MIDI
        oledNeedsUpdate = true;   // Update OLED (e.g., to show MIDI control status or LED state)
        Serial.print("LED Controlled by MIDI CC#"); Serial.print(CC_LED_CONTROL);
        Serial.println(ledState ? " -> ON" : " -> OFF");
      }
      break;
    case 0x90: // Note On message
      Serial.println("Received Note On");
      // Add Note On handling if needed
      break;
    case 0x80: // Note Off message
      Serial.println("Received Note Off");
      // Add Note Off handling if needed
      break;
    // Add other MIDI message types as needed
  }
}