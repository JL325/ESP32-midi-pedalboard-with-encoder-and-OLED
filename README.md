# ESP32 MIDI Controller (OLED & Rotary Encoder)

This is an ESP32-based MIDI controller project featuring an OLED screen to display BPM and Preset information, adjustable via a rotary encoder. It also includes multiple buttons for triggering MIDI CC messages and a Tap Tempo function.
This MIDI pedalboard will be used with the Reaper DAW and the Helix Native VST to switch electric guitar tones (snapshots & presets), includes BPM control (tap tempo/direct setup). I will update my demo video later.
![image](https://github.com/user-attachments/assets/8a98148f-e535-4958-8c29-de750c6980bf)
![5DD8A519-B131-4BED-B63F-6E00A2BC3400](https://github.com/user-attachments/assets/0b60032a-0886-4dc4-8650-d23a8e851b20)
![33C69B30-7150-4691-89FA-017B33A18C91](https://github.com/user-attachments/assets/c33616a3-a082-4a7a-9b3b-42c542759f0e)


## ✨ Features

*   **BPM Adjustment:** Use the rotary encoder to adjust BPM (30-250 BPM).
*   **Preset Mode:** Select Presets (0-127) using the rotary encoder and send MIDI Program Change messages by pressing the encoder switch.
*   **OLED Display:**
    *   In BPM Mode, primarily displays BPM, secondarily shows the current Preset.
    *   In Preset Mode, primarily displays Preset, secondarily shows the current BPM.
    *   Mode indicator text (BPM/Preset) blinks.
*   **Tap Tempo:** Dedicated button (Button 0) implements Tap Tempo functionality and sends CC#3.
*   **Rotary Encoder Switch:**
    *   In BPM Mode, pressing it simulates 4 Tap Tempo events (based on the current BPM).
    *   In Preset Mode, pressing it sends the selected Program Change.
*   **Multiple MIDI Buttons:**
    *   21 buttons mapped to various MIDI CC messages. (1 button already on the board)
    *   Pressing specific trigger buttons (Button 1-8, CC#69) resets all toggle buttons (Button 11-18) to their OFF state.
*   **Mode Switching:** Dedicated button (Button 20) switches between BPM Mode and Preset Mode.
*   **MIDI LED Indication:**
    *   Onboard LED blinks according to the BPM.
    *   Can be externally controlled via MIDI CC#20 (Channel 1).
*   **USB MIDI:** Acts as a standard MIDI device over USB.

## 🔩 Hardware Requirements

*   ESP32 Development Board (ESP32S2 mini)
*   SSD1306 OLED Display (I2C, 0.96 inch)
*   Rotary Encoder Module (EC11, with push-button switch)
*   21 Push Buttons (momentary, 1 button already on the board)
*   1 LED (already on the board)
*   Pull-up/pull-down resistors (may be needed depending on your button and encoder wiring)

## 💻 Software Requirements

*   Arduino IDE
*   ESP32 Board Package for Arduino IDE
*   The following Arduino Libraries:
    *   `Adafruit GFX Library`
    *   `Adafruit SSD1306`
    *   `ESP32-USB-MIDI` (often included with the ESP32 USB libraries, ensure you have a version that provides `USBMIDI.h`)
    *   `Wire` (built-in)

## 🛠️ Installation & Setup

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/YOUR_USERNAME/YOUR_REPOSITORY_NAME.git
    cd YOUR_REPOSITORY_NAME
    ```
2.  **Install Arduino IDE and ESP32 Core:** Refer to the [ESP32 Arduino Core Installation Guide](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html).
3.  **Install necessary libraries:** In the Arduino IDE, go to "Sketch" -> "Include Library" -> "Manage Libraries..." and search for and install the libraries listed above.
4.  **Connect the hardware:** Wire your OLED, rotary encoder, buttons, and LED according to the pin definitions in the code.
    *   OLED SDA: Pin 17
    *   OLED SCL: Pin 16
    *   Encoder CLK (ENCODER_PIN_A): Pin 13
    *   Encoder DT (ENCODER_PIN_B): Pin 14
    *   Encoder SW (ENCODER_PIN_SW): Pin 12
    *   LED Pin: Pin 15
    *   Button Pins: Refer to the `buttonPins` array in the code.
5.  **Configure Arduino IDE:**
    *   Select the correct ESP32 board.
    *   Select the correct serial port.
6.  **Upload the code:** Click the "Upload" button in the Arduino IDE.

## 🚀 Usage

*   **Power On:** After connecting via USB, the controller will start, display a splash screen, and then enter BPM Mode.
*   **BPM Mode:**
    *   **Rotate Encoder:** Adjusts BPM. Fast rotation enables acceleration.
    *   **Press Encoder Switch:** Simulates 4 Tap Tempo events.
    *   **Press Button 0 (Tap Tempo):** For manual Tap Tempo input.
    *   **Press Button 20:** Switches to Preset Mode.
*   **Preset Mode:**
    *   **Rotate Encoder:** Selects MIDI Preset (0-127).
    *   **Press Encoder Switch:** Sends the currently selected Program Change message (MIDI Channel 1).
    *   **Press Button 20:** Switches back to BPM Mode.
*   **Buttons:**
    *   **Button 0:** Tap Tempo (CC#3, Value 127, Channel 1)
    *   **Button 1-8:** Trigger buttons (CC#69, Value 0-7, Channel 1). Pressing these resets Buttons 11-18 to OFF.
    *   **Button 9:** CC#1, Value 1, Channel 1
    *   **Button 10:** CC#2, Value 2, Channel 1
    *   **Button 11-18:** Toggle buttons.
        *   Button 11: CC#4 (0/127)
        *   Button 12: CC#5 (0/127)
        *   Button 13: CC#6 (0/127)
        *   Button 14: CC#7 (0/127)
        *   Button 15: CC#8 (0/127)
        *   Button 16: CC#9 (0/127)
        *   Button 17: CC#10 (0/127)
        *   Button 18: CC#11 (0/127)
    *   **Button 19:** CC#12, Value 0, Channel 1
    *   **Button 20:** Mode Switch (BPM/Preset)
*   **MIDI Input:**
    *   The controller listens on MIDI Channel 1.
    *   Receiving CC#20 will control the LED state based on its value (> 64 for ON).
