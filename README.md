# Appliance-Beep-Detector
Arduino-based device for detecting appliance beeps and sending a notification. Uses an Arduino Uno R4 WiFi, electret microphone, and Goertzel tone detection to identify beep patterns from appliances such as washers, dryers, or ovens.

![License](https://img.shields.io/badge/license-MIT-green)
![Platform](https://img.shields.io/badge/platform-Arduino-blue)
![Python](https://img.shields.io/badge/tool-Python-yellow)
![Hardware](https://img.shields.io/badge/hardware-Arduino%20Uno%20R4%20WiFi-orange)
---
## Repository Structure

- Arduino_Beep_Detector.ino
  - Arduino firmware implementing the tone detector

- beep_detector_logger.py
  - Python script for logging serial data and visualising detection behaviour

---

## HARDWARE

- Arduino Uno R4 WiFi
- Adafruit MAX9814 electret microphone (Adafruit product number 1713)

---

## WIRING

Connect:
- microphone 'GND' to Arduino 'GND'
- microphone 'Vdd' to Arduino '5V'
- microphone 'Out' to 'A0'

---

## ARDUINO SETUP

The detector works by:

1. Sampling audio at 20 kHz (needs to be at least 2x your highest frequency of interest to account for the Nyquist limit)
2. Running Goertzel filters at the target frequencies. Goertzel used instead of FFT because it requires less computation for a small number of frequencies.
3. Comparing magnitudes against a rolling baseline
4. Using a state machine to detect the beep / pause sequence

The sketch is currently tuned for detecting a sequence of:

- 3x 1s beeps with 1s pauses between them.

Each beep contains two tones:
- 2400Hz
- 4800Hz
  
These tones and timings were determined by recording the appliance beep on a phone and analysing it in Audacity. The parameters and state machine detection logic can be modified to support different beep frequencies, durations, or patterns.

Example of Audacity view including the frequency analysis where the two distinct tones can be seen as the sharp peaks (the 2400Hz one is selected, see the red line):

<img width="2241" height="1244" alt="Screenshot 2026-03-08 155141" src="https://github.com/user-attachments/assets/9939ccfe-9280-45a2-9816-f4e47b36a58c" />

<br>
<br>

The Arduino sketch can also be configured to send a notification via Arduino cloud or by integration into other IoT / smart home systems.

### DETECTION TIMING NOTES

The timeouts used in the detection state machine are intentionally longer than the actual beep durations.

This accounts for:
- loop execution time
- sampling jitter
- occasional missed detections

Without these margins the detector occasionally became unreliable.

### LED MATRIX EQ DISPLAY

An LED matrix “EQ” display was included mainly for experimentation and to make use of the built-in LED matrix on the Uno R4 WiFi.

It provides a rough visualisation of energy in several frequency bands.

If running the sketch on an Arduino without the built-in matrix, the EQ section can be removed without affecting the detector functionality.

---

## PYTHON SCRIPT

The Python script is used to help tune the detector parameters.

It:
- logs the Arduino serial output
- generates an interactive Plotly chart
- saves a CSV file for detailed inspection

To use it:
1. Connect the Arduino to your computer
2. Run the Python script
3. Play real or recorded appliance beeps into the microphone

Note: the script currently uses Windows keyboard handling (msvcrt), so the “Press ENTER to start recording” feature may not work on macOS or Linux.

Dependencies: pip install pyserial plotly

Example output plot:
<img width="1630" height="822" alt="newplot (3)" src="https://github.com/user-attachments/assets/a6e3849a-98ef-439e-a174-0e7ebb1f45ca" />

---

## FUTURE ADDITIONS
- A Python mirror of the Arduino detection code for running analysis directly on WAV files along with an Arduino + Python pair for creating WAV files via the Arduino. This allows repeatable tuning without needing to replay recordings into the microphone each time.

- A 3D printed wall mounted housing with clear acrylic lid is WIP. Photos and CAD files will be uploaded when ready.
