# Shoulder exoskeleton – Viexo project

## Main Controller:
This folder contains an '.ino' code written for the **Teensy 4.1**.
The current controller used is:
**'Julian_AAN_v6_1_T.ino.'**

---

## GUI:
Right now there is no avaliable GUI to Control Teensy.
The Teensy responds to the following serial commands:

- **START** – starts the controller  
- **STOP** – stops the controller  
- **SET** – updates signal or tuning parameters



### SET Command Format
Use the `SET` command to change sine-wave properties or the tuning parameters 
**Example:**

"SET 0.1 35.0 25.0 1.57 0.01 1.5"

This sets the sine wave to:

- **Frequency:** 0.1 Hz  
- **Amplitude:** 35&ddeg  
- **Offset:** 25&ddeg  
- **Phase shift:** 1.57 rad  

And updates the tuning parameters:

- **a = 0.01**  
- **&alpha = 1.5** 

---

## Reading SD Card Data
Use **`Bin_file_reader.py`** to read `.bin` files stored on the SD card.

---