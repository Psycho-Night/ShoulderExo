# Shoulder exoskeleton – Viexo project

## Main Controller:
This folder contains an `.ino` code written for the **Teensy 4.1**.
The current controller used is:
**`Julian_AAN_v6_2_T.ino.`**

In **`Constants.h`** value FinalIteration specifies number of iterations to run.
Duration of each run is specified in **`Globals.h`** as RunDuration and is multiplied period of sin wave.

---

## GUI:
In order to control exo use **`GUI.py`**.
There are three avaliable commands:

- **START** – starts the controller  
- **STOP** – stops the controller  
- **SET** – updates signal and tuning parameters based on values in brackets

Live plotting shows last 60s of received signal.

---

## Reading SD Card Data
Use **`Bin_file_reader.py`** to read `.bin` files stored on the SD card.
At the top specify folder direction of your `.bin` files and number of iterations that were gathered.

---