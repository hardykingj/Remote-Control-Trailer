# Remote-Control-Trailer

Code for a scale model of a remote control trailer; developed for a University project.

## Controller Information

Any bluetooth controller can be connected to the ESP32; this was implemented using BluePad32

## Compiling Code (Windows)

1. Install ESP-IDF v4.4. For further info, read: ESP-IDF Getting Started for Windows
    * Either the Online or Offline version shoud work
    * When asked which components to install, don't change anything. Default options are Ok.
    * When asked whether ESP can modify the system, answer "Yes"

2. Launch the "ESP-IDF v4.4 CMD" (type that in the Windows search box)

3. CD to the "Remote-Control-Trailer" directory

4. Compile the code:

    ```sh
    # Compile the Code
    idf.py build

    # Flash + Open Debug Terminal
    idf.py flash monitor
    ```

## ESP32 Setup

1. Create a wireless network with the following credentials:
    * SSID = "TPD_Test_Wifi"
    * Password = ""

2. Pair the controller with the ESP32

3. Install RealTerm: Serial/TCP Terminal, read the website for more information. Configure the software as follows:
    * Display as = Ascii
    * NewLine Mode = Enabled
    * Bytes per Data frame = 2
    * Terminal Rows = 40
    * Terminal Cols = 80
    * Baud Rate = 115200
    * Port: (Dependent on the ESP's IP):80
    * Parity: None
    * Data Bits: 8
    * Stop Bits: 1
    * Hardware Flow Control: None
    * Winsock is: Telnet
    * TimeStamp: YMDHS
    * Delimiter: Comma

4. Using RealTerm open the conection to the ESP32
