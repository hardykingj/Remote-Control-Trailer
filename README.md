# Remote-Control-Car-and-Trailer

Code for a scale model of a remote control car and trailer; developed for a University project.

## Controller Information

Any bluetooth controller can be connected to the ESP32; this was implemented using BluePad32

## Compiling Code (Windows)

1. Install ESP-IDF v4.4. For further info, read: ESP-IDF Getting Started for Windows
    * Either the Online or Offline version shoud work
    * When asked which components to install, don't change anything. Default options are Ok.
    * When asked whether ESP can modify the system, answer "Yes"

2. Launch the "ESP-IDF v4.4 CMD" (type that in the Windows search box)

3. CD to the "Remote-Control-Car-and-Trailer" directory

4. Compile the code:

    ```sh
    # Compile the Code
    idf.py build

    # Flash + Open Debug Terminal
    idf.py flash monitor
    ```
