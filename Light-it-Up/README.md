# Light-it-Up

Compiling and flashing this project via ESP-IDF toolchain onto ESP32 board will allow you to control the LED with nothing else but touch.

## How to use

First, you need to set up your hardware. You'll need an ESP32 chip, LED and a wire.

Connect the LED's positive terminal to the GPIO 23 and it's negative terminal to the GND pin.

Attach the wire to the GPIO 4.

To build and flash, the simplest way is use the VS Code extension provided by Espressif, make sure to install it first.


1) Connect to your chip via usb cable (make sure it's not charging cable only)

2) Open the VS Code command panel (ctrl + shift + P) and type "ESP-IDF: ", you'll then see various commands provided by the ESP-IDF framework

3) From the listed commands, select "Select port to use" and pick the port to which the device is connected

4) Then search for the "Build, Flash and start monitor on your device" command and run it. Choose UART flashing method when the selection pops up.

5) When the previous command finishes, you should see a logger window in your VS Code IDE, wait till you see a line looking like this: "I (X) main_task: Calling app_main()"

6) Touch a conductive part of the wire and see what happens!
