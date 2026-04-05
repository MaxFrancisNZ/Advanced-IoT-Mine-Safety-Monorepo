# Uploading Firmware to an ESP32 using esptool

This guide explains how to upload a compiled firmware file
(e.g. `firmware_vXX.bin`) to an ESP32 using **esptool**.

Reference: Espressif esptool documentation.

------------------------------------------------------------------------

# 1. Requirements

-   ESP32 board
-   USB cable (data capable)
-   Computer with Python installed
-   Firmware file (e.g. `firmware_v01.bin`)

------------------------------------------------------------------------

# 2. Install Python

Check if Python is installed:

``` bash
python --version
```

or

``` bash
python3 --version
```

If not installed, download from:

https://www.python.org

------------------------------------------------------------------------

# 3. Install esptool

## Windows

Open **PowerShell** or **Command Prompt** with Administrator privileges:

``` bash
pip install esptool
```

Verify installation:

``` bash
esptool version
```

If `esptool` is not recognized:

``` bash
python -m esptool version
```

------------------------------------------------------------------------

## Linux

Install using pip:

``` bash
pip3 install esptool
```

Verify installation:

``` bash
esptool version
```

Alternatively (Ubuntu/Debian):

``` bash
sudo apt install esptool
```

------------------------------------------------------------------------

# 4. Connect the ESP32

1.  Connect the ESP32 to your computer via USB.
2.  Identify the serial port.

## Windows

Look in **Device Manager → Ports (COM & LPT)**.

Example:

    COM3

## Linux

Run:

``` bash
ls /dev/ttyUSB*
```

or

``` bash
ls /dev/ttyACM*
```

Example:

    /dev/ttyUSB0

------------------------------------------------------------------------

# 5. (Optional) Erase Flash

Recommended before installing new firmware.

``` bash
esptool --port <PORT> erase-flash
```

Example:

``` bash
esptool --port COM3 erase-flash
```

or

``` bash
esptool --port /dev/ttyUSB0 erase-flash
```

------------------------------------------------------------------------

# 6. Upload Firmware

Run:

``` bash
esptool -p COM7 -b 460800 --before default-reset --after hard-reset --chip esp32 write-flash --flash_mode dio --flash_freq 40m --flash_size 4MB 0x0 firmware_vXX.bin
```

Example:

``` bash
esptool -p COM7 -b 460800 --before default-reset --after hard-reset --chip esp32 write-flash --flash_mode dio --flash_freq 40m --flash_size 4MB 0x0 firmware_v1.bin
```


Explanation:

  Option               Description
  -------------------- -------------------------
  `-p`                 Serial port
  `--before`           Command to execute before upload
  `--after`            Command to execute after upload
  `--chip`             Target device
  `write_flash`        Write firmware to flash
  `0x0`                Flash offset (0x0 for pre-merged firmware file)
  `firmware_vXX.bin`   Pre-merged firmware file


------------------------------------------------------------------------

# 7. Troubleshooting

### Permission denied (Linux)

``` bash
sudo usermod -a -G dialout $USER
```

Then log out and back in.

### Flash fails at high baud rate

Lower the speed:

``` bash
--baud 115200
```

### Cannot connect

Hold the **BOOT** button while starting the flash command.

------------------------------------------------------------------------

# 9. Minimal Flash Command

``` bash
esptool --port <PORT> write_flash 0x1000 firmware_vXX.bin
```
