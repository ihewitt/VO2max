# VO2max
Code and build files for Rabbitcreek's VO2Max Spirometer. _Incorporating CO2 enhancements from Ulrich Rissel._
Build instructions on [Instructable](https://www.instructables.com/Accurate-VO2-Max-for-Zwift-and-Strava/)

Source code for Arduino under "VO2Max" - Arduino board settings to use for TTGO T-Display:

    Board: ESP32 Dev Module
    Upload Speed: 921600
    CPU Frequency: 240Mhz (WiFi/BT)
    Flash Frequency: 80Mhz
    Flash Mode: QIO
    Flash Size: 4MB (32Mb)
    Partition Scheme: Default 4MB with spiffs (1.2MB APP/1.5 SPIFFS)
    Core Debug Level: None`

![Build parts](/images/parts.jpg "Build parts")
![First build](/images/built.jpg "First build")
![Upgraded build](/images/built2.jpg "Upgraded build")
Pictured with the CO2 sensor upgrade attached (waiting for glue to dry!)

3D printing files are within the `design` folder, Ulrich Rissel's design files to use a larger venturi diameter with CO2 sensor holder in `design/CO2_upgrade`

Additional changes in this version:
- Menu system enhanced with adjustable calibration and setup options.
- Additional GoldenCheetah integration (with VO2 master output)
- CO2 sensor support (Ulrich's mods)
