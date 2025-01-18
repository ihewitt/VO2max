# VO₂max
Code and build files for Rabbitcreek's VO₂Max Spirometer. _Incorporating CO₂ enhancements from Ulrich Rissel._
Build instructions on [Instructable](https://www.instructables.com/Accurate-VO2-Max-for-Zwift-and-Strava/)
See Nik's pages for excellent build details: [meteoscientific](https://github.com/meteoscientific/VO2max)

Source code for Arduino under "VO₂Max" - Arduino board settings to use for TTGO T-Display:

    Board: ESP32 Dev Module
    Upload Speed: 921600
    CPU Frequency: 240Mhz (WiFi/BT)
    Flash Frequency: 80Mhz
    Flash Mode: QIO
    Flash Size: 4MB (32Mb)
    Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)
    Core Debug Level: None
    PSRAM: Disabled

<figure>
    <img src="/images/parts.jpg" width="640" height="480"
         alt="Build parts">
    <figcaption>Source parts, top to bottom. 3M mask with front plate removed, 3D printed case, Oxygen sensor, TTGo T-Display, Flow sensor.</figcaption>
</figure><br><br>
<figure>
    <img src="/images/built.jpg" width="640" height="480"
         alt="First build">
    <figcaption>First finished build.</figcaption>
</figure><br><br>
<figure>
    <img src="/images/upgrading.jpg" width="640" height="480"
         alt="Upgrading">
    <figcaption>Rebuilding to use CO₂ sensor. SCD30 pictured right.</figcaption>
</figure><br><br>
<figure>
    <img src="/images/casefilling.jpg" width="640" height="480"
         alt="Upgraded build">
    <figcaption>Assembled into case tightly, BM280 barometer addition mounted onto front of tube, wiring for CO₂ monitor fed behind and out to top.</figcaption>
</figure><br><br>
<figure>
    <img src="/images/built2.jpg" width="640" height="480"
         alt="Upgraded build">
    <figcaption>Pictured with the CO₂ sensor upgrade attached</figcaption>
</figure><br><br>

3D printing files are within the `design` folder, Ulrich Rissel's design files to use a larger venturi diameter with CO₂ sensor holder in `design/CO2_upgrade`

Additional changes in this version:
- Menu system enhanced with adjustable calibration and setup options.
- Additional GoldenCheetah integration (with VO₂ master output)
- CO₂ sensor support (Ulrich's mods)
- STC31 CO₂ sensor support
- Ability to run without O₂ sensor

More notes on the CO₂ sensor use can be found at: [CO₂ sensors](https://blog.ivor.org/2024/12/carbon-dioxide.html)
Details of the interface: [Screens](docs/screens.md)
