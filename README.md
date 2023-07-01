<p align="right">
    <a name="readme-top"></a>
    <a href="/LICENSE.txt"><img src="https://img.shields.io/badge/license-MIT-green" /></a> <a href=""><img src="https://img.shields.io/badge/version-1.0.0-red" /></a> 
</p>
<br><br>
<p align="center">
    <h1 align="center">BD845-PWB Firmware</h1>
    <p align="center">This repository contains the source code of the BD845-PWB firmware.</p>
    <p align="center"><strong><a href="https://github.com/bluedot-arcade/bd845-pwb-board">Go to board repository</a></strong></p>
    <p align="center"><strong><a href="https://docs.bluedotarcade.com/boards/bd845-pwb">Explore the docs</a></strong></p>
    <br><br>
</p>

## Features

- [X] **Sensors polling**
- [X] **Sensors debouncing**: when the sensor is released it will be considered pressed for an additional 4 milliseconds.
- [X] **Light-on-press mode**: when enabled the panel light will turn of when the panel is pressed, without the need of an external light driver.
- [X] **External lights driver**: lights can be controlled by an external driver, just like the original board.
- [X] **Legacy mode**: when enabled the board will check for the KSYS573 DDR init command.
- [X] **KSYS573 IO check**: emulates the original KSYS573 stage IO check performed on boot sequence.
- [X] **Sensors mask**: a command can be sent to the board to set the sensors mask and disable faulty sensors or check individual sensor state. The same command works also on the original board.

For more info check the [documentation].

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Project structure

* [/boards/](/boards/) - contains the PlatformIO definitions for the custom board.
* [/inc/](/inc/) - contains project include files.
* [/lib/](/lib/) - contains project specific private libraries.
* [/src/](/src/) - contains project source files.
* [/test/](/test/) - contains project test files.
* [/bd845-pwb.ioc](/bd845-pwb.ioc) - the STM32CubeMX project file.
* [/platformio.ini](/platformio.ini) - the PlatformIO project configuration file.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## How to build and upload

This project uses PlatformIO to build and upload the project. Only the PlatformIO Core CLI is required but consider installing the PlatformIO IDE for a better user experience.

The following is the recommended way to build and upload the firmware.

1. [Install PlatformIO Core][PlatformIO Core Docs]

2. Clone the repository 
    ```bash
    git clone https://github.com/bluedot-arcade/bd845-pwb-firmware
    ```

3. Navigate to the root of the project
   ```bash
    cd bd845-pwb-firmware
    ```

4. Build the project. Binaries will be created inside the `.pio` folder.
    ```bash
    platformio run
    ```
    
5. Upload the project to the board
    ```bash
    platformio run --target upload
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## License

Distributed under the MIT License. See [LICENSE.txt] for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

[LICENSE.txt]: /LICENSE.txt
[PlatformIO Core Docs]: https://docs.platformio.org/en/latest/core/index.html
[PlatformIO Docs]: https://docs.platformio.org/
[documentation]: https://docs.bluedotarcade.com/boards/bd845-pwbhttps://docs.bluedotarcade.com/boards/bd845-pwb