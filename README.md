<p align="right">
    <a name="readme-top"></a>
    <a href="/LICENSE.txt"><img src="https://img.shields.io/badge/license-MIT-green" /></a> <a href=""><img src="https://img.shields.io/badge/version-0.0.1-red" /></a> 
</p>
<br><br>
<p align="center">
    <h1 align="center">BD845-PWB Firmware</h1>
    <p align="center">This repository contains the source code of the BD845-PWB firmware.</p>
    <p align="center"><strong><a href="https://github.com/bluedot-arcade/bd845-pwb-board">Go to board repository</a></strong></p>
    <p align="center"><strong><a href="https://docs.bluedotarcade.com/boards/bd845-pwb">Explore the docs</a></strong></p>
    <br><br>
</p>

## Status

The project is still in development. Don't expect all things to work just yet.
The DDR Stage IO communication protocol has not been yet implemented.

Feature status:
- [X] Sensors polling and outputting.
- [X] Sensors debouncing.
- [X] Light-on-press mode.
- [X] External lights driver.
- [ ] Legacy mode (KSYS573 protocol enable).
- [ ] KSYS573 IO check on boot sequence.
- [ ] KSYS573 Sensor addressing in settings.

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