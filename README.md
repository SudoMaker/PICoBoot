# PICoBoot

Supercharged USB bootloader for various PIC24/dsPIC33 MCUs.

## Features
### Community version

- Supports all PIC24/dsPIC33 MCUs with USB module 
- No driver installation needed on modern host OS (Linux/macOS/Windows)
- `fastboot` like command line tool, for quick integration into development workflows
- Versatile protocol with strong integrity check
- Non-volatile environment variable (like U-Boot)
- Read/Write inhibit, to prevent the firmware from being read out or overwritten
- No interrupt proxying, keeps you fast in reacting to interrupts

### Commercial version

- All community version features
- Software based voltage glitch protection
- Firmware checksum & signing support  
- Customizable OEM commands

## Usage
### Installation
- Flash the `.hex` file to your board using the MPLAB X IPE. (This is the only one we can use at this moment. One day we will get `openocd` to support PIC24!)
- Usually, you just need to hold the user button and push the MCLR button to enter bootloader. However, depending on the board definition, some boards may have different ways for this.
- Connect the board to your computer.
- Use the [`picoboot` utility](https://github.com/SudoMaker/PICoBoot_Utility) to manipulate the device.


![Screenshot_20210519_203901](https://user-images.githubusercontent.com/34613827/118813974-41bda780-b8e2-11eb-9c12-2e6157f4c421.png)


## Supported boards / products
|Board|MCU|Enter Bootloader|
|---|---|---|
|PotatoPi PICo24|PIC24FJ256GB206|Hold RD7 + MCLR|
|CartBoy RW v1|PIC24FJ256GB108|TBD|

If you use PICoBoot in your project, feel free to expand this list!


## Development
Hey! It's 2021! Use CMake!

### Build requirements
- XC16 1.70+
- CMake 3.13+

![Screenshot_20210519_204243](https://user-images.githubusercontent.com/34613827/118814464-c3add080-b8e2-11eb-8df7-b0c34b17f043.png)

![Screenshot_20210519_204324](https://user-images.githubusercontent.com/34613827/118814558-dd4f1800-b8e2-11eb-9c44-b501e8f393eb.png)

### IDE
- Any IDE with proper CMake support would be good. Personally I recommend CLion.
- Using MPLAB X IDE is a sin and totally unforgivable!

![Screenshot_20210519_204636](https://user-images.githubusercontent.com/34613827/118814966-4f276180-b8e3-11eb-999b-491e74062448.png)

### Adding new boards
- Duplicate an existing directory in `Boards` directory
- Change it to the board's name
- Modify `PICoBoot_Board.*` source code files and `PICoBoot.ld` linker script, edit the board info, boot mode logic, button handling, flash offsets, init routines as you like. Ensure the bootloader is installed to the second page of flash.
- Add the board info to `CMakeLists.txt`. The syntax is `picoboot_add_board(board_name chip_name heap_size)`.
- You can use existing ones as examples.

### User application (firmware) modification
Only the linker script needs to be modified.
- Change the start position of program data to the address of the next page after last page used by bootloader
- Decrease the program data length by minus it with the size of pages used by bootloader
- That's all!

## How to help this project
- Code improvements, of cource
- **Urge Microchip to be more opensource friendly:**
    + **Stop selling their outdated compiler**
    + **Add PIC18/24/33 support to mainline GCC & GDB**
- Buy a commercial version of PICoBoot, if you need the extra features
- Have a look at our [store on Tindie](https://www.tindie.com/stores/sudomaker/)


## Licensing
### Community version
AGPLv3. Also read [this FAQ](https://www.gnu.org/licenses/gpl-faq.html#GPLRequireSourcePostedPublic).

### Commercial version
Please contact us.