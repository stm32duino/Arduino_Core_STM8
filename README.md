# Arduino core support for STM8 based boards
[![GitHub release](https://img.shields.io/github/release/stm32duino/Arduino_Core_STM8.svg)](https://github.com/stm32duino/Arduino_Core_STM8/releases/latest)

* [Introduction](https://github.com/stm32duino/Arduino_Core_STM8#Introduction)<br>
* [Getting Started](https://github.com/stm32duino/Arduino_Core_STM8#getting-started)<br>
* [Boards available](https://github.com/stm32duino/Arduino_Core_STM8#boards-available)<br>
* [Troubleshooting](https://github.com/stm32duino/Arduino_Core_STM8#troubleshooting)<br>
* [Wiki](https://github.com/stm32duino/wiki/wiki/)

## Introduction

This repo adds the support of STM8 architecture in Arduino IDE.<br>

This porting is based on several external components :
* STMicroelectronics Standard Peripheral Libraries (SPL)
  * [SPL for STM8L](https://www.st.com/en/embedded-software/stsw-stm8016.html)
  * [SPL for STM8S](https://www.st.com/en/embedded-software/stsw-stm8069.html)
* Cosmic compiler, allowing to compile c++ source code on stm8 family (http://www.cosmic-software.com/)

Solution is running on Windows only.

## Getting Started

This repo is available as a package usable with [Arduino Boards Manager](https://www.arduino.cc/en/guide/cores).

Use this link in the "*Additional Boards Managers URLs*" field:

https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json

**Warning**:
* Default branch has changed to main.
* Since core release 1.0.0 this link has changed.

For full instructions on using the "**Boards Manager**", see the [Getting Started](https://github.com/stm32duino/wiki/wiki/Getting-Started) page.

For advanced user, you can use the repository: see the [Using git repository](https://github.com/stm32duino/wiki/wiki/Using-git-repository) page.

## Boards available
  * STM8L
    * [Nucleo STM8L152R8](https://www.st.com/en/evaluation-tools/nucleo-8l152r8.html)

  * STM8S
    * [Nucleo STM8S208RB](https://www.st.com/en/evaluation-tools/nucleo-8s208rb.html)


## Troubleshooting and known issues

Please note that this is a new core using a new compiler. You may face some issues with libraries compatibility.<br>
We have several known issues :
 * Windows 10:
 *Board binary may be corrupted when plugged on a Windows 10 machine.
    To ensure compatibility with Windows 10, please upgrade STLink with revision 2.32.22 or higher (estimated availability October 1st).*
 * Endianness:
 *STM8 microcontrollers are natively Big Endian whereas Arduino boards and STM32 microcontrollers are Little Endian.
	By default, some libraries don't handle endianess, therefore these won't be directly compatible with STM8 (this is the case of SD library for example).*

If you have any issue, you could [file an issue on Github](https://github.com/stm32duino/Arduino_Core_STM8/issues/new).

If you have a question about compiler, you can also send a mail to Cosmic support : [send a mail to Cosmic support](mailto:support@cosmic.fr)

