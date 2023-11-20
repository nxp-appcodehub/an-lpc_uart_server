# NXP Application Code Hub
[<img src="https://mcuxpresso.nxp.com/static/icon/nxp-logo-color.svg" width="100"/>](https://www.nxp.com)

## AN12625: Multi-TTY Expansion
In industry applications, it is a general requirement to expand massive serial ports and GPIOs. Usually, the specific solution is expensive and less flexible. In this document, we introduce one proof of concept to expand slow speed interfaces through general MCU. Only multi-TTY expansion is addressed in this document. The hardware and software architecture are suitable for virtual GPIOs and CAN ports as well.

The LPC540xx/LPC54S0xx is a family of Arm Cortex®-M4-based microcontrollers for embedded applications featuring a rich peripheral set with very low power consumption and enhanced debug features.

The LPC54018 includes 360 KB of on-chip SRAM, a quad SPI Flash Interface (SPIFI) for expanding program memory, one high-speed and one full-speed USB host and device controller, CAN FD, each Flexcomm interface of 10 serial peripherals can be selected by software to be a USART, SPI, or I2C interface



Please refer to AN12625for complete instructions on how to use this software.
https://www.nxp.com/docs/en/application-note/AN12625.pdf

| folder                     | description                                   |
| -------------------------- | --------------------------------------------- |
| kernel driver              | Linux kernel driver source files              |
| lpc_firmware_download_tool | Linux side DFU download tool and MCU firmware |
| user_space_test_program    | Linux user space program for test and demo    |
| misc                       | other reference                               |

#### Boards: LPCXpresso54S018
#### Categories: Industrial
#### Peripherals: DMA, UART
#### Toolchains: GCC

## Table of Contents
1. [Software](#step1)
2. [Hardware](#step2)
3. [Setup](#step3)
4. [Results](#step4)
5. [FAQs](#step5) 
6. [Support](#step6)
7. [Release Notes](#step7)

## 1. Software<a name="step1"></a>

This software does not use MCUSDK.

* user_space_test_program V1.0.0
* kernel_driver for Multi-TTY Expansion: V1.0.0

## 2. Hardware<a name="step2"></a>

* LPCXpresso54S018M-EVK Rev A 
* LPCXpresso546xx EVK Rev D

### LPC54S108-EVK or LPC546xx-EVK setup guide

If user has LPC54S018-EVK or LPC546xx-EVK on hand, the binary image can also work normally on it. But because of evk board's resource limitation not all of 10 UART ports are reserved on board, user can take below table of available UART ports on EVK as reference to debug. Except ttyUSB0 ports, user should use a USB2TTL tool to connect specified MCU UART pins to Putty, Tera Term or some else PC serial ports tool. 

|               UART Pins on EVK                | ttyUSB Ports |
| :-------------------------------------------: | :----------: |
| J13 Pin13(P3_27, TXD) / J13 Pin15(P3_26, RXD) |   ttyUSB4    |
|  J9 Pin12(P4_3, TXD) / J9 Pin10(P4_2, RXD)*   |   ttyUSB6    |
| J9 Pin18(P2_20, TXD)* / J9 Pin20(P2_19, RXD)  |   ttyUSB7    |
| J13 Pin18(P3_17, TXD) / J12 Pin3(P3_16, RXD)  |   ttyUSB8    |
|   J9 Pin8(P3_3, TXD) / J13 Pin11(P3_2, RXD)   |   ttyUSB9    |
|    J8 on board USB debugger(P0_30, P0_29)     |   ttyUSB0    |

** Before testing ttyUSB6 and ttyUSB7, the resistor R87 should be removed to disconnect P4_3 and P2_20 on EVK

Here please follow below steps to make EVK board on your hand to work:

1. Because USB1 high speed port is used as the USB device in MCU firmware, user should connect J2 with a USB cable to your Linux host and MCU is therefore powered on by the USB host VBUS, no other external power required;

2. Either LPC54S018-EVK or LPC546xx-EVK, user should force MCU to enter USB1 DFU mode firstly to be ready for firmware download by firstly pressing the key SW2(ISP-2) and SW3(ISP-1) in the same time(USB1 DFU mode -> ISP2:1:0 = 0:0:1) , secondly press-release SW1(MCU RESET) and the last step releasing SW2 and SW3. Now MCU should be at USB1 DFU mode;

3. Refer the above section "Download MCU firmware" to use dfu-util tool to program the firmware into MCU. 

   **Be noted that if you are using LPC54S018-EVK, the firmware is downloaded into MCU SRAM because LPC54018 has no internal flash, so during the test you should not press the Reset pin or not power down the MCU, otherwise the firmware will lost and you should start over again the above 3 steps, but if you are using LPC546xx-EVK, the firmware is downloaded into MCU flash, don't worry about that**. 

## 3. Setup<a name="step3"></a>

## Getting Started

### Add USB kernel driver, compile kernel

1. Copy file *kernel_driver/lpc54xxx.c* file to Linux kernel source directory *drivers/usb/serial/*
2. Add below code to Linux kernel source file *drivers/usb/serial/Makefile*:

`obj-$(CONFIG_USB_SERIAL_LPC54XXX)   += lpc54xxx.o`

3. Add below code to Linux kernel source file *drivers/usb/serial/Kconfig*:

```
config USB_SERIAL_LPC54XXX
     tristate "USB LPC54XXX USB Serial Driver"
     help
       Say Y here if you want to use a LPC54XXX 10 port
       USB to serial converter device.
       To compile this driver as a module, choose M here.

```

4. On Linux kernel source folder, run ‘make menuconfig’ and select above item (“Linux kernel source”) to ‘*’ (as built-in), then compile the Linux kernel by executing ‘make’ or 'make -j$(nproc)'.

### Download MCU firmware

Environment required for download tool (*LPC_firmware_download_tool/dfu-utils*):

* ARCH:  ARM

* OS:    Linux

* MCU Boards Support: 

  (1) LPC54018 multi-tty demo board

  (2) LPC54S018-EVK

  (3) LPC546xx-EVK(LPC54608, LPC54618 and LPC54628 are compatible on this EVK)

1. Copy the download folder *LPC_firmware_download_tool/YOUR PLATFORM/dfu-utils* to the directory of */usr/bin/*. Connect a USB cable between ARM platform USB host port and target LPC54018 multi-tty demo board’s USB upstream port. The firmware runs on MCU USB1 Port;

   **Note: if LPC54S018-EVK or LPC546xx-EVK is used instead of the multi-tty demo board, user should force MCU to enter USB DFU mode manually. Please refer the last section for details.**

2. On arm platform Linux console, goto the directory of *LPC_firmware_download_tool/*, execute below shell script to download firmware to LPC board automatically through USB cable:

   

```
root@ls1012afrwy:/LPC_firmware_download_tool# ./download.sh
```

Related log:

```
dfu-util 0.9

Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
Copyright 2010-2016 Tormod Volden and Stefan Schmidt
This program is Free Software and has ABSOLUTELY NO WARRANTY
Please report bugs to http://sourceforge.net/p/dfu-util/tickets/

dfu-util: Invalid DFU suffix signature
dfu-util: A valid DFU suffix will be required in a future dfu-util release!!!
Opening DFU capable USB device...
ID 1fc9:001f
Run-time device DFU version 0110
Claiming USB DFU Interface...
Setting Alternate Setting #0 ...
Determining device status: state = dfuIDLE, status = 0
dfuIDLE, continuing
DFU mode device DFU version 0110
Device returned transfer size 512
Copying data from PC to DFU device
Download        [=========================] 100%        62776 bytes
Download done.
state(8) = dfuMANIFEST-WAIT-RESET, status(0) = No error condition is present
Done!
************************************************************************************************
```

3. After firmware download complete, you will see upto 10 ttyUSBx device file shown in */dev/*, which mean they are ready for use:

```
root@ls1021atwr:/usr/bin/LPC_firmware_download_tool# ls /dev/ttyUSB*
```



Note: There is a user space program which can be used (on arm64 platform) for function test and demo, if you use 32bit platform, please recompile the source. please see folder *user_space_test_program/* for details.



### Debug

The device’s driver has added three attribute in sysfs to show the number of sending and receiving data (in bytes) and control the switch of loopback. The named “send_bytes” means the data ttyUSBx device has sent since usb serial device got enumerated. Similarly, the named “recv_bytes” means the data ttyUSBx device have received since usb serial device got enumerated.The "send_bytes" and "recv_bytes" only for read. The attribute of "loopback" can readed and writen , default value is 0(closed), if you want use loopback,execute command "echo 1 > loopback" ,otherwise "echo 0 > loopback".



Here is an example:

```
root@ls1043ardb:/mnt# ./usbttytestm 1 10 hello.c  recvhello 115200 1

root@ls1043ardb:/sys/class/tty/ttyUSB1/device# cat send_bytes
23755
root@ls1043ardb:/sys/class/tty/ttyUSB1/device# cat recv_bytes
23755
root@ls1043ardb:/sys/class/tty/ttyUSB1/device# cat loopback
1
root@ls1043ardb:/sys/class/tty/ttyUSB1/device# echo 0 > loopback
root@ls1043ardb:/sys/class/tty/ttyUSB1/device# cat loopback
0


```



### MCU Available UART Ports

| UART PORT                | PIN   | pin  mux |
| ------------------------ | ----- | -------- |
| UART0_TX(internal debug) | P0_30 | 1        |
| UART0_RX(internal debug) | P0_29 | 1        |
| UART0_CTS                | N/A   | N/A      |
| UART0_RTS                | N/A   | N/A      |
| UART1_TX                 | P1_11 | 2        |
| UART1_RX                 | P1_10 | 2        |
| UART1_CTS                | P0_13 | 1        |
| UART1_RTS                | P0_14 | 1        |
| UART2_TX                 | P1_25 | 1        |
| UART2_RX                 | P1_24 | 1        |
| UART2_CTS                | P1_26 | 1        |
| UART2_RTS                | P1_27 | 1        |
| UART3_TX                 | P0_2  | 1        |
| UART3_RX                 | P0_3  | 1        |
| UART3_CTS                | P0_1  | 2        |
| UART3_RTS                | P0_7  | 1        |
| UART4_TX                 | P3_27 | 3        |
| UART4_RX                 | P3_26 | 3        |
| UART4_CTS                | P3_28 | 3        |
| UART4_RTS                | P3_29 | 3        |
| UART5_TX                 | P2_13 | 5        |
| UART5_RX                 | P2_12 | 5        |
| UART5_CTS                | P2_14 | 5        |
| UART5_RTS                | P2_15 | 5        |
| UART6_TX                 | P4_3  | 2        |
| UART6_RX                 | P4_2  | 2        |
| UART6_CTS                | P4_0  | 2        |
| UART6_RTS                | N/A   | N/A      |
| UART7_TX                 | P2_20 | 3        |
| UART7_RX                 | P2_19 | 3        |
| UART7_CTS                | P1_21 | 1        |
| UART7_RTS                | P1_20 | 1        |
| UART8_TX                 | P3_17 | 1        |
| UART8_RX                 | P3_16 | 1        |
| UART8_CTS                | P3_4  | 3        |
| UART8_RTS                | P3_5  | 3        |
| UART9_TX                 | P3_3  | 2        |
| UART9_RX                 | P3_2  | 2        |
| UART9_CTS                | P3_30 | 1        |
| UART9_RTS                | P3_31 | 1        |

## 4. Results<a name="step4"></a>

### LS1012ardb or LS1012afrwy test guide and result

* 1 Program the IMG file

```
  wget wget https://www.nxp.com/lgfiles/sdk/lsdk2108/firmware_ls1012afrwy_qspiboot.img
  wget wget https://www.nxp.com/lgfiles/sdk/lsdk2108/firmware_ls1012ardb_qspiboot.img
  flash the img to QSPI flash with codewarrior jtag.
```

* 2 Create an SD card image
  Host linux:

```
  wget https://www.nxp.com/lgfiles/sdk/lsdk2108/flex-installer && chmod +x flex-installer && sudo mv flex-installer /usr/bin
  flex-installer -i auto -m ls1012ardb /dev/sdX
  or
  flex-installer -i auto -m ls1012afrwy /dev/sdX
  where X is a letter such as a, b, c. Make sure to choose the correct device name, because data on this device will be replaced.
```

* 3 Compile the kernel and copy it to SD
  follow the instructions above to generate the 'Image' and 'Image.gz' files, and then copy these two files to the '/dev/sdx1' directory on the SD card, i.e., the '/boot' partition.
* 4 UART stress testing
  Short Pin13(P3_27, FC4_TXD) and Pin15(P3_26, FC4_RXD) on J13 of the LPC54S108-evk board with a jumper wire as an external loopback. Pin13 and Pin15 of J13 correspond to the ttyUSB4 interface.
  Select any small file, such as 'Image.gz' (4.2M).
  Setting the baud rate to 2M using the command:

```
  stty -F /ttyUSB4 2000000 raw -echo
  cat /ttyUSB4 > Image_r.gz &
  echo Image.gz > /dev/ttyUSB4 &
```

  Wait until the file sizes match, then run the 'ls -al Image*' command. 
  If the MD5 checksums of the two files are the same, it indicates that the 2M baud rate is working properly.

## 5. FAQs<a name="step5"></a>

## 6. Support<a name="step6"></a>
#### Project Metadata
<!----- Boards ----->
[![Board badge](https://img.shields.io/badge/Board-LPCXPRESSO54S018-blue)](https://github.com/search?q=org%3Anxp-appcodehub+LPCXpresso54S018+in%3Areadme&type=Repositories)

<!----- Categories ----->
[![Category badge](https://img.shields.io/badge/Category-INDUSTRIAL-yellowgreen)](https://github.com/search?q=org%3Anxp-appcodehub+industrial+in%3Areadme&type=Repositories)

<!----- Peripherals ----->
[![Peripheral badge](https://img.shields.io/badge/Peripheral-DMA-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+dma+in%3Areadme&type=Repositories) [![Peripheral badge](https://img.shields.io/badge/Peripheral-UART-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+uart+in%3Areadme&type=Repositories)

<!----- Toolchains ----->
[![Toolchain badge](https://img.shields.io/badge/Toolchain-GCC-orange)](https://github.com/search?q=org%3Anxp-appcodehub+gcc+in%3Areadme&type=Repositories)

Questions regarding the content/correctness of this example can be entered as Issues within this GitHub repository.

>**Warning**: For more general technical questions regarding NXP Microcontrollers and the difference in expected funcionality, enter your questions on the [NXP Community Forum](https://community.nxp.com/)

[![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/@NXP_Semiconductors)
[![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/nxp-semiconductors)
[![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/nxpsemi/)
[![Follow us on Twitter](https://img.shields.io/badge/Twitter-Follow%20us%20on%20Twitter-white.svg)](https://twitter.com/NXP)

## 7. Release Notes<a name="step7"></a>
| Version | Description / Update                           | Date                        |
|:-------:|------------------------------------------------|----------------------------:|
| 1.0     | Initial release on Application Code HUb        | 六月 25<sup>th</sup> 2023 |

