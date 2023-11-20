* lpc54018_uart_server: RAM based image,  running on LPC54018, the image will lost if re-power or press RESET pin.
* lpc54608_uart_server: flash based image, running on LPC54608. the image will NOT lost if re-power or press RESET pin.

Those two images will not be provided in this repository due to licensing issues. You can download them from here:

* https://github.com/NXP-MCU-X-Lab/lpc_uart_server/tree/master



Please update the 'download.sh' file based on the USB ID obtained from the 'lsusb' command on the host machine (Linux) for the MCU in DFU mode. 
Modify it according to the actual situation.

