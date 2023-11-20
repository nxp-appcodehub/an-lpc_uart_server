
# SPDX-License-Identifier: GPL-2.0+
#
#  LPC USB-10 ports Serial Converters
#
#  Copyright 2018 NXP
#  All rights reserved.
#
#  Shamelessly based on DIGI driver
#
#
#

sudo dfu-util -d 1fc9:001f -D lpc54018_uart_server.bin
sleep 1
echo "1s"
sleep 1
echo "2s"
sleep 1
echo "3s"
sleep 1
echo "4s"
sleep 1
echo "5s"
ls /dev/ttyU*
sleep 1
sudo chmod -R 777 /dev/ttyU*

