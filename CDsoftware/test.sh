#!/bin/bash
while true
do
    /root/Downloads/stlink-1.4.0/st-flash write main.bin 0x08000000
done
