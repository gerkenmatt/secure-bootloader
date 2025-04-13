#!/bin/bash
sudo openocd -f interface/stlink.cfg -f target/stm32f7x.cfg -c "program blinky.elf verify reset exit"

