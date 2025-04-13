
# To Load onto Board
sudo openocd -f interface/stlink.cfg -f target/stm32f7x.cfg -c "program blinky.elf verify reset exit"

# Fixes I needed to make
- sudo ln -s /lib/x86_64-linux-gnu/libncurses.so.6 /lib/x86_64-linux-gnu/libncurses.so.5
- sudo ln -s /lib/x86_64-linux-gnu/libtinfo.so.6 /lib/x86_64-linux-gnu/libtinfo.so.5

# VSCode Setup
## Debugging
    - NOTE: Don't run openocd before calling Start Debugging (F5). All I need to do is just click Start Debugging (F5) with no instance of openocd running yet. Check if there is an instance with "ps aux | grep ocd" 
    
