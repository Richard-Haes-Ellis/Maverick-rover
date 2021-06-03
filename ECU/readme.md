Tool to generate code and cofigure peripherals

    Use the STM32CubeMX program and generate code with the makefile toolchain

    [Images of the process]

Tool for building and flashing 

    sudo apt install stlink-tools

For building

    cd ~/project_folder
    make

To flash:

    cd ~/project_folder
    st-flash write build/firmware.bin 0x08000000



