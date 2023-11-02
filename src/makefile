help:
	@echo 'Help details:'
	@echo 'hex: compile hex file'
	@echo 'flash: install hex file'
	@echo 'default: compile hex and install'

default: hex flash	
	
hex:
	avr-gcc -Os -mmcu=atmega32 -c main.c
	avr-gcc -mmcu=atmega32 -o main.elf main.o
	avr-objcopy -O ihex main.elf main.hex
	del main.o
	del main.elf
	
flash:
	avrdude -c usbasp-clone -p atmega32 -B 93.75 -U flash:w:main.hex
	
