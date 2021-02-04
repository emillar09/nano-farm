
.PHONY : node

node : endNode/endNode.ino
	arduino-cli compile --fqbn Seeeduino:avr:V4 endNode/ ;\
	arduino-cli upload -p /dev/ttyS5 --fqbn Seeeduino:avr:V4 endNode/
