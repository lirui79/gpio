all: testMicroSwitch



CROSS_COMPILE   = arm-linux-gnueabihf-
#CROSS_COMPILE   = arm-none-eabi-
GXX		= $(CROSS_COMPILE)g++
#GXX             = g++

testMicroSwitch: MicroSwitch.cpp testMicroSwitch.cpp
	$(GXX) MicroSwitch.cpp testMicroSwitch.cpp -lpthread -o testMicroSwitch

liblocation.so: location.c
	$(GCC) location.c  gsm_usb.c xml.c -fPIC -shared -L./ -o liblocation.so

test_location: location.c test_location.c
	$(GCC) test_location.c -L. -llocation -o test_location

clean:
	rm  testMicroSwitch
