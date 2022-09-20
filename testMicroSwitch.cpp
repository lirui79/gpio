#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/types.h>  
#include <sys/time.h>
#include <signal.h>
#include <errno.h>
#include <string.h>  

#include "MicroSwitch.h"

int testMicroSwitch(int argc, const char *argv[]) {
	if (argc < 1) {
		printf("input ./test_buzzer gpin\n");
		printf("input ./test_buzzer 3 \n");
		return -1;
	}
	int gpin = atoi(argv[1]);
        MicroSwitch  msw;
        bool status = false;
	int code = msw.readBoxStatus(gpin, status);
	printf("gpin:%d status:%d retcode:%d \n", gpin, status, code);
	return 0;
}

static void button_click() {
	static int state = 0;
	state = (state + 1) % 2;
	printf("state:%d\n", state);
}

int test2(int argc, const char *argv[]) {
	MicroSwitch  msw;
	int code = msw.init();
	int count = 0;
	printf("init:%d\n", code);
	msw.registerScreenButtonFunc(button_click);
	while(1) {
		printf("continue %d \n" , count);
		++count;
		if (count > 200)
			break;
		sleep(1);
	}
    msw.exit();
    return 0;
}

int main(int argc, const char *argv[]){

	return test2(argc, argv);

	//return testMicroSwitch(argc, argv);
}
