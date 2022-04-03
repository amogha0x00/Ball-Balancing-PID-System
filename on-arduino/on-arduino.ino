/*
	Author: Amoghavarsha S G
*/

#include <Servo.h>

Servo xServo;
Servo yServo;

int xServoVal;
int yServoVal;
int cmd_len = 0;
char cmd[11];
int i;

void setup() {
	Serial.begin(115200);
	xServo.attach(5);
	yServo.attach(6);
}

void loop() {
	if (Serial.available() > 0){
		cmd_len = Serial.readBytesUntil('$', cmd, 10);
		cmd[cmd_len] = '\0';

		// find index of separator '#'
		for(i=0; cmd[i] != '#' && cmd[i] != '\0' && i<=cmd_len; i++);

		// make data at separator '#' as '\0'
		cmd[i] = '\0';
		// atoi only reads till first '\0' character
		xServoVal = atoi(cmd);
		// atoi reads from first '\0' secound '\0' which is at index `cmd_len`
		yServoVal = atoi(cmd + i + 1);

		// Serial.println(xServoVal);
		// Serial.println(yServoVal);

		if (xServoVal >= 500 && xServoVal <= 2500)
			xServo.writeMicroseconds(xServoVal);
		if (yServoVal >= 500 && yServoVal <= 2500)
			yServo.writeMicroseconds(yServoVal);
	}

}
