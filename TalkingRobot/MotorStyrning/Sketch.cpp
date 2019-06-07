/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
using namespace std;
/*End of auto generated code by Atmel studio */
#include <AFMotor.h>
#include <Math.h>
#include <Servo.h>
#include "SD.h"
#include "TMRpcm.h"
#include "SPI.h"
//#include <SoftwareSerial.h>

// SD card pin
#define SD_ChipSelectPin 53
#define SD_ClockPin 52

// RF05 Ultrasonic Sensor
#define TRIG 37 // the SRF05 Trig pin
#define ECHO 39 // the SRF05 Echo pin

// Number of degrees for each step
#define SENSOR_STEP 10

// Speed
#define SPEED_STOP 0
#define SPEED_MEDIUM 100
#define SPEED_FAST 200
#define SPEED_TURN 150

// Distance
#define DISTANCE_TO_OBJECT 30

// IR Distance Sensor
int IRsensorFront = A15;
int IRsensorBack = A14;

// Bluetooth module Variables
//SoftwareSerial btSerialPort(15, 14); // RX, TX

enum Direction
{
	Stop,
	Forward,
	ForwardLeft,
	ForwardRight,
	Backward,
	BackwardLeft,
	BackwardRight,
	Left,
	Right,
	DIRECTION_NUMBER_OF_ELEMENTS
};

enum Motor
{
	FrontLeft,
	FrontRight,
	BackLeft,
	BackRight,
	All,
	MOTOR_NUMBER_OF_ELEMENTS
};

// Beginning of Auto generated function prototypes by Atmel Studio
int     FindTheNearestObstacle();
int     GetMinDistanceFromArray();
int     ReadIRsensorDistanceInCm(int irSensor);
void    TurnRobotToObstacle(int obstaclePosition);
void    ResetTurningCounter();
void    Speed(Motor motor, int newSpeed);
void    Move(Direction robotMovement);
void    PlaySoundFile(char charBuf[8]);
void    SaySomethingRandom();
void    BluetoothControl(int distanceIrSensorFront, int distanceIrSensorBack);
void    Autonomous(int distanceIrSensorFront, int distanceIrSensorBack);
float   MeasureDistanceWithUltrasonicInCm(bool takeAverage);
boolean CheckDataNeighbourhood(int arrayIndex);
// End of Auto generated function prototypes by Atmel Studio
char receivedCharacter;
bool newCharacterRecieved;

char command = 'S';
char prevCommand = 'A';
//Stores the time (in millis since execution started)
unsigned long timeCurrentCommand = 2000;
//Stores the time when the last command was received from the phone
unsigned long timePreviousCommand = 0;

// Variables
const int     minFileName = 1; // meter/seconds
const int     maxFileName = 22; // meter/seconds
const int     speedOfSound = 340; // meter/seconds
int           numberOfSteps = int(180 / SENSOR_STEP);
int           turningCounter = 0;
int           startTime = 0;
int           speed = 100;
int           distanceIRsensor;
int           lastTimeISpoke = 0;
int           fileNameIndex;
int           fileNameRequestIndex;
int           timeLastRequest = 0;
bool          playFileRequestRecieved = false;
bool          playFileRequest = false;
bool          moveLeft = false;
bool          runAutonomous = false;
const bool    debugMessageOn = false;
TMRpcm        tmrpcm;
Direction     RobotMovementDirection = Stop;
unsigned long srfDistanceArray[180 / SENSOR_STEP];
unsigned long minimumDistanceThreshold = 10;
unsigned long maximumDistanceThreshold = 400;

// Servo for Distance Measurements
Servo servoObjectFinderAndDistance;
// DC motor on M1
AF_DCMotor FrontLeftMotor(1);
// DC motor on M2
AF_DCMotor FrontRightMotor(2);
// DC motor on M3
AF_DCMotor BackLeftMotor(3);
// DC motor on M4 -
AF_DCMotor BackRightMotor(4);

void setup()
{
	// Serial Port
	Serial.begin(9600);
	Serial3.begin(9600);

	// turn on motor #1
	FrontLeftMotor.setSpeed(speed);
	FrontLeftMotor.run(RELEASE);

	// turn on motor #2
	FrontRightMotor.setSpeed(speed);
	FrontRightMotor.run(RELEASE);

	// turn on motor #3
	BackLeftMotor.setSpeed(speed);
	BackLeftMotor.run(RELEASE);

	// turn on motor #4
	BackRightMotor.setSpeed(speed);
	BackRightMotor.run(RELEASE);
	
	// Define Servo
	servoObjectFinderAndDistance.attach(9);
	delay(300);
	//servoObjectFinderAndDistance.write(0);
	//delay(1500);
	servoObjectFinderAndDistance.write(90);

	// Ultrasonic Sensor
	pinMode(TRIG, OUTPUT);
	pinMode(ECHO, INPUT);
	
	// Speaker.
	// 5,6,11 or 46 on Mega, 9 on Uno, Nano, etc
	tmrpcm.setVolume(4);
	tmrpcm.speakerPin = 46;
	
	// SD card
	// make sure that the default chip select pin is set to
	// output, even if you don't use it:
	pinMode(53, OUTPUT);
	if (!SD.begin(SD_ChipSelectPin))
	{
		Serial.println("SD fail");
	}
	else
	{
		//Just for a second verification. Make sure there is a file with the name 1.wav
		File myfile = SD.open("1.wav", FILE_READ);
		if (myfile)
		{
			Serial.println("File Opened");
			myfile.close();
		}
		else
		{
			Serial.println("FAILED Open File");
		}
	}
	
	lastTimeISpoke = millis();
	fileNameIndex = minFileName;
}

void loop()
{
	int distanceIrSensorFront = ReadIRsensorDistanceInCm(IRsensorFront);
	int distanceIrSensorBack = ReadIRsensorDistanceInCm(IRsensorBack);
	if(debugMessageOn)
	{
		Serial.print("distanceIrSensorFront: ");
		Serial.println(distanceIrSensorFront);
		Serial.print("distanceIrSensorBack: ");
		Serial.println(distanceIrSensorBack);
	}

	if (newCharacterRecieved)
	{
		switch (receivedCharacter)
		{
			// App: Front lights ON
			// Implementation: Autonomous ON
			case 'W':
			runAutonomous = true;
			timePreviousCommand = millis();
			prevCommand = command;
			command = receivedCharacter;
			newCharacterRecieved = false;
			Move(Stop);
			break;
			
			// App: Front Lights OFF
			// Implementation: Autonomous OFF
			case 'w':
			runAutonomous = false;
			timePreviousCommand = millis();
			prevCommand = command;
			command = receivedCharacter;
			newCharacterRecieved = false;
			Move(Stop);
			break;
		}
	}

	if(runAutonomous)
	{
		Autonomous(distanceIrSensorFront, distanceIrSensorBack);
	}
	else
	{
		BluetoothControl(distanceIrSensorFront, distanceIrSensorBack);
	}
}

void BluetoothControl(int distanceIrSensorFront, int distanceIrSensorBack)
{
	if (newCharacterRecieved)
	{
		newCharacterRecieved = false;
		timePreviousCommand = millis();
		prevCommand = command;
		command = receivedCharacter;
		
		//Change pin mode only if new command is different from previous.
		if (command != prevCommand)
		{
			Serial.println(command);
			switch (command)
			{
				// Forward
				case 'F':
				if(!runAutonomous && RobotMovementDirection != Forward && distanceIrSensorFront > DISTANCE_TO_OBJECT)
				{
					Speed(All, speed);
					Move(Forward);
				}

				if(distanceIrSensorFront <= DISTANCE_TO_OBJECT)
				{
					Move(Backward);
					delay(100);
					Move(Stop);
					Serial.print("distanceIrSensorFront: ");
					Serial.println(distanceIrSensorFront);
					Serial.println("Item too close will not drive forward");
				}
				break;
				
				// Backward
				case 'B':
				if(!runAutonomous && RobotMovementDirection != Backward && distanceIrSensorBack > DISTANCE_TO_OBJECT)
				{
					Speed(All, speed);
					Move(Backward);
				}

				if(distanceIrSensorBack <= DISTANCE_TO_OBJECT)
				{
					Move(Forward);
					delay(100);
					Move(Stop);
					Serial.print("distanceIrSensorBack: ");
					Serial.println(distanceIrSensorBack);
					Serial.println("Item too close will not drive backward");
				}
				break;

				// Left
				case 'L':
				if(!runAutonomous && RobotMovementDirection != Left )
				{
					Speed(All, speed);
					Move(Left);
				}
				
				break;
				
				// Right
				case 'R':
				if(!runAutonomous && RobotMovementDirection != Right)
				{
					Speed(All, speed);
					Move(Right);
				}
				
				break;
				
				// Stop
				case 'S':
				if(!runAutonomous && RobotMovementDirection != Stop)
				{
					Speed(All, speed);
					Move(Stop);
				}
				
				break;
				
				// Forward Right
				case 'I':
				if(!runAutonomous && RobotMovementDirection != ForwardRight && distanceIrSensorFront > DISTANCE_TO_OBJECT)
				{
					Speed(All, speed);
					Move(ForwardRight);
					//Serial.println("Forward Right");
				}
				if(distanceIrSensorFront <= DISTANCE_TO_OBJECT)
				{
					Move(Backward);
					delay(100);
					Move(Stop);
					Serial.print("distanceIrSensorFront: ");
					Serial.println(distanceIrSensorFront);
					Serial.println("Item too close will not drive forward");
				}
				break;
				
				// Backward Right
				case 'J':
				if(!runAutonomous && RobotMovementDirection != BackwardRight && distanceIrSensorBack > DISTANCE_TO_OBJECT)
				{
					Speed(All, speed);
					Move(BackwardRight);
					//Serial.println("Backward Right");
				}
				if(distanceIrSensorBack <= DISTANCE_TO_OBJECT)
				{
					Move(Forward);
					delay(100);
					Move(Stop);
					Serial.print("distanceIrSensorBack: ");
					Serial.println(distanceIrSensorBack);
					Serial.println("Item too close will not drive backward");
				}
				
				break;
				
				// Forward Left
				case 'G':
				if(!runAutonomous && RobotMovementDirection != ForwardLeft && distanceIrSensorFront > DISTANCE_TO_OBJECT)
				{
					Speed(All, speed);
					Move(ForwardLeft);
					//Serial.println("Forward Left");
				}
				if(distanceIrSensorFront <= DISTANCE_TO_OBJECT)
				{
					Move(Backward);
					delay(100);
					Move(Stop);
					Serial.print("distanceIrSensorFront: ");
					Serial.println(distanceIrSensorFront);
					Serial.println("Item too close will not drive forward");
				}
				break;
				
				// Backwards Left
				case 'H':
				if(!runAutonomous && RobotMovementDirection != ForwardRight && distanceIrSensorBack > DISTANCE_TO_OBJECT)
				{
					Speed(All, speed);
					Move(BackwardLeft);
					//Serial.println("Backwards Left");
				}
				if(distanceIrSensorBack <= DISTANCE_TO_OBJECT)
				{
					Move(Forward);
					delay(100);
					Move(Stop);
					Serial.print("distanceIrSensorBack: ");
					Serial.println(distanceIrSensorBack);
					Serial.println("Item too close will not drive backward");
				}
				break;
				
				// App: Front lights ON
				// Implementation: Autonomous ON
				case 'W':
				runAutonomous = true;
				break;
				
				// App: Front Lights OFF
				// Implementation: Autonomous OFF
				case 'w':
				runAutonomous = false;
				break;

				// App: Warning Triangle OFF
				// Implementation: Will play a fill base on number of times the user clicks on warning triangle
				case 'X':
				if(!runAutonomous)
				{
					if(!playFileRequestRecieved)
					{
						timeLastRequest = millis();
						playFileRequestRecieved = true;
						fileNameRequestIndex = minFileName;
					}
					else
					{
						if(playFileRequestRecieved)
						{
							timeLastRequest = millis();
							fileNameRequestIndex++;
						}
					}

					if(fileNameIndex > maxFileName)
					{
						fileNameIndex=minFileName;
					}

					Serial.print("fileNameRequestIndex: ");
					Serial.println(fileNameRequestIndex);
				}
				break;
				
				// App: Warning Triangle OFF
				// Implementation: Will play a fill base on number of times the user clicks on warning triangle
				case 'x':
				if(!runAutonomous)
				{
					if(!playFileRequestRecieved)
					{
						timeLastRequest = millis();
						playFileRequestRecieved = true;
						fileNameRequestIndex = minFileName;
					}
					else
					{
						if(playFileRequestRecieved)
						{
							timeLastRequest = millis();
							fileNameRequestIndex++;
						}
					}

					if(fileNameRequestIndex > maxFileName)
					{
						fileNameRequestIndex=minFileName;
					}

					Serial.print("fileNameRequestIndex: ");
					Serial.println(fileNameRequestIndex);
				}
				break;
				
				// App: Back lights ON
				// Implementation: Plays a sound files in sequence
				case 'U':
				if(!runAutonomous)
				{
					char charBuf[12];
					sprintf(charBuf, "%d.wav", fileNameIndex++);
					Serial.println(charBuf);
					PlaySoundFile(charBuf);

					if(fileNameIndex > maxFileName)
					{
						fileNameIndex=minFileName;
					}
				}
				
				break;
				
				// App: Back Lights Off
				// Implementation: Plays a sound files in sequence
				case 'u':
				if(!runAutonomous)
				{
					char charBuf[12];
					sprintf(charBuf, "%d.wav", fileNameIndex++);
					Serial.println(charBuf);
					PlaySoundFile(charBuf);

					if(fileNameIndex > maxFileName)
					{
						fileNameIndex=minFileName;
					}
				}
				
				break;

				// App: Horn ON
				// Implementation: Play Sound in Speaker
				case 'V':
				if(!runAutonomous)
				{
					SaySomethingRandom();
				}
				
				break;

				// App: Horn OFF
				// Implementation: Play Sound in Speaker
				case 'v':
				if(!runAutonomous)
				{
					SaySomethingRandom();
				}
				
				break;
				
				// App: Disconnect from BT Module. Everything OFF
				// Implementation: Run Autonomous
				case 'D':
				runAutonomous = true;
				break;
				
				// Get velocity
				default:
				if (command == 'q')
				{
					// Full velocity
					//Serial.println("Velocity Max");
					speed = 255;
				}
				else
				{
					// Chars '0' - '9' have an integer equivalence of 48 - 57, accordingly.
					if ((command >= 48) && (command <= 57))
					{
						// Subtracting 48 changes the range from 48-57 to 0-9.
						// Multiplying by 25 changes the range from 0-9 to 0-225.
						speed = (command - 48) * 25;
						
						if(debugMessageOn)
						{
							Serial.print("Speed: ");
							Serial.println(speed);
						}
					}
				}
			}
		}
		else
		{
			// Get the current time (millis since execution started).
			timeCurrentCommand = millis();
			
			// Check if it has been 500ms since we received last command.
			if ((timeCurrentCommand - timePreviousCommand) > 500 && !runAutonomous)
			{
				if (RobotMovementDirection != Stop)
				{
					// More than 500ms have passed since last command received, car is out of range.
					// Therefore stop the car and turn lights off.
					//Serial.println("Turn off lights and stop car");
					Move(Stop);
				}
			}
		}

	}

	if(playFileRequestRecieved)
	{
		int timeSinceLastRequest = millis() - timeLastRequest;
		if(timeSinceLastRequest > 750)
		{
			char charBuf[12];
			sprintf(charBuf, "%d.wav", fileNameRequestIndex);
			Serial.println(charBuf);
			PlaySoundFile(charBuf);
			fileNameRequestIndex = minFileName;
			playFileRequestRecieved = false;
		}
	}
}

void Autonomous(int distanceIrSensorFront, int distanceIrSensorBack)
{
	float distanceUltraSonicFront = MeasureDistanceWithUltrasonicInCm(true);

	//Serial.print("distanceUltraSonicFront ");
	//Serial.println(distanceUltraSonicFront);

	if (distanceUltraSonicFront <= 10 || distanceIrSensorFront <= 10)
	{
		if (speed != SPEED_FAST || RobotMovementDirection != Backward)
		{
			speed = SPEED_FAST;

			if( distanceIrSensorBack < 15 )
			{
				Speed(All, speed);
				Move(Left);
				delay(500);
			}

			Speed(All, speed);
			Move(Backward);
			Serial.println("Back Fast from Target");
		}
	}
	else if ((distanceUltraSonicFront <= 20 || distanceIrSensorFront <= 20) && (distanceUltraSonicFront > 10 || distanceIrSensorFront > 10))
	{
		if (speed != SPEED_MEDIUM || RobotMovementDirection != Backward)
		{
			speed = SPEED_MEDIUM;
			if( distanceIrSensorBack < 15 )
			{
				Speed(All, speed);
				Move(Left);
				delay(500);
			}

			Speed(All, speed);
			Move(Backward);
			startTime = millis();
			Serial.println("Back Slow from target");
		}
	}
	else if ((distanceUltraSonicFront <= 40 || distanceIrSensorFront <= 40) && (distanceUltraSonicFront > 20 || distanceIrSensorFront > 20))
	{
		if (speed != SPEED_STOP || RobotMovementDirection != Stop)
		{
			speed = SPEED_STOP;
			Move(Stop);
			delay(50);
			Serial.println("Stop at target");
		}
	}
	else if ((distanceUltraSonicFront <= 60 || distanceIrSensorFront <= 60) && (distanceUltraSonicFront > 40 || distanceIrSensorFront > 40))
	{
		if (speed != SPEED_MEDIUM || RobotMovementDirection != Forward)
		{
			speed = SPEED_MEDIUM;
			Speed(All, speed);
			Move(Forward);
			ResetTurningCounter();
			Serial.println("Move Slow to Target");
		}
	}
	else if (distanceUltraSonicFront <= 350 && distanceUltraSonicFront > 60)
	{
		if (speed != SPEED_FAST || RobotMovementDirection != Forward)
		{
			speed = SPEED_FAST;
			Speed(All, speed);
			Move(Forward);
			ResetTurningCounter();
			Serial.println("Move Fast to target");
		}
	}
	else if (distanceUltraSonicFront > 300 && RobotMovementDirection != Left && RobotMovementDirection != Right)
	{
		Serial.println("Find Target");
		Move(Stop);
		int nearestObstacleAngle = FindTheNearestObstacle();
		if (nearestObstacleAngle != -1)
		{
			TurnRobotToObstacle(nearestObstacleAngle);
		}
		else
		{
			// No obstacle found continue driving forward
			speed = SPEED_FAST;
			Speed(All, speed);
			Move(Forward);
			delay(1000);
		}

		// Look straight forward with the ultrasonic sensor
		servoObjectFinderAndDistance.write(90);
		delay(300);
	}

	if (RobotMovementDirection == Stop)
	{
		int timeStandingStill = millis() - startTime;

		if (timeStandingStill > 500 && timeStandingStill < 1000)
		{
			int timeSinceILastSpoke = millis() - lastTimeISpoke;
			
			if(timeSinceILastSpoke > 3000)
			{
				SaySomethingRandom();
				lastTimeISpoke = millis();
			}
		}
		else if (timeStandingStill > 2000)
		{
			Speed(All, SPEED_TURN);

			
			if (moveLeft)
			{
				Move(Left);
				moveLeft = false;
			}
			else
			{
				Move(Right);
				moveLeft = true;
			}

			delay(random(500, 1750));
			Move(Stop);
		}
	}
}

void ResetTurningCounter()
{
	turningCounter = 0;
}

//////////////////////////////////////////////////////////////////////////
// Finds the closest obstacle by scanning the surrounding with the ultra
// sound sensor.
//
// Returns: the angle where the obstacle is located. If nothing can be
// found with the parameter maximumDistanceThreshold it will return -1
//////////////////////////////////////////////////////////////////////////
int FindTheNearestObstacle()
{
	int nearestObstaclePosition;
	unsigned long nearestObstacleDistance;

	for (int position = 0; position < numberOfSteps; position++)
	{
		// Tell servo to go to 180 degree, with step size SENSOR_STEP
		// Move to next position
		servoObjectFinderAndDistance.write(position * SENSOR_STEP);

		if (position == 0)
		{
			// Pause to get it time to move to first position
			delay(1000);
		}

		// Short pause to allow it to move, min 20ms
		delay(31);
		srfDistanceArray[position] = MeasureDistanceWithUltrasonicInCm(false);

		Serial.println("Angle: " + String(position*SENSOR_STEP) + ", Distance: " + String(srfDistanceArray[position]) + " cm");
	}

	nearestObstaclePosition = GetMinDistanceFromArray();
	nearestObstacleDistance = srfDistanceArray[nearestObstaclePosition];
	Serial.println("nearestObstaclePosition: " + String(nearestObstaclePosition * SENSOR_STEP) + " degrees, " + String(nearestObstacleDistance) +" cm");
	if (nearestObstacleDistance < maximumDistanceThreshold)
	{
		// Tell servo to go to nearest obstacle position
		servoObjectFinderAndDistance.write(nearestObstaclePosition * SENSOR_STEP);

		// Pause to get it time to move
		delay(1500);
		return nearestObstaclePosition * SENSOR_STEP;
	}

	return -1;
}

//KOLLA DENNA
boolean CheckDataNeighbourhood(int arrayIndex)
{
	unsigned long lowerNeighbourDiff = abs(srfDistanceArray[arrayIndex] - srfDistanceArray[arrayIndex - 1]);
	unsigned long upperNeighbourDiff = abs(srfDistanceArray[arrayIndex] - srfDistanceArray[arrayIndex + 1]);
	if (lowerNeighbourDiff < 5 || upperNeighbourDiff < 5)
	{
		return true;
	}

	return false;
}

//////////////////////////////////////////////////////////////////////////
/// Get minimum value in the array
/// <returns> index for the minimum value </returns>
//////////////////////////////////////////////////////////////////////////
int GetMinDistanceFromArray()
{
	unsigned long minDistance = srfDistanceArray[0];
	int index = 0;
	for (int i = 1; i < numberOfSteps - 1; i++)
	{
		if (srfDistanceArray[i] > minimumDistanceThreshold && srfDistanceArray[i] < minDistance)
		{
			// if (CheckNeighborhood(i))
			//    {
			index = i;
			minDistance = srfDistanceArray[i];
			//  }
		}
	}

	return index;
}

void TurnRobotToObstacle(int obstacleAngle)
{
	int delayTime;
	
	if (turningCounter < 10 )
	{
		Speed(All, 200);
		if (obstacleAngle >= 0 && obstacleAngle <= 45)
		{
			Serial.println("Turn to target. Right 0-45 deg.");
			Move(Forward);
			delay(100);
			Move(Right);
			delayTime = 500;
		}
		else if (obstacleAngle > 45 && obstacleAngle < 90)
		{
			Serial.println("Turn to target. Right 45-89 deg.");
			Move(Forward);
			delay(100);
			Move(Right);
			delayTime = 1000;
		}
		else if (obstacleAngle > 90 && obstacleAngle <= 135)
		{
			Serial.println("Turn to target. Left 91-135 deg.");
			Move(Forward);
			delay(100);
			Move(Left);
			delayTime = 500;
		}
		else if (obstacleAngle > 135 && obstacleAngle <= 180)
		{
			Serial.println("Turn to target. Left 135-180 deg.");
			Move(Forward);
			delay(100);
			Move(Left);
			delayTime = 1000;
		}

		turningCounter++;
	}
	else
	{
		Move(Stop);
	}
	
	delay(delayTime);
	Move(Stop);
}

//*********************************************************************************************************
// Calculates the distance to an obstacle in front of the ultra sonic sensor.
// Returns the distance in centimeters.
//*********************************************************************************************************
float MeasureDistanceWithUltrasonicInCm(bool takeAverage)
{
	float measurement;
	int numberOfMeasurements = 1;
	unsigned long pulseTimeMicroSecMeasurement = 0;
	
	if(takeAverage)
	{
		numberOfMeasurements = 3;
	}
	
	for(int i = 0; i < numberOfMeasurements; i++)
	{
		digitalWrite(TRIG, LOW);
		delayMicroseconds(2);

		digitalWrite(TRIG, HIGH);
		delayMicroseconds(10);
		digitalWrite(TRIG, LOW);

		// wait for the pulse to return. The pulse goes from low to HIGH to low, so we specify
		// that we want a HIGH-going pulse below: result in micro seconds
		pulseTimeMicroSecMeasurement += pulseIn(ECHO, HIGH);
	}
	
	// Step by step calculations replaced by one step.
	// unsigned long pulseTimeSeconds = pulseTimeMicroSeconds / 1000000;
	// float distanceInMeters = pulseTimeSeconds * speedOfSound / 2;
	// return distanceInMeters * 100;
	pulseTimeMicroSecMeasurement /= numberOfMeasurements;
	return pulseTimeMicroSecMeasurement / 58.82352941176471;
}

int ReadIRsensorDistanceInCm(int irSensor)
{
	int distanceIRsensorFront = analogRead(irSensor);
	float voltage = distanceIRsensorFront * (5.0 / 1023.0);
	if (distanceIRsensorFront > 17)
	{
		// https://home.roboticlab.eu/en/examples/sensor/ir_distance
		int distanceCm = 5461 / (distanceIRsensorFront - 17) - 2;
		return distanceCm;
	}
	return 5000;
}

void Speed(Motor motor, int newSpeed)
{
	switch (motor)
	{
		case FrontLeft:
		FrontLeftMotor.setSpeed(newSpeed);
		FrontLeftMotor.run(RELEASE);
		break;

		case FrontRight:
		FrontRightMotor.setSpeed(newSpeed);
		FrontRightMotor.run(RELEASE);
		break;

		case BackLeft:
		BackLeftMotor.setSpeed(newSpeed);
		BackLeftMotor.run(RELEASE);
		break;

		case BackRight:
		BackRightMotor.setSpeed(newSpeed);
		BackRightMotor.run(RELEASE);
		break;

		case All:
		FrontLeftMotor.setSpeed(newSpeed);
		FrontLeftMotor.run(RELEASE);
		FrontRightMotor.setSpeed(newSpeed);
		FrontRightMotor.run(RELEASE);
		BackLeftMotor.setSpeed(newSpeed);
		BackLeftMotor.run(RELEASE);
		BackRightMotor.setSpeed(newSpeed);
		BackRightMotor.run(RELEASE);
		break;
	}
}

void Move(Direction robotMovement)
{
	int	turnspeed = speed * 0.3;

	switch (robotMovement)
	{
		case Stop:
		FrontLeftMotor.run(RELEASE);
		FrontRightMotor.run(RELEASE);
		BackLeftMotor.run(RELEASE);
		BackRightMotor.run(RELEASE);
		RobotMovementDirection = Stop;
		break;

		case Forward:
		FrontLeftMotor.run(FORWARD);
		FrontRightMotor.run(FORWARD);
		BackLeftMotor.run(FORWARD);
		BackRightMotor.run(FORWARD);
		RobotMovementDirection = Forward;
		break;

		case ForwardLeft:
		Speed(FrontLeft, turnspeed);
		FrontLeftMotor.run(FORWARD);
		Speed(BackLeft, turnspeed);
		BackLeftMotor.run(FORWARD);
		FrontRightMotor.run(FORWARD);
		BackRightMotor.run(FORWARD);
		RobotMovementDirection = ForwardLeft;
		break;

		case ForwardRight:
		FrontLeftMotor.run(FORWARD);
		BackLeftMotor.run(FORWARD);
		Speed(FrontRight, turnspeed);
		FrontRightMotor.run(FORWARD);
		Speed(BackRight, turnspeed);
		BackRightMotor.run(FORWARD);
		RobotMovementDirection = ForwardRight;
		break;

		case Backward:
		FrontLeftMotor.run(BACKWARD);
		FrontRightMotor.run(BACKWARD);
		BackLeftMotor.run(BACKWARD);
		BackRightMotor.run(BACKWARD);
		RobotMovementDirection = Backward;
		break;

		case BackwardLeft:
		Speed(FrontLeft, turnspeed);
		FrontLeftMotor.run(BACKWARD);
		Speed(BackLeft, turnspeed);
		BackLeftMotor.run(BACKWARD);
		FrontRightMotor.run(BACKWARD);
		BackRightMotor.run(BACKWARD);
		RobotMovementDirection = BackwardLeft;
		break;

		case BackwardRight:
		FrontLeftMotor.run(BACKWARD);
		BackLeftMotor.run(BACKWARD);
		Speed(FrontRight, turnspeed);
		FrontRightMotor.run(BACKWARD);
		Speed(BackRight, turnspeed);
		BackRightMotor.run(BACKWARD);
		RobotMovementDirection = BackwardRight;
		break;

		case Left:
		Serial.println("Turning left");
		FrontLeftMotor.run(BACKWARD);
		BackLeftMotor.run(BACKWARD);
		FrontRightMotor.run(FORWARD);
		BackRightMotor.run(FORWARD);
		RobotMovementDirection = Left;
		break;

		case Right:
		Serial.println("Turning right");
		FrontLeftMotor.run(FORWARD);
		BackLeftMotor.run(FORWARD);
		FrontRightMotor.run(BACKWARD);
		BackRightMotor.run(BACKWARD);
		RobotMovementDirection = Right;
		break;

		default:
		Serial.println("Default");
		FrontLeftMotor.run(RELEASE);
		FrontRightMotor.run(RELEASE);
		BackLeftMotor.run(RELEASE);
		BackRightMotor.run(RELEASE);
		RobotMovementDirection = Stop;
		break;
	}

	startTime = millis();
}

//////////////////////////////////////////////////////////////////////////
// Plays a random file within the range of 1 to  22
//
//////////////////////////////////////////////////////////////////////////
void SaySomethingRandom()
{
	int file = random(minFileName, maxFileName);
	char charBuf[12];
	sprintf(charBuf, "%d.wav", file);
	PlaySoundFile(charBuf);
}

//////////////////////////////////////////////////////////////////////////
// Plays a file from a SD card connected to the Arduino
//
//////////////////////////////////////////////////////////////////////////
void PlaySoundFile(char charBuf[12])
{
	Serial.println(charBuf);
	tmrpcm.play(charBuf);
}

void serialEvent3()
{
	if (Serial3.available() > 0)
	{
		receivedCharacter = Serial3.read();
		newCharacterRecieved = true;
	}
}