#include "WPILib.h"
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <Latch.h>
#include <iostream>




class Robot: public IterativeRobot
{
	RobotDrive *myDrive;
	VictorSP *frontLeft, *frontRight, *rearLeft, *rearRight, launchAngle, climbMotor, climbLock;
	TalonSRX launchLeft, launchRight;
	Servo launchLoader;
	Joystick driverControl, shooterControl;
	AnalogInput pot; // obj for the potentiometer
	DigitalInput tLimit, bLimit, decswitch0, decswitch1;
	ADXRS450_Gyro gyro;
	Latch revlatch;
	Timer robotimer;
	Latch ramplatch;


	float currleft = 0.0, currright = 0.0;
	const float SPEED_INC = 0.02;
	bool revbutton = false, revstate = false; // bools associated with josh's ridiculous  needs
	bool rampbutton = false, rampstate = false;
	bool driveStraight = false, automode = false;
	bool usesock = false;
	const float TURN_SCALING = 0.03;
	const float MIN_SHOOTER_ANGLE = -50.00; // minimum angle the shooter can go down
	const float MAX_SHOOTER_ANGLE = 175.00; // maximum angle the shooter can go up
	char joybuff[sizeof(int)];
	int conn_desc;

public:
	Robot() :
		launchAngle(8), climbMotor(6), climbLock(7), launchLeft(0),
		launchRight(5), launchLoader(9), driverControl(0), shooterControl(1),
		pot(0), tLimit(2), bLimit(3), decswitch0(0), decswitch1(1), gyro(), revlatch(), robotimer(),
		ramplatch()
	{
		frontLeft = new VictorSP(1); // drive motor
		frontRight = new VictorSP(2); // drive motor
		rearLeft = new VictorSP(3); // drive motor
		rearRight = new VictorSP(4); // drive motor
		myDrive = new RobotDrive(frontLeft, frontRight, rearLeft, rearRight); // declare the robot drive
		myDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, false);
		myDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, false);
		myDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor,false);
		myDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, false);
		myDrive->SetExpiration(0.1);
		SmartDashboard::init();
	}
	~Robot()
	{
		delete frontLeft;
		delete frontRight;
		delete rearLeft;
		delete rearRight;
		delete myDrive;
	}

	float ShooterAngle()
	{
		float y = (pot.GetAverageVoltage() - 0.94) / 0.021;
		y -= 9.995 - 7.954;
		y += 4.078;
		std::cout << y << std::endl;
		return y;
	}

	void BallControl(float joy, bool shoot, bool suck, bool up, bool down, bool pshoot)
	{
		if (shoot)
			launchLoader.SetAngle(0);
		else
			launchLoader.SetAngle(50);

		if (pshoot) {
			launchLeft.Set(-1.0);
			launchRight.Set(1.0);
		} else if (suck) {
			launchLeft.Set(0.4);
			launchRight.Set(-0.4);
		} else {
			launchLeft.Set(0.0);
			launchRight.Set(0.0);
		}

		if (down) {
			if (ShooterAngle() >= MIN_SHOOTER_ANGLE)
				launchAngle.Set(-1.0);
			else
				launchAngle.Set(0.0);
		}
		else if (up) {
			if (ShooterAngle() <= MAX_SHOOTER_ANGLE)
				launchAngle.Set(1.0);
			else
				launchAngle.Set(0.0);
		}
		else
			launchAngle.Set(0.0);

		joy *= 160;
		joy += 160;
		joy += 0.5;
		SmartDashboard::PutNumber("Pixel Cord", (int)joy);

		float turnangle = 155.0 - (int)joy;
		turnangle *= 0.166;
		SmartDashboard::PutNumber("Angle To Turn", (int)turnangle);
		//std::snprintf(joybuff, sizeof(joybuff), "%d", 0);
		//send(conn_desc, joybuff, sizeof(joybuff), 0);
	}

	void ClimbControl(bool down, bool up)
	{
		if (down) {
			if (!bLimit.Get()) {
				climbMotor.Set(0.0);
				climbLock.Set(0.0);
			} else {
				climbLock.Set(0.0);
				climbMotor.Set(-1.0);
			}
		} else if (up) {
			if (!tLimit.Get()) {
				climbLock.Set(0.0);
				climbMotor.Set(0.0);
			} else {
				climbLock.Set(1.0);
				climbMotor.Set(1.0);
			}
		} else {
			climbLock.Set(0.0);
			climbMotor.Set(0.0);
		}
	}

	void RampSpeed(float leftjoy, float rightjoy, float &l, float &r)
	{
		if (leftjoy > l)
			l += SPEED_INC;
		else if (leftjoy < l)
			l -= SPEED_INC;

		if (rightjoy > r)
			r += SPEED_INC;
		else if (rightjoy < r)
			r -= SPEED_INC;
	}

	void PublishDash()
	{
		SmartDashboard::PutNumber("Gyro Angle", gyro.GetAngle());
		SmartDashboard::PutNumber("Pot Voltage", pot.GetAverageVoltage());
		SmartDashboard::PutNumber("Shooter Angle", (double)ShooterAngle());
		SmartDashboard::PutBoolean("Reversed", revstate);
		SmartDashboard::PutBoolean("Ramp", rampstate);
	}

private:

	void RobotInit()
	{
		SmartDashboard::PutString("Robot Messages", "Entering RobotInit()");
		gyro.Reset();
		SmartDashboard::PutString("Robot Messages", "Gyro Reset");
/*
		int sock_descriptor;
		struct sockaddr_in serv_addr, client_addr;
		sock_descriptor = socket(AF_INET, SOCK_STREAM, 0);
		if (sock_descriptor < 0)
			SmartDashboard::PutString("Robot Messages", "Couldn't make socket");
		else {
			bzero((char *)&serv_addr, sizeof(serv_addr));
			serv_addr.sin_family = AF_INET;
			serv_addr.sin_addr.s_addr = INADDR_ANY;
			serv_addr.sin_port = htons(5801);
			if (bind(sock_descriptor, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
				SmartDashboard::PutString("Robot Messages", "Failed to bind");
			else {
				listen(sock_descriptor, 5);
				SmartDashboard::PutString("Robot Messages", "Waiting for connection...");
				socklen_t size;
				size = sizeof(client_addr);
				struct sockaddr client_addr;
				conn_desc = accept(sock_descriptor, &client_addr, &size);
				if (conn_desc == -1) {
					SmartDashboard::PutString("Robot Messages", "Could not connect");
					usesock = false;
				}
				else {
					SmartDashboard::PutString("Robot Messages", "Connected to DS");
					usesock = true;
				}
			 }
		}
		*/
	}

	void AutonomousInit()
	{
		SmartDashboard::PutString("Robot Messages", "AutoInit()");
		robotimer.Reset();
		robotimer.Start();
		gyro.Reset();
	}

	void AutonomousPeriodic()
	{
		SmartDashboard::PutString("Robot Messages", "Entering AutoPeridoic()");
		PublishDash();
		float gyroangle = gyro.GetAngle();
		if (decswitch0.Get()) {
			if (robotimer.Get() < 6.0) {
				RampSpeed(0.0, 0.6, currleft, currright);
				myDrive->Drive(currright, -gyroangle * TURN_SCALING);
				BallControl(0.0, false, false, false, false, false);
			}
			else {
				myDrive->Drive(0.0, 0.0);
			}
		}
		else {
			myDrive->Drive(0.0, 0.0);
			if (ShooterAngle() > 55)
				BallControl(0.0, false, false, false, true, false);
			else
				BallControl(0.0, false, false, false, false, true);
				if (robotimer.Get() > 7.0)
					BallControl(0.0, true, false, false, false, true);
		}
		Wait(0.005);
	}

	void TeleopInit()
	{
		robotimer.Reset();
		gyro.Reset();
	}

	void TeleopPeriodic()
	{
		PublishDash();

		// Below is code for the drive system
		RampSpeed(driverControl.GetRawAxis(1), driverControl.GetRawAxis(3), currleft, currright);
		if (driverControl.GetRawButton(5)) {
			gyro.Reset();
			driveStraight = true;
			while (driveStraight) {
				float gyroangle = gyro.GetAngle();
				float speed = 0.8;
				if (driverControl.GetRawButton(6))
					speed = 1.0;
				else
					speed = 0.8;
				speed *= driverControl.GetRawAxis(3);
				myDrive->Drive(-speed, -gyroangle * TURN_SCALING);

				// non driving code
				ClimbControl(driverControl.GetRawButton(7), driverControl.GetRawButton(8));
				BallControl(shooterControl.GetRawAxis(0), shooterControl.GetRawButton(1), shooterControl.GetRawButton(3),
						shooterControl.GetRawButton(6), shooterControl.GetRawButton(5),
						shooterControl.GetRawButton(2));
				driveStraight = driverControl.GetRawButton(5); // keep this at the end
			}
		} else {
			driveStraight = false;
			revlatch.Toggle(driverControl.GetRawButton(1), revbutton, revstate);
			ramplatch.Toggle(driverControl.GetRawButton(4), rampbutton, rampstate);
			if (rampstate) {
				if (revstate)
					myDrive->TankDrive(currright, currleft);
				else
					myDrive->TankDrive(-currleft, -currright);
			} else {
				float speed = 0.8;
				if (driverControl.GetRawButton(6))
					speed = 1.0;
				else
					speed = 0.8;
				if (revstate)
					myDrive->TankDrive(driverControl.GetRawAxis(3) * speed, driverControl.GetRawAxis(1) * speed);
				else
					myDrive->TankDrive(-driverControl.GetRawAxis(1) * speed, -driverControl.GetRawAxis(3) * speed);
			}


		}
		// End of code for the drive system
		ClimbControl(driverControl.GetRawButton(7), driverControl.GetRawButton(8));
		BallControl(shooterControl.GetRawAxis(0), shooterControl.GetRawButton(1), shooterControl.GetRawButton(3),
				shooterControl.GetRawButton(6), shooterControl.GetRawButton(5),
				shooterControl.GetRawButton(2));
		// A little code for the assist aim system

		Wait(0.005);
	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot)
