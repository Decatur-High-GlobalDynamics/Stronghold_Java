package org.usfirst.frc.team4026.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String autoNameDefault = "Default";
	final String autoNameLowBar = "Low Bar";
	final String autoNameMoatRampart = "Moat or Rampart";
	final String autoNameScore = "Low Bar Score";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	RobotDrive myRobot;
	Talon rightDriveMotor;
	Talon leftDriveMotor;
	Talon intakeWheel;
	Spark intakeTilt;
	Spark winchTal;
	Joystick driveLeftStick;
	Joystick driveRightStick;
	Joystick manipulatorStick;
	Timer toggleButtonTimer;
	Timer autoDriveTimer;
	Timer winchTimer;
	Timer shootTimer;
	AnalogGyro driveGyro;
	DigitalInput shootLimitSwitch;
	AnalogInput wallDistanceSensor;
	DoubleSolenoid hanger;
	Compressor compressorPointer;
	int autoState;
	boolean winchRetracted;
	boolean shootTimerStarted;
	boolean driveReverse;
	boolean isGyroResetTelop;
	int turningButtonState;
	int turningButtonAngle;
	int currentTurningButton;
	double rightPower=0;
	double leftPower=0;
	double maxSpeedChange=0.2;
	double defualtThrottle=1;
	double guestThrottle=0.5;
	double intake_Wheel_Speed = 1.0;
	boolean use_Drive_Timer = true;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		/*chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);*/
		rightDriveMotor = new Talon(0);
		leftDriveMotor = new Talon(1);
		intakeWheel = new Talon(2);
		intakeTilt = new Spark(3);
		winchTal = new Spark(4);
		driveLeftStick = new Joystick(1);
		driveRightStick = new Joystick(0);
		manipulatorStick = new Joystick(2);
		toggleButtonTimer = new Timer();
		autoDriveTimer = new Timer();
		winchTimer = new Timer();
		shootTimer = new Timer();
		driveGyro = new AnalogGyro(0);
		shootLimitSwitch = new DigitalInput(0);
		wallDistanceSensor = new AnalogInput(1);
		autoState = 0;
		winchRetracted = false;
		isGyroResetTelop = false;
		hanger= new DoubleSolenoid(0, 3);
		driveReverse = false;
		toggleButtonTimer.reset();
		toggleButtonTimer.start();
		compressorPointer = new Compressor();
		compressorPointer.setClosedLoopControl(true);

		shootTimerStarted = false;
		shootTimer.reset();
		turningButtonState=0;
		turningButtonAngle=0;
		currentTurningButton=0;
		chooser.addDefault("Default Auto", autoNameDefault);
		chooser.addObject("Low Bar", autoNameLowBar);
		chooser.addObject("Moat or Rampart", autoNameMoatRampart);
		chooser.addObject("Low Bar Score(Non Functional)", autoNameScore);
		SmartDashboard.putData("Auto choices", chooser);
		driveGyro.reset();
		hanger.set(DoubleSolenoid.Value.kReverse);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		while (isAutonomous() && isEnabled())
		{
			if(autoSelected == autoNameLowBar){
				/*myRobot.SetSafetyEnabled(false);
				myRobot.Drive(-0.5, 1.0); 	// spin at half speed
				Wait(2.0); 				//    for 2 seconds
				myRobot.Drive(0.0, 0.0); 	// stop robot
				*/
				lowBarAutonomous();
			}
			else if (autoSelected == autoNameMoatRampart)
			{
				moatRampartAutonomous();
			}
			else if (autoSelected == autoNameScore)
			{
				lowBarScoreAutonomous();
			}
			else
			{
				//Default Auto goes here
	//			myRobot.SetSafetyEnabled(false);
	//			myRobot.Drive(0.0, 0.0); 	// stop robot
				driveStraightAutonomous();
			}
			updateDashboard();
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		//myRobot.SetSafetyEnabled(true);
				while (isOperatorControl() && isEnabled())
				{
					//if (turnButtons())
					//{
						tankDrive();
					//}
					intakeWheelControl(5,7);
					winchControl();
					ShooterControl();
					tiltControl(3, 2);  //Using stick now..not buttons
					updateDashboard();
					hangerPiston(6,8);
				}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	double smoothJoyStick(float joyInput)
	{
		return Math.pow(joyInput,3);
	}
	//-1 is reverse and 1 is norm
		void tankDrive()
		{
			toggleDriveDirection();
			//float right = smoothJoyStick(driveRightStick.GetY());
			//float left = smoothJoyStick(driveLeftStick.GetY());
			double right = driveRightStick.getY();
			double left = driveLeftStick.getY();
//			leftDriveMotor.Set(left);
//			rightDriveMotor.Set(-right);
			if(!driveRightStick.getTrigger())
			{
				if (driveReverse)
				{
					leftDriveMotor.set(-right);
					rightDriveMotor.set(left);
				}
				else
				{
					leftDriveMotor.set(left);
					rightDriveMotor.set(-right);
				}
				isGyroResetTelop = false;
			}
			else
			{
				if(isGyroResetTelop == false)
				{
					driveGyro.reset();
					isGyroResetTelop = true;
				}
				if (driveReverse)
				{
					keepDriveStraight(driveRightStick.getY(), driveRightStick.getY(), 0);
				}
				else
				{
					keepDriveStraight(-driveRightStick.getY(), -driveRightStick.getY(), 0);
				}
			}
		}
	/*
		void setDrive(float right, float left, bool reverse)
		{
			if (reverse)
			{
				leftDriveMotor.Set(-right);
				rightDriveMotor.Set(left);
			}
			else
			{
				leftDriveMotor.Set(left);
				rightDriveMotor.Set(-right);
			}
		}
	*/
		void hangerPiston(int extend, int retract)
		{
			if (manipulatorStick.getRawButton(extend))
			{
				hanger.set(DoubleSolenoid.Value.kForward);
			}
			else if (manipulatorStick.getRawButton(retract))
			{
				hanger.set(DoubleSolenoid.Value.kReverse);
			}
		}
		void toggleDriveDirection()
		{
			if(driveLeftStick.getRawButton(2))
			{
				driveReverse=true;
			}
			else
			{
				driveReverse=false;
			}
		}
		/*boolean turnButtons()
		{
			int direction = 1;
			if (driveReverse==true)
			{
				direction=-1;
			}
			switch(turningButtonState)
			{
				case 0:
					turningButtonAngle=0;
					currentTurningButton=0;
					if (driveRightStick.getRawButton(5))
					{
						driveGyro.reset();
						turningButtonAngle=(-90*direction);
						currentTurningButton=5;
						turningButtonState++;
					}
					else if (driveRightStick.getRawButton(4))
					{
						driveGyro.reset();
						turningButtonAngle=(90*direction);
						currentTurningButton=4;
						turningButtonState++;
					}
					else if (driveRightStick.getRawButton(2))
					{
						driveGyro.reset();
						turningButtonAngle=(180*direction);
						currentTurningButton=2;
						turningButtonState++;
					}
					return true;
				case 1:
					if (!driveRightStick.getRawButton(currentTurningButton) || turnGyro(turningButtonAngle))
					{
						turningButtonState++;
					}
					return false;
				case 2:
					if (!driveRightStick.getRawButton(currentTurningButton))
					{
						turningButtonState=0;
					}
					return false;
				default:
					turningButtonState=0;
			}
		}*/
		double calculateWallDistance(boolean averaged)
		{
			double rawVoltage;
			double crateDistance;

			if(averaged)
				rawVoltage = (double)(wallDistanceSensor.getAverageVoltage());
			else
				rawVoltage = (double)(wallDistanceSensor.getVoltage());

			//MB1013
			//double VFiveMM = 0.0048359375;  //((4.952 / 5120) * 5);
			//crateDistance = ((rawVoltage * 5 * 0.0393) / VFiveMM);  //Units inch

			//MB1030
			double VFiveMM = 0.009671875;
			crateDistance = rawVoltage / VFiveMM;

			return crateDistance;
		}
		void intakeWheelControl(int mIntakeInButton, int mIntakeOutButton)
		{
			if (manipulatorStick.getRawButton(mIntakeInButton) || driveLeftStick.getTrigger())
			{
				intakeWheel.set(-intake_Wheel_Speed);
			}
			else if (manipulatorStick.getRawButton(mIntakeOutButton))
			{
				intakeWheel.set(intake_Wheel_Speed);
			}
			else
			{
				intakeWheel.set(0);
			}
		}

		void tiltControl(int up, int down)
		{
			/*if (manipulatorStick.GetRawButton(up))
			{
				intakeTilt.Set(0.3);
			}
			else if (manipulatorStick.GetRawButton(down))
			{
				intakeTilt.Set(-0.1);
			}
			else
			{
				intakeTilt.Set(0);
			}*/
			intakeTilt.set(manipulatorStick.getY());
		}

		void winchControl()
		{
			//pulls back winc
			if (manipulatorStick.getRawButton(2) && !winchRetracted)
			{
				winchTimer.start();

				if(winchTimer.get() > 0.025)
				{
					if(!shootLimitSwitch.get())
					{
						winchRetracted = true;
					}
				}
				winchTal.setSpeed(0.5);
			}
			else
			{
				winchTimer.reset();
				if(!manipulatorStick.getRawButton(2))
				{
					winchRetracted = false;
				}
				winchTal.setSpeed(0.0);
			}
		}

		void ShootBall()
		{
			/*
			if(!shootTimerStarted)
			{
				shootTimer.Start();
				shootTimerStarted = true;
			}
			else
			{
				if(shootTimer.Get() > 0.3)
				{
					//Release winch
					if(shootTimer.Get() <= 0.4)
						winchTal.SetSpeed(1.0);
					else
						winchTal.SetSpeed(0.0);
				}
			}
			*/
			winchTal.setSpeed(-0.75);
		}

		void StopShooting()
		{
			//Stop winch
			if(!manipulatorStick.getRawButton(2))
				winchTal.setSpeed(0.0);

			shootTimer.stop();
			shootTimer.reset();
			shootTimerStarted = false;
		}

		void ShooterControl()
		{
			//Shoots the ball
			if (manipulatorStick.getRawButton(4))
			{
				ShootBall();
			}
			else
			{
				StopShooting();
			}
		}

		void stopRobotDrive()
		{
			leftDriveMotor.set(0.0);
			rightDriveMotor.set(0.0);
		}

		void resetDrive(boolean use_Drive_Timer2)
		{
			if(use_Drive_Timer2)
			{
				autoDriveTimer.reset();
				autoDriveTimer.start();
				driveGyro.reset();
			}
			else
			{
				//leftDriveEncoder.Reset();
				//rightDriveEncoder.Reset();
			}
		}

		void keepDriveStraight(double d, double e, float targetAngle)
		{
			double error = 0, correctionFactor;
			error = targetAngle - driveGyro.getAngle();
			correctionFactor = (error/75.0);

			if(targetAngle > (driveGyro.getAngle() - 0.5) || targetAngle < (driveGyro.getAngle() + 0.5))
			{
				leftDriveMotor.set((-d) + correctionFactor);
				rightDriveMotor.set(e + correctionFactor);
			}
			else
			{
				leftDriveMotor.set(-d);
				rightDriveMotor.set(e);
			}
		}

		boolean turnGyro(float rAngle)
		{
			double error = 0;
			//Positive gyro angle means turning left
			if(rAngle < driveGyro.getAngle())
			{
				error = Math.abs(rAngle) - driveGyro.getAngle();
				if(driveGyro.getAngle() <= Math.abs(rAngle) && Math.abs(error) > 2.0)
				{
					//turn left
					leftDriveMotor.set((error/45) + 0.2); //0.8  div 140
					rightDriveMotor.set((error/45) + 0.2); //0.8
				}
				else
				{
					stopRobotDrive();
					return true;
				}
			}
			else if(rAngle > driveGyro.getAngle())
			{
				error = -rAngle - driveGyro.getAngle();
				if(driveGyro.getAngle() >= -rAngle && Math.abs(error) > 2.0)
				{
					//turn right
					leftDriveMotor.set((error/45) - 0.2); //-0.8
					rightDriveMotor.set((error/45) - 0.2); //-0.8
				}
				else
				{
					stopRobotDrive();
					return true;
				}
			}
			else
			{
				stopRobotDrive();
				return true;
			}

			return false;
		}

		//Prior to calling this function you must call resetDrive1
		boolean autoDriveRobot(double d, double e, double f, float distanceInch, boolean isTimerBased)
		{
			if(isTimerBased)
			{
				if(autoDriveTimer.get() <= f)
				{
					//leftDriveMotor.Set(-velocityLeft);
					//rightDriveMotor.Set(velocityRight);
					keepDriveStraight(d, e, 0);
				}
				else
				{
					stopRobotDrive();
					return true;
				}
			}
			else
			{
				/*if(fabs(convertDriveTicksToInches(rightDriveEncoder.GetRaw())) < fabs(distanceInch))
				{
					leftDriveMotor.Set(-velocityLeft);
					rightDriveMotor.Set(velocityRight);
				}
				else
				{
					stopRobotDrive();
					return true;
				}*/
			}
			return false;
		}

		void driveStraightAutonomous()
		{
			switch(autoState)
			{
				case 0:
					resetDrive(use_Drive_Timer);
					intakeTilt.set(-0.4);
					//wait( 1.0);
					autoState++;
					break;
				case 1:
					intakeTilt.set(0.0);
					resetDrive(use_Drive_Timer);
					intakeWheel.set(-1);
					//wait( 0.5);
					autoState++;
					break;
				case 2:
					intakeWheel.set(0.0);
					if(autoDriveRobot(0.6, 0.6, 1.0, 0, use_Drive_Timer)) //1.0 sec
					{
						resetDrive(use_Drive_Timer);
						autoState++;
					}
					break;

				case 3:
					if(autoDriveRobot(0.85, 0.85, 2.25, 0, use_Drive_Timer))
						autoState++;
					break;

				case 4:
					stopRobotDrive();
					break;
				default:
					stopRobotDrive();
					break;
			}
		}
		void moatRampartAutonomous()
		{
			switch(autoState)
			{
				case 0:
					resetDrive(use_Drive_Timer);
					intakeTilt.set(-0.4);
					//wait(1.0);
					autoState++;
					break;
				case 1:
					intakeTilt.set(0.0);
					resetDrive(use_Drive_Timer);
					intakeWheel.set(-1);
					//wait(0.5);
					autoState++;
					break;
				case 2:
					intakeWheel.set(0.0);
					if(autoDriveRobot(0.6, 0.6, 1.0, 0, use_Drive_Timer)) //1.0 sec
					{
						resetDrive(use_Drive_Timer);
						autoState++;
					}
					break;

				case 3:
					if(autoDriveRobot(0.85, 0.85, 2.5, 0, use_Drive_Timer))
						autoState++;
					break;

				case 4:
					stopRobotDrive();
					break;
				default:
					stopRobotDrive();
					break;
			}
		}
		void lowBarScoreAutonomous()
		{
			//float wallTurnDistance;
			switch(autoState)
			{
				case 0:
					resetDrive(use_Drive_Timer);
					intakeTilt.set(-0.4);
					//wait(1.0);
					autoState++;
					break;
				case 1:
					intakeTilt.set(0.0);
					resetDrive(use_Drive_Timer);
					intakeWheel.set(-1);
					//wait(0.5);
					autoState++;
					break;
				case 2:
					intakeWheel.set(0.0);
					if(autoDriveRobot(0.5, 0.5, 1.0, 0, use_Drive_Timer))
					{
						resetDrive(use_Drive_Timer);
						autoState++;
					}
					break;

				case 3:
					if(autoDriveRobot(0.5, 0.5, 2.5, 0, use_Drive_Timer))
					{
						resetDrive(use_Drive_Timer);
						autoState++;
					}
					break;
				//case 4:
				//	if (calculateWallDistance(true)>wallTurnDistance){}
				//	break;
				case 999:
					stopRobotDrive();
					break;
				default:
					stopRobotDrive();
					break;
			}
		}

		void lowBarAutonomous()
		{
			switch(autoState)
			{
				case 0:
					resetDrive(use_Drive_Timer);
					intakeTilt.set(-0.4);
					//wait(1.0);
					autoState++;
					break;
				case 1:
					intakeTilt.set(0.0);
					resetDrive(use_Drive_Timer);
					intakeWheel.set(-1);
					//wait(0.5);
					autoState++;
					break;
				case 2:
					intakeWheel.set(0.0);
					if(autoDriveRobot(0.6, 0.6, 1.0, 0, use_Drive_Timer))
					{
						resetDrive(use_Drive_Timer);
						autoState++;
					}
					break;

				case 3:
					if(autoDriveRobot(0.6, 0.6, 2.5, 0, use_Drive_Timer))
						autoState++;
					break;

				case 4:
					stopRobotDrive();
					break;
				default:
					stopRobotDrive();
					break;
			}
		}		
		void updateDashboard()
		{
			SmartDashboard.putNumber("Wall Distance: ", calculateWallDistance(false));
			SmartDashboard.putNumber("Gyro Reading: ", driveGyro.getAngle());
		}
}


