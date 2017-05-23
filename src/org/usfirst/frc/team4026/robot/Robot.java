package org.usfirst.frc.team4026.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4026.robot.PDP;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	/*final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();*/
	RobotDrive myRobot;
	Talon rightDriveMotor;
	Talon leftDriveMotor;
	Talon intakeWheel;
	Spark intakeTilt;
	Spark winchTal;
	Joystick driveGamepad;
	Joystick guestStick;
	Timer toggleButtonTimer;
	Timer autoDriveTimer;
	Timer winchTimer;
	Timer shootTimer;
	AnalogGyro driveGyro;
	DigitalInput shootLimitSwitch;
	AnalogInput wallDistanceSensor;
	DoubleSolenoid hanger;
	Compressor compressorPointer;
	//PowerDistributionPanel pdp;
	PDP pdp;
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
		driveGamepad = new Joystick(0);
		guestStick = new Joystick(1);
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
		hanger.set(Value.kReverse);
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
		/*autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);*/
	}
	void updateDashboard(){
		SmartDashboard.putNumber("Current for Channel 0", pdp.current(0));
		SmartDashboard.putNumber("Current for Channel 1", pdp.current(1));
		SmartDashboard.putNumber("current for Channel 2", pdp.current(2));
		SmartDashboard.putNumber("Current for Channel 3", pdp.current(3));
		SmartDashboard.putNumber("Current for Channel 4", pdp.current(4));
		SmartDashboard.putNumber("Current for Channel 5", pdp.current(5));
		SmartDashboard.putNumber("Current for Channel 6", pdp.current(6));
		SmartDashboard.putNumber("Current for Channel 7", pdp.current(7));
		SmartDashboard.putNumber("Current for Channel 8", pdp.current(8));
		SmartDashboard.putNumber("Current for Channel 9", pdp.current(9));
		SmartDashboard.putNumber("Current for Channel 10", pdp.current(10));
		SmartDashboard.putNumber("Current for Channel 11", pdp.current(11));
		SmartDashboard.putNumber("Current for Channel 12", pdp.current(12));
		SmartDashboard.putNumber("Current for Channel 13", pdp.current(13));
		SmartDashboard.putNumber("Current for Channel 14", pdp.current(14));
		SmartDashboard.putNumber("Current for Channel 15", pdp.current(15));
		SmartDashboard.putNumber("PDP Total Current:", pdp.totalCurrent());
		SmartDashboard.putNumber("PDP Total Power:", pdp.totalPower());
		SmartDashboard.putNumber("PDP Total Energy:", pdp.totalEnergy());
		SmartDashboard.putNumber("PDP Input Voltage:", pdp.voltage());
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
	/*	switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			break;
		}*/
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		while (isOperatorControl())
		{
			if (driveGamepad.getRawButton(10))
			{
				guestDrive(guestThrottle);
				guestIntakeControl(1,2);
				updateDashboard();
			}
			else if (driveGamepad.getRawButton(8))
			{
				tankDrive(defualtThrottle);
				intakeWheelControl(5,7);
				cheval(2,4);
				hangerPiston(1,3);
				updateDashboard();
			}
			else
			{
				stopRobotDrive();
				winchTal.setSpeed(0.0);
				intakeWheel.set(0);
				updateDashboard();
			}
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	double smoothJoyStick(float joyInput)
	{
		return Math.pow(joyInput,3);
	}
	//-1 is reverse and 1 is norm
		void tankDrive(double defualtThrottle2)
		{
			double right = driveGamepad.getY();
			double left  = driveGamepad.getThrottle();
			double rightCorrection = ((right - left)/2)*(1-defualtThrottle2);
			double leftCorrection = ((left - right)/2)*(1-defualtThrottle2);
			right *=defualtThrottle2;
			left *=defualtThrottle2;
			setDriveMotors(right + rightCorrection, left + leftCorrection, driveGamepad.getRawButton(8), 1);
		}
		void guestDrive(double guestThrottle2)
		{
			double x = guestStick.getX();
			double y = guestStick.getX()*guestThrottle2;
			double right = y-x;
			double left = y+x;
			setDriveMotors(right, left, false, 1);
		}

		void setDrive(float right, float left, boolean reverse, float throttle)
		{
			rightPower=rightDriveMotor.get()/throttle; //will never be outside of -throttle to +throttle
			leftPower=leftDriveMotor.get()/throttle;
			double rightDirection;
			double leftDirection;
			if ((right-rightPower)==0)
			{
				rightDirection = 1;
				leftDirection = 1;
			}
			else
			{
				rightDirection = (right-rightPower)/Math.abs(right-rightPower); // -1 or 1 returened for back/forward
				leftDirection = (left-leftPower)/Math.abs(left-leftPower);
			}
			double rightDifference = Math.min(Math.abs(right-rightPower), maxSpeedChange);
			double leftDifference = Math.min(Math.abs(left-leftPower), maxSpeedChange);
			rightPower+=rightDifference*rightDirection;
			leftPower+=leftDifference*leftDirection;
			setDriveMotors(rightPower, leftPower, reverse, throttle);
		}
		void setDriveMotors(double rightPower2, double leftPower2, boolean reverse, float throttle)
		{
			if (reverse)
			{
				leftDriveMotor.set(leftPower2*throttle);
				rightDriveMotor.set(-rightPower2*throttle);
			}
			else
			{
				leftDriveMotor.set(-rightPower2*throttle);
				rightDriveMotor.set(leftPower2*throttle);
			}
		}
		void hangerPiston(int extend, int retract)
		{
			if (driveGamepad.getRawButton(extend))
			{
				hanger.set(DoubleSolenoid.Value.kForward);
			}
			else if (driveGamepad.getRawButton(retract))
			{
				hanger.set(DoubleSolenoid.Value.kReverse);
			}
		}
		void intakeWheelControl(int mIntakeInButton, int mIntakeOutButton)
		{
			if (driveGamepad.getRawButton(mIntakeInButton))
			{
				intakeWheel.set(-intake_Wheel_Speed);
			}
			else if (driveGamepad.getRawButton(mIntakeOutButton))
			{
				intakeWheel.set(intake_Wheel_Speed);
			}
			else
			{
				intakeWheel.set(0);
			}
		}
		void guestIntakeControl(int mIntakeInButton, int mIntakeOutButton)
		{
			if (guestStick.getRawButton(mIntakeInButton))
			{
				intakeWheel.set(-intake_Wheel_Speed);
			}
			else if (guestStick.getRawButton(mIntakeOutButton))
			{
				intakeWheel.set(intake_Wheel_Speed);
			}
			else
			{
				intakeWheel.set(0);
			}
		}
		void cheval(int upButton, int downButton)
		{
			if (driveGamepad.getRawButton(upButton) && shootLimitSwitch.get())
			{
				winchTal.setSpeed(0.5);
			}
			else if (driveGamepad.getRawButton(downButton))
			{
				winchTal.setSpeed(-0.5);
			}
			else
			{
				winchTal.setSpeed(0.0);
			}
		}

		void stopRobotDrive()
		{
			leftDriveMotor.set(0.0);
			rightDriveMotor.set(0.0);
		}
		
}

