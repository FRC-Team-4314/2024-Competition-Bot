// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
// import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.cscore.*;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

// import com.ctre.phoenix6.hardware.*;
// import com.ctre.phoenix6.mechanisms.*;
import com.ctre.phoenix.motorcontrol.can.*;
// import com.ctre.phoenix.signals.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
/*
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {	
	private final WPI_VictorSPX upperLeftDrive = new WPI_VictorSPX(4);
	private final WPI_VictorSPX upperRightDrive = new WPI_VictorSPX(3);
	private final WPI_VictorSPX lowerLeftDrive = new WPI_VictorSPX(5);
	private final WPI_VictorSPX lowerRightDrive = new WPI_VictorSPX(1);
	
	private final WPI_VictorSPX launcherMotor = new WPI_VictorSPX(2);
	private final WPI_VictorSPX launcherMotor2 = new WPI_VictorSPX(0);
	private final WPI_VictorSPX robotLiftMotor = new WPI_VictorSPX(6);

	private final MecanumDrive mecanumDrive = new MecanumDrive(upperLeftDrive, lowerLeftDrive, upperRightDrive,
			lowerRightDrive);

	private final XboxController controller = new XboxController(0);

	private double currSpeedX = 0, currSpeedY = 0, currRotation = 0;
	private double stickX = 0, stickY = 0, rStickX = 0;
	private int autoSet = 0;
	private final Timer timer = new Timer();
	private Command autonomousCommand;

	private RobotContainer robotContainer;

	
	private boolean primed = false;
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		launcherMotor.set(VictorSPXControlMode.PercentOutput,0);
		launcherMotor2.set(VictorSPXControlMode.PercentOutput,0);
		upperRightDrive.setInverted(true);
		lowerRightDrive.setInverted(true);
		lowerLeftDrive.setInverted(false);
		upperLeftDrive.setInverted(false);
		robotLiftMotor.setInverted(true);
		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.s

		
		robotContainer = new RobotContainer();

		CameraServer.startAutomaticCapture();
		// CvSink frontSink = new CvSink("frontSink");
		// frontSink.setSource(frontCam);
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items
	 * like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		
		CommandScheduler.getInstance().run();
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = robotContainer.getAutonomousCommand();
		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
		timer.reset();
		timer.start();
		
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		// if (timer.get() >= 0 && timer.get() <= 2){ // Primes for 1 second
		// 	launcherMotor2.set(ControlMode.PercentOutput, 1);
		// 	if (timer.get() >= 1){ //Fires
		// 		launcherMotor.set(ControlMode.PercentOutput, 1);
		// 	}
		// 	}
		// if (autoSet == 0){ // In front of the subwoofer(parellel to speaker)
		// 	if (timer.get() >= 1.5 && timer.get() <= 4.5){ // Moves back unkown ft
		// 		currSpeedY = 0.5;
		// 		currSpeedX = 0.5;
		// 	}
		// 	}
		// if (autoSet == 1){
		// }
			currRotation = 0;
			currSpeedX = 0;
			currSpeedY = 0;
			launcherMotor2.set(ControlMode.PercentOutput,0);
		

		mecanumDrive.driveCartesian(currSpeedX, -currSpeedY, currRotation);

	}

	@Override
	public void teleopInit() {
		currRotation = 0;
		currSpeedX = 0;
		currSpeedY = 0;
		
		SmartDashboard.putBoolean("primed", primed);
		SmartDashboard.putNumber("lift", lift);

		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		if (RobotState.isEnabled()) {
			/* lowerLeftDrive: launch motor
			 * 
			 * 
			 * 
			 * 
			 */
		 if(controller.getYButton())
		 {
			upperLeftDrive.set(ControlMode.PercentOutput,.5);
		 	return;
		}
		 else{
		 }
			lowerLeftDrive.set(0);
			
			stickX = Math.abs(controller.getLeftX()) < .1 ? 0 : controller.getLeftX();
			stickY = Math.abs(controller.getLeftY()) < .1 ? 0 : controller.getLeftY();
			rStickX = Math.abs(controller.getRightX()) < .1 ? 0 : controller.getRightX();

			currRotation = speedControl(currRotation, rStickX);
			currSpeedX = speedControl(currSpeedX, stickX);
			currSpeedY = speedControl(currSpeedY, stickY);

			if(controller.getRightBumperPressed())
			{
				primed = !primed;
			}
			// logic for launching the note
			if (controller.getRightTriggerAxis()>.5) {
				launcherMotor.set(VictorSPXControlMode.PercentOutput,1.0);
				launcherMotor2.set(VictorSPXControlMode.PercentOutput,1.0);
				primed= false;
			}
			else if(primed){
				launcherMotor2.set(VictorSPXControlMode.PercentOutput, 1);
			}
			else if(controller.getAButton()) // Loading through the front
			{
				launcherMotor.set(VictorSPXControlMode.PercentOutput,-.5);
				launcherMotor2.set(VictorSPXControlMode.PercentOutput,-.5);	
			}
			else if(controller.getPOV() != -1){ // Mechanism to pull robot on chain
				if (controller.getPOV() == 0){
					lift++;
					robotLiftMotor.set(ControlMode.PercentOutput,.5);
				}
				else if (controller.getPOV() == 180){
					lift--;
					robotLiftMotor.set(ControlMode.PercentOutput,-0.5);
				}
			}
			else {
				primed = false;
				launcherMotor.set(VictorSPXControlMode.PercentOutput,0);
				launcherMotor2.set(VictorSPXControlMode.PercentOutput,0);
				robotLiftMotor.set(ControlMode.PercentOutput,0);

			}
			

		} 
		else {
			// Defaults
			currSpeedX = 0.0;
			currSpeedY = 0.0;
			currRotation = 0.0;
			launcherMotor.set(VictorSPXControlMode.PercentOutput,0);
			launcherMotor2.set(VictorSPXControlMode.PercentOutput,0);
		}

		mecanumDrive.driveCartesian(currSpeedX, -currSpeedY, currRotation);
		// SmartDashboard.updateValues();
		SmartDashboard.putBoolean("primed", primed);		
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
	}

	private double speedControl(double currentSpeed, double targetSpeed) {

		double newSpeed = currentSpeed;
		// Initial speed increase is fast to prevent a delay between controller input
		// and the motor
		// If currentSpeed is < 0.25 and the targetSpeed is > than 0.25 then move
		if (Math.abs(currentSpeed) < 0.25 && Math.abs(targetSpeed) >= 0.25) {
			newSpeed += Math.signum(targetSpeed) * Math.min(0.025, Math.max(0.25 - Math.abs(currentSpeed), 0.005));

		}
		// If the targetSpeed is greater than currentSpeed then increase the speed
		else if (Math.abs(targetSpeed) > Math.abs(currentSpeed)
				&& (currentSpeed == 0 || (Math.signum(targetSpeed) == Math.signum(currentSpeed)))) {
			// increases the speed at a minimum of 0.001
			newSpeed += Math.signum(targetSpeed)
					* Math.max(0.005 * (Math.abs(targetSpeed) - Math.abs(currentSpeed)), 0.005);
		}
		//
		else if (Math.abs(currentSpeed) >= 0.01) {
			newSpeed -= Math.signum(currentSpeed)
					* Math.max(0.005 * (Math.abs(targetSpeed) - Math.abs(currentSpeed)), 0.015);
		}
		// Stops the robot when the speed is low enough
		else {
			newSpeed = 0;
		}
		// Caps the max speed at 1
		if (Math.abs(newSpeed) > 1.0)
			newSpeed = 1.0 * Math.signum(newSpeed);

		return newSpeed;

	}
}
