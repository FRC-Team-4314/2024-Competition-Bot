// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax upperLeftDrive = new PWMSparkMax(0);
  private final PWMSparkMax upperRightDrive = new PWMSparkMax(1);
  private final PWMSparkMax lowerLeftDrive = new PWMSparkMax(2);
  private final PWMSparkMax lowerRightDrive = new PWMSparkMax(3);

  private final MecanumDrive mecanumDrive = new MecanumDrive(upperLeftDrive, lowerLeftDrive, upperRightDrive, lowerRightDrive);
  
  private final Joystick Lcontroller = new Joystick(0);
  private final Joystick Rcontroller = new Joystick(1);

  private double currSpeedL = 0, currSpeedR = 0;
  private double leftStickY = 0, rightStickY = 0;


  private final Timer timer = new Timer();
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
  private double speedControl(double currentSpeed, double targetSpeed){
        
      double newSpeed = currentSpeed;
      //Initial speed increase is fast to prevent a delay between controller input and the motor
      //If currentSpeed is < 0.25 and the targetSpeed is > than 0.25 then move
      if(Math.abs(currentSpeed) < 0.25 && Math.abs(targetSpeed) >= 0.25){
          newSpeed += Math.signum(targetSpeed) * Math.min(0.025, Math.max(0.25 - Math.abs(currentSpeed), 0.005));

      }
      //If the targetSpeed is greater than currentSpeed then increase the speed
      else if (Math.abs(targetSpeed) > Math.abs(currentSpeed) && (currentSpeed == 0 || (Math.signum(targetSpeed) == Math.signum(currentSpeed)))){
          //increases the speed at a minimum of 0.001
          newSpeed += Math.signum(targetSpeed) * Math.max(0.005 * (Math.abs(targetSpeed) - Math.abs(currentSpeed)), 0.005);
      }
      //
      else if (Math.abs(currentSpeed) >= 0.01) {
          newSpeed -= Math.signum(currentSpeed) * Math.max(0.005 * (Math.abs(targetSpeed) - Math.abs(currentSpeed)), 0.015);
      }
      //Stops the robot when the speed is low enough
      else{
          newSpeed = 0;
      }
      //Caps the max speed at 1 
      if(Math.abs(newSpeed) > 1.0)
          newSpeed = 1.5 * Math.signum(newSpeed);

      return newSpeed;
      
    }
}
