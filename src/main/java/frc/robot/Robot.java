// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriverDisplay;
import frc.robot.subsystems.VisionSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  boolean crouchMode = true;
  double crouchSpeed = 1;
  boolean preciseMode = true;
  double LSX, LSY, RSX;
  double LSYB, RSYB;
  double timeSinceStartAtAutoStart = 0;
  int selectedAutoSequence = Constants.defaultAutoSequence;
  double demoTargetX = 0;
  double demoTargetY = 0;
  boolean advancedArmControlEnabled = false;
  boolean RPS = false;
  int RPSValue = 0;
  double RPSTime = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // ^No it won't
    m_robotContainer = new RobotContainer();

    Constants.gripperMotorA.setSmartCurrentLimit((int)Constants.clawAmpLimit);
    Constants.gripperMotorB.setSmartCurrentLimit((int)Constants.clawAmpLimit);
    Constants.primaryAccelerometer.setYaw(0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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
    selectedAutoSequence = (int)DriverDisplay.autoSequenceSelector.getInteger(0);
    demoTargetX = DriverDisplay.demoTargetXSelector.getDouble(0);
    demoTargetY = DriverDisplay.demoTargetYSelector.getDouble(0);
    SmartDashboard.putNumber("Targeted X", demoTargetX);
    SmartDashboard.putNumber("Targeted Y", demoTargetY);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    timeSinceStartAtAutoStart = Timer.getFPGATimestamp();
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() 
  {
    switch(selectedAutoSequence)
    {
      case 0:
        autoSequence2();
        break;
      case 1:
        autoSequence2();
        break;
      case 2:
        autoSequence2();
        break;
      default:
        autoSequence2();
        break;
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    advancedArmControlEnabled = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    //Calculates the speed multipler applied to the joystick axes
    crouchSpeed = 1-((1-Constants.swerveCrouchModeMult) * Constants.controller.getLeftTriggerAxis());
    if(Constants.controllerB.getBButtonPressed())
    {
      preciseMode = !preciseMode;
    }
      
    //Updates doubles representing stick axes
    LSX = (crouchSpeed)*Functions.Exponential(Functions.DeadZone(Constants.controller.getLeftX(), Constants.controllerDeadZone));
    LSY = (crouchSpeed)*Functions.Exponential(Functions.DeadZone(-Constants.controller.getLeftY(), Constants.controllerDeadZone));
    RSX = Constants.turnMultiplier * (crouchSpeed)*Functions.Exponential(Functions.DeadZone(Constants.controller.getRightX(), Constants.controllerDeadZone));

    LSYB = (preciseMode?Constants.armPreciseModeMult:1)*Functions.Exponential(Functions.DeadZone(Constants.controllerB.getLeftY(), Constants.controllerDeadZone));
    RSYB = (preciseMode?Constants.armPreciseModeMult:1)*Functions.Exponential(Functions.DeadZone(Constants.controllerB.getRightY(), Constants.controllerDeadZone));

    //Determines what drive function to use based on controller buttons
    if(Constants.controller.getAButton())
    {
      RobotContainer.driveTrain.DriveTo(demoTargetX, demoTargetY, RobotContainer.driveTrain.robotYawFieldRelative);
    }
    else if(Constants.controller.getPOV() >= 0)
    {
      RobotContainer.driveTrain.DriveDriverOrientedAtAngle(LSX, LSY, Constants.controller.getPOV());
    }
    else
    {
      RobotContainer.driveTrain.DriveDriverOriented(LSX, LSY, RSX);
    }

    if(Constants.controller.getRightBumper())
    {
      Constants.primaryAccelerometer.setYaw(0);
    }
    
    if(Constants.controllerB.getXButtonPressed())
    {
      advancedArmControlEnabled = !advancedArmControlEnabled;
      RobotContainer.armSystem.setTargetToCurrent();
    }
    if(advancedArmControlEnabled){
      RobotContainer.armSystem.updateTarget(RSYB, LSYB);
      RobotContainer.armSystem.moveArmToTarget();
    }else{
      RobotContainer.armSystem.moveArm(LSYB, RSYB);
    }
    if(!RPS)
    {
      Constants.gripperMotorA.set(-(Constants.controllerB.getRightTriggerAxis()-Constants.controllerB.getLeftTriggerAxis()));
      Constants.gripperMotorB.set((Constants.controllerB.getRightTriggerAxis()-Constants.controllerB.getLeftTriggerAxis()));
    }
    if(Constants.controllerB.getStartButtonPressed() && advancedArmControlEnabled)
    {
      RPS = true;
      RPSValue = (int)(3*Math.random());
      RPSTime = 0;
      RobotContainer.armSystem.setTargetToCurrent();
    }
    if(Constants.controllerB.getStartButton() && advancedArmControlEnabled)
    {
      RPSTime += 0.02;
      RobotContainer.armSystem.RockPaperScissorsPeriodic(RPSTime, RPSValue);
    }
    if(Constants.controllerB.getStartButtonReleased() && advancedArmControlEnabled)
    {
      Constants.gripperMotorA.set(0);
      Constants.gripperMotorB.set(0);
      RobotContainer.armSystem.setTargetToCurrent();
      RPS = false;
    }

    if(Constants.controllerB.getYButtonPressed()){
      RobotContainer.armSystem.resetArmAngles();
    }


    if(Constants.PDP.getTotalCurrent() > Constants.currentWarningLevel)
    {Constants.controller.setRumble(RumbleType.kBothRumble, 1);
      Constants.controllerB.setRumble(RumbleType.kBothRumble, 1);}
    else
    {Constants.controller.setRumble(RumbleType.kBothRumble, 0);
      Constants.controllerB.setRumble(RumbleType.kBothRumble, 0);}
    
    if(Constants.controller.getYButtonPressed())
    {
      if(!VisionSubsystem.camCheck())
      {
        VisionSubsystem.conFieldX = 0;
        VisionSubsystem.conFieldY = 0;
      }
      DriverDisplay.demoTargetXSelector.setDouble(VisionSubsystem.conFieldX);
      DriverDisplay.demoTargetYSelector.setDouble(VisionSubsystem.conFieldY);
    }
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

  public boolean isAutoTimeBetween(double time1, double time2)
  {
    if(time1 < Timer.getFPGATimestamp() - timeSinceStartAtAutoStart && Timer.getFPGATimestamp()-timeSinceStartAtAutoStart < time2)
    {
      return true;
    }
    return false;
  }
  public void autoSequence0()
  {
    Functions.KillAllArm();
    Functions.KillAllSwerve();
    Constants.primaryAccelerometer.setYaw(0);
  }
  public void autoSequence1()
  {
    if(isAutoTimeBetween(0, 1)) //first 1 second
    {
      Functions.KillAllSwerve();
      Functions.KillAllArm();
      Constants.primaryAccelerometer.setYaw(0);
    }
    else if(isAutoTimeBetween(1, 2)) //next 1 seconds
    {
      RobotContainer.driveTrain.DriveDriverOrientedAtAngle(0,0.5,0);
    }
    else
    {
      Functions.KillAllArm();
      Functions.KillAllSwerve();
    }
  }
  public void autoSequence2()
  {
    if(isAutoTimeBetween(0, 1)) //first 1 second
    {
      Functions.KillAllSwerve();
      Functions.KillAllArm();
      Constants.primaryAccelerometer.setYaw(0);
    }
    else if(isAutoTimeBetween(1, 2.6)) //next 1 seconds
    {
      RobotContainer.driveTrain.DriveDriverOrientedAtAngle(0,0.4,0);
    }
    else if(isAutoTimeBetween(2.6, 15)) //remainder
    {
      RobotContainer.driveTrain.AutoBalance();
    }
    else
    {
      Functions.KillAllArm();
      Functions.KillAllSwerve();
    }
  }
}
