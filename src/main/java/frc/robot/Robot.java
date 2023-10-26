// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.dense.row.decomposition.hessenberg.HessenbergSimilarDecomposition_FDRM;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriverDisplay;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem2D;

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
  double demoTargetAngle = 0;
  boolean RPS = false;
  int RPSValue = 0;
  double RPSTime = 0;
  boolean holdArmPosition = false;
  double armTargetAngle = 0;
  double clawTargetAngle = 0;

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
    Constants.primaryAccelerometerPitchOffset = Constants.primaryAccelerometer.getPitch();
    //Constants.primaryAccelerometer.configMountPosePitch(Constants.primaryAccelerometer.getPitch());
    RobotContainer.armSystem.resetArmAngles();
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
    demoTargetAngle = DriverDisplay.demoTargetAngleSelector.getDouble(0);
    SmartDashboard.putNumber("Targeted X", demoTargetX);
    SmartDashboard.putNumber("Targeted Y", demoTargetY);
    SmartDashboard.putNumber("Targeted Angle", demoTargetAngle);

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
        autoSequence0();
        break;
      case 1:
        autoSequence1();
        break;
      case 2:
        autoSequence2();
        break;
      case 3:
        //autoSequence3();
        break;
      case 4:
        autoSequence4();
        break;
      case 5:
        //autoSequence5();
        break;
      case 6:
      autoSequence6();
        break;
      default:
        autoSequence0();
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
    holdArmPosition = false;

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    //Calculates the speed multipler applied to the joystick axes
    crouchSpeed = 1-((1-Constants.swerveCrouchModeMult) * Constants.controller.getLeftTriggerAxis());
    if(Constants.controllerB.getLeftStickButtonPressed())
    {
      preciseMode = true;
    }
      
    //Updates doubles representing stick axes
    LSX = 1.0*(crouchSpeed)*Functions.Exponential(Functions.DeadZone(Constants.controller.getLeftX(), Constants.controllerDeadZone));
    LSY = 1.0*(crouchSpeed)*Functions.Exponential(Functions.DeadZone(-Constants.controller.getLeftY(), Constants.controllerDeadZone));
    RSX = Constants.turnMultiplier * (crouchSpeed)*Functions.Exponential(Functions.DeadZone(Constants.controller.getRightX(), Constants.controllerDeadZone));

    LSYB = (preciseMode?Constants.armPreciseModeMult:1)*Functions.Exponential(Functions.DeadZone(Constants.controllerB.getLeftY(), Constants.controllerDeadZone));
    RSYB = (preciseMode?Constants.armPreciseModeMult:1)*Functions.Exponential(Functions.DeadZone(Constants.controllerB.getRightY(), Constants.controllerDeadZone));

    //Determines what drive function to use based on controller buttons
    if(Constants.controller.getAButton())
    {
      RobotContainer.driveTrain.DriveTo(demoTargetX, demoTargetY, demoTargetAngle, crouchSpeed, crouchSpeed);
    }
    else if(Constants.controller.getPOV() >= 0)
    {
      RobotContainer.driveTrain.DriveDriverOrientedAtAngle(LSX, LSY, Constants.controller.getPOV(), crouchSpeed);
    }
    else if(Constants.controller.getXButton())
    {
      RobotContainer.driveTrain.AutoBalance();
    }
    else if(Constants.controller.getBButton())
    {
      RobotContainer.driveTrain.DriveDriverOriented(LSX, LSY, VisionSubsystem2D.tagYaw / 270.0);
    }
    else
    {
      RobotContainer.driveTrain.DriveDriverOriented(LSX, LSY, RSX);
    }

    if(Constants.controller.getRightBumper())
    {
      Constants.primaryAccelerometer.setYaw(0);
    }
    
    if(Constants.controllerB.getLeftBumperPressed())
    {
      holdArmPosition = !holdArmPosition;
      armTargetAngle = RobotContainer.armSystem.raiseAngle;
      clawTargetAngle = RobotContainer.armSystem.segment2HorizonAngle;
    }

    if(!RPS)
    {
      if(Constants.controllerB.getYButton())
      {
        RobotContainer.armSystem.moveArmToAngles(Constants.armCollectAngle1, Constants.armCollectAngle2);
        holdArmPosition = false;
      }
      else if(Constants.controllerB.getXButton())
      {
        RobotContainer.armSystem.moveArmToAngles(Constants.armDepositAngle1, Constants.armDepositAngle2);
        holdArmPosition = false;
      }
      else if(Constants.controllerB.getBButton())
      {
        RobotContainer.armSystem.moveArmToAngles(Constants.armScoopAngle1, Constants.armScoopAngle2);
        holdArmPosition = false;
      }
      else if(Constants.controllerB.getAButton())
      {
        RobotContainer.armSystem.moveArmToAngles(Constants.armRestingAngle1, Constants.armRestingAngle2);
        holdArmPosition = false;
      }
      else if(Constants.controllerB.getBackButton())
      {
        RobotContainer.armSystem.moveArmToAngles(-5, 0);
        holdArmPosition = false;
      }
      else if(Constants.controllerB.getPOV() == 0)
      {
        RobotContainer.armSystem.moveArmToAngles(Constants.armBackDepositAngle1, Constants.armBackDepositAngle2);
        holdArmPosition = false;
      }
      else if(holdArmPosition)
      {
        armTargetAngle -= 2.5*LSYB;
        clawTargetAngle -= 2.5*RSYB;
        RobotContainer.armSystem.moveArmToAngles(armTargetAngle, clawTargetAngle);
      }
      else{RobotContainer.armSystem.moveArm(-LSYB, -RSYB);}
      Constants.gripperMotorA.set(Functions.Clamp(-(Constants.controllerB.getRightTriggerAxis()-Constants.controllerB.getLeftTriggerAxis()), -0.25, 1));
      Constants.gripperMotorB.set(Functions.Clamp((Constants.controllerB.getRightTriggerAxis()-Constants.controllerB.getLeftTriggerAxis()), -0.25, 1));
    }

    if(Constants.controllerB.getStartButtonPressed())
    {
      RPS = true;
      RPSValue = (int)(3*Math.random());
      RPSTime = 0;
      RobotContainer.armSystem.setTargetToCurrent();
    }
    if(Constants.controllerB.getStartButton())
    {
      RPSTime += 0.02;
      RobotContainer.armSystem.RockPaperScissorsPeriodic(RPSTime, RPSValue);
    }
    if(Constants.controllerB.getStartButtonReleased())
    {
      Constants.gripperMotorA.set(0);
      Constants.gripperMotorB.set(0);
      RobotContainer.armSystem.setTargetToCurrent();
      RPS = false;
    }

    if(Constants.controllerB.getRightBumperPressed()){
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
      DriverDisplay.demoTargetAngleSelector.setDouble(RobotContainer.driveTrain.robotYawFieldRelative);
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
    if(isAutoTimeBetween(0, 0.04)) //first 0.04 seconds
    {
      Functions.KillAllSwerve();
      Functions.KillAllArm();
      Constants.primaryAccelerometer.setYaw(0);
    }
    else if(isAutoTimeBetween(0.04, 1.04)) //next 1 seconds
    {
      RobotContainer.driveTrain.DriveDriverOrientedAtAngle(0,0.5,0, 0.5);
    }
    else
    {
      Functions.KillAllArm();
      Functions.KillAllSwerve();
    }
  }
  public void autoSequence2()
  {
    if(isAutoTimeBetween(0, 0.04)) //first 0.04 second
    {
      Functions.KillAllSwerve();
      Functions.KillAllArm();
      Constants.primaryAccelerometer.setYaw(0);
    }
    else if(isAutoTimeBetween(0.04, 1.64)) //next 1 seconds
    {
      RobotContainer.driveTrain.DriveDriverOrientedAtAngle(0,0.4,0, 0.5);
    }
    else if(isAutoTimeBetween(1.64, 15)) //remainder
    {
      RobotContainer.driveTrain.AutoBalance();
    }
    else
    {
      Functions.KillAllArm();
      Functions.KillAllSwerve();
    }
  }
  public void autoSequence3() 
  {
    if(isAutoTimeBetween(0, 0.04)) //first 0.04 second
    {
      Functions.KillAllSwerve();
      Functions.KillAllArm();
      Constants.primaryAccelerometer.setYaw(180);
    }
    else if(isAutoTimeBetween(0.04, 5))
    {
      RobotContainer.armSystem.moveArmToAngles(Constants.armDepositAngle1, Constants.armDepositAngle2);
    }
    else if(isAutoTimeBetween(5, 5.5))
    {
      Constants.gripperMotorA.set(-1);
      Constants.gripperMotorB.set(1);
    }
    else if(isAutoTimeBetween(5.5, 10.5))
    {
      Constants.gripperMotorA.set(0);
      Constants.gripperMotorB.set(0);
      RobotContainer.armSystem.moveArmToAngles(Constants.armRestingAngle1, Constants.armRestingAngle2);
    }
    else
    {
      Functions.KillAllSwerve();
      Functions.KillAllArm();
    }
  }
  public void autoSequence4()
  {
    if(isAutoTimeBetween(0, 0.04)) //first 0.04 second
    {
      Functions.KillAllSwerve();
      Functions.KillAllArm();
      Constants.primaryAccelerometer.setYaw(180);
    }
    else if(isAutoTimeBetween(0.04, 5))
    {
      RobotContainer.armSystem.moveArmToAngles(Constants.armDepositAngle1, Constants.armDepositAngle2);
    }
    else if(isAutoTimeBetween(5, 5.5))
    {
      Constants.gripperMotorA.set(-1);
      Constants.gripperMotorB.set(1);
    }
    else if(isAutoTimeBetween(5.5, 8.5))
    {
      Constants.gripperMotorA.set(0);
      Constants.gripperMotorB.set(0);
      RobotContainer.driveTrain.DriveDriverOrientedAtAngle(0,0.5,180, 0.5);
    }
    else
    {
      Functions.KillAllSwerve();
      Functions.KillAllArm();
    }
  }
  public void autoSequence5() 
  {
    if(isAutoTimeBetween(0, 0.04)) //first 0.04 second
    {
      Functions.KillAllSwerve();
      Functions.KillAllArm();
      Constants.primaryAccelerometer.setYaw(180);
    }
    else if(isAutoTimeBetween(0.04, 3))
    {
      RobotContainer.armSystem.moveArmToAngles(Constants.armDepositAngle1, Constants.armDepositAngle2);
    }
    else if(isAutoTimeBetween(3, 3.5  ))
    {
      Constants.gripperMotorA.set(-1);
      Constants.gripperMotorB.set(1);
    }
    else if(isAutoTimeBetween(3.5, 8.5))
    {
      Constants.gripperMotorA.set(0);
      Constants.gripperMotorB.set(0);
      RobotContainer.armSystem.moveArmToAngles(Constants.armRestingAngle1, Constants.armRestingAngle2);
    }
    else if(isAutoTimeBetween(8.5, 10.1))
    {
      RobotContainer.driveTrain.DriveDriverOrientedAtAngle(0,0.4,180, 0.5);
    }
    else
    {
      Functions.KillAllSwerve();
      Functions.KillAllArm();
    }
  }
  public void autoSequence6()
  {
    if(isAutoTimeBetween(0, 0.04)) //first 0.04 second
    {
      Functions.KillAllSwerve();
      Functions.KillAllArm();
      Constants.primaryAccelerometer.setYaw(180);
    }
    else if(isAutoTimeBetween(0.04, 3))
    {
      RobotContainer.armSystem.moveArmToAngles(Constants.armDepositAngle1, Constants.armDepositAngle2);
    }
    else if(isAutoTimeBetween(3, 3.5  ))
    {
      Constants.gripperMotorA.set(-1);
      Constants.gripperMotorB.set(1);
    }
    else if(isAutoTimeBetween(3.5, 8.5))
    {
      Constants.gripperMotorA.set(0);
      Constants.gripperMotorB.set(0);
      RobotContainer.armSystem.moveArmToAngles((Constants.armRestingAngle1), Constants.armRestingAngle2);
    }
    else if(isAutoTimeBetween(8.5, 10.1))
    {
      RobotContainer.driveTrain.DriveDriverOrientedAtAngle(0,0.4,180, 0.5);
    }
    else if(isAutoTimeBetween(10.1, 15)) //remainder
    {
      RobotContainer.driveTrain.AutoBalance();
    }
    else
    {
      Functions.KillAllSwerve();
      Functions.KillAllArm();
    }
  }
}
