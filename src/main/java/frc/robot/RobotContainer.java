// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriverDisplay;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ExperimentalSubsystem;
import frc.robot.subsystems.SimmyGripperSubsystem;
//import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem2D;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  
  public static final SwerveDriveSubsystem driveTrain = new SwerveDriveSubsystem();

  private final ExperimentalSubsystem experimental = new ExperimentalSubsystem();

  public static DriverDisplay driverDisplay = new DriverDisplay();

  public static final ArmSubsystem armSystem = new ArmSubsystem();

  public static final SimmyGripperSubsystem simmyGripperSystem = new SimmyGripperSubsystem();
  //private final PneumaticsSubsystem pneumaticsStuff = new PneumaticsSubsystem();

  

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final VisionSubsystem m_vision = new VisionSubsystem();

  private final VisionSubsystem2D m_vision2D = new VisionSubsystem2D();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
