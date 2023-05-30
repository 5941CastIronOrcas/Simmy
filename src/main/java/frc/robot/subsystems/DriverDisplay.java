// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriverDisplay extends SubsystemBase {

  public static ShuffleboardTab mainTab = Shuffleboard.getTab("mainTab");
  public static GenericEntry autoSequenceSelector = mainTab.add("Selected Auto Sequence", Constants.defaultAutoSequence).getEntry();
  public static GenericEntry demoTargetXSelector = mainTab.add("Target X", 0).getEntry();
  public static GenericEntry demoTargetYSelector = mainTab.add("Target Y", 0).getEntry();
  public static GenericEntry demoTargetAngleSelector = mainTab.add("Target Angle", 0).getEntry();
  public DriverDisplay() {}

  @Override
  public void periodic() {
    
  }
}
