// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExperimentalSubsystem extends SubsystemBase {

  public ExperimentalSubsystem() {}

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Accelerometer Pitch", Constants.primaryAccelerometer.getPitch());
    SmartDashboard.putNumber("Accelerometer Pitch", Constants.primaryAccelerometer.getRoll());
    SmartDashboard.putNumber("Accelerometer Pitch", Constants.primaryAccelerometer.getYaw());
    
  }

  @Override
  public void simulationPeriodic() {}
}
