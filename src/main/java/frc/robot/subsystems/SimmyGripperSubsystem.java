// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SimmyGripperSubsystem extends SubsystemBase {
  

  public SimmyGripperSubsystem() {}

  @Override
  public void periodic() 
  {
    if(Constants.controllerB.getAButton())
    {

    }
  }

  @Override
  public void simulationPeriodic() 
  {

  }

  public void intakeCone()
  {
    Constants.gripperMotorA.set(0);
  }
}
