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
    
  }

  @Override
  public void simulationPeriodic() 
  {

  }

  public void intakeCone()
  {
    Constants.gripperMotorA.set(Constants.coneGripperPowerIn);
    Constants.gripperMotorB.set(Constants.coneGripperPowerIn);
  }

  public void releaseCone()
  {
    Constants.gripperMotorA.set(Constants.coneGripperPowerOut);
    Constants.gripperMotorB.set(Constants.coneGripperPowerOut);
  }

  public void intakeCube()
  {
    Constants.gripperMotorA.set(Constants.cubeGripperPowerIn);
    Constants.gripperMotorB.set(Constants.cubeGripperPowerIn);
  }

  public void releaseCube()
  {
    Constants.gripperMotorA.set(Constants.cubeGripperPowerOut);
    Constants.gripperMotorB.set(Constants.cubeGripperPowerOut);
  }
}
