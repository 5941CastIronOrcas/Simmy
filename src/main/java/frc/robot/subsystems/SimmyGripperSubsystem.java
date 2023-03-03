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
    if(Constants.gripperMotorA.getOutputCurrent() < Constants.coneAmpLimit){
      Constants.gripperMotorA.set(Constants.coneGripperPowerIn);
    }else{
      Constants.gripperMotorA.set(0);
    }
    if(Constants.gripperMotorB.getOutputCurrent() < Constants.coneAmpLimit){
      Constants.gripperMotorB.set(Constants.coneGripperPowerIn);
    }else{
      Constants.gripperMotorB.set(0);
    }
  }

  public void releaseCone()
  {
    Constants.gripperMotorA.set(-Constants.coneGripperPowerOut);
    Constants.gripperMotorB.set(-Constants.coneGripperPowerOut);
  }

  public void intakeCube()
  {
    if(Constants.gripperMotorA.getOutputCurrent() < Constants.cubeAmpLimit){
      Constants.gripperMotorA.set(Constants.cubeGripperPowerIn);
    }else{
      Constants.gripperMotorA.set(0);
    }
    if(Constants.gripperMotorB.getOutputCurrent() < Constants.cubeAmpLimit){
      Constants.gripperMotorB.set(Constants.cubeGripperPowerIn);
    }else{
      Constants.gripperMotorB.set(0);
    }
  }

  public void releaseCube()
  {
    Constants.gripperMotorA.set(-Constants.cubeGripperPowerOut);
    Constants.gripperMotorB.set(-Constants.cubeGripperPowerOut);
  }

  public void stopGripper()
  {
    Constants.gripperMotorA.set(0);
    Constants.gripperMotorB.set(0);
  }
}
