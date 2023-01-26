// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public PneumaticsSubsystem() {}

  @Override
  public void periodic() {
    if(Constants.controller.getBButton())
    {
        Constants.demoCompressor.enableDigital();
    }
    else
    {
        Constants.demoCompressor.disable();
    }
    
    if(Constants.controller.getXButton())
    {
        Constants.demoSolenoid.set(true);
        Constants.demoDoubleSolenoid.set(Value.kForward);
    }
    else if(Constants.controller.getYButton())
    {
        Constants.demoSolenoid.set(true);
        Constants.demoDoubleSolenoid.set(Value.kReverse);
    }
    else 
    {
        Constants.demoSolenoid.set(false);
        Constants.demoDoubleSolenoid.set(Value.kOff);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
