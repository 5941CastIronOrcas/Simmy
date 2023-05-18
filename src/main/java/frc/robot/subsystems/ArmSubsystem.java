// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  //data
  public double raiseAngle = 0; //current raise angle of the arm, in degrees. Horizontal is 0 degrees, down is negative, up is positive. 
  public double bendAngle = 0; //lower angle between the first arm segment and second arm segment. Arm straight is 180, arm bent is 90. 
  public double segment2HorizonAngle; //angle of second arm relative to the horizon. 

  public ArmSubsystem() {}

  @Override
  public void periodic() {
    raiseAngle = Constants.armMotor1.getEncoder().getPosition() * 360 * Constants.armGearRatio1; //sets current raise angle to encoder positions.
    segment2HorizonAngle = (Constants.armMotor2.getEncoder().getPosition() * 360 * Constants.armGearRatio2); //if it doesn't work check the 180
    bendAngle = segment2HorizonAngle - raiseAngle; //sets the current bend angle based on the encoder positions.
    
    SmartDashboard.putNumber("S1 Angle", raiseAngle);
    SmartDashboard.putNumber("S2 Angle", bendAngle);
    SmartDashboard.putNumber("HorizonAngle", segment2HorizonAngle);
    SmartDashboard.putNumber("S1 Speed", Constants.armMotor1.getEncoder().getVelocity());
    SmartDashboard.putNumber("S2 Speed", Constants.armMotor2.getEncoder().getVelocity());
    // This method will be called once per scheduler run
  }

  //calibrates motor encoders and sets angles to predetermined values. 
  public void resetArmAngles(){
    Constants.armMotor1.getEncoder().setPosition((Constants.raiseRestingAngle / Constants.armGearRatio1) / 360);
    Constants.armMotor2.getEncoder().setPosition((Constants.bendRestingAngle / Constants.armGearRatio2) / 360);
  }

  public void setTargetToCurrent()
  {

  }

  public void moveArmToAngles(double a1, double a2)
  {
    moveArm(Functions.Clamp((Constants.armSegment1PMult * (a1 - raiseAngle))
    +(Constants.armSegment1GravMult * Math.cos(Math.toRadians(raiseAngle)))
    +(Constants.armSegment1DMult*Constants.armMotor1.getEncoder().getVelocity()), 
    -Constants.maxArmSpeed, Constants.maxArmSpeed), 
            Functions.Clamp((Constants.armSegment2PMult * (a2 - segment2HorizonAngle))
    +(Constants.armSegment2GravMult * -Math.cos(Math.toRadians(segment2HorizonAngle)))
    +(Constants.armSegment2DMult*Constants.armMotor2.getEncoder().getVelocity()), 
    -Constants.maxArmSpeed, Constants.maxArmSpeed)
    );
    SmartDashboard.putNumber("S1 Target", a1);
    SmartDashboard.putNumber("S2 Target", a2);
  }

  //moves the motors directly, inverts motor movements if set to.
  public void moveArm(double S1, double S2)
  {
    Constants.armMotor1.set(Constants.raiseMotorInverted ? -S1 : S1);
    Constants.armMotor2.set(Constants.bendMotorInverted ? -S2 : S2);
    SmartDashboard.putNumber("S1 output", Constants.raiseMotorInverted ? -S1 : S1);
    SmartDashboard.putNumber("S2 output", Constants.bendMotorInverted ? -S2 : S2);
  }
  public void RockPaperScissorsPeriodic(double timeSinceStart, int id)
  {
    if(timeSinceStart < 2.0*Math.PI)
    {
      
    }
    else
    {
      switch(id)
      {
        case 1:
          Constants.gripperMotorA.set(0.5);
          break;
        case 2:
          Constants.gripperMotorA.set(0.5);
          Constants.gripperMotorB.set(0.5);
          break;
        default:
          Constants.gripperMotorA.set(0);
          Constants.gripperMotorB.set(0);
          break;
      }
    }
  }
}
