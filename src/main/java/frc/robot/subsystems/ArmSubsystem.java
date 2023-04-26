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

  //constants
  public double mountX = Constants.armOriginHorizontalOffset;
  public double mountY = Constants.armOriginVerticalOffset;
  public double segment1Length = Constants.armSegmentALength;
  public double segment2Length = Constants.armSegmentBLength;

  //data
  public double raiseAngle = 0; //current raise angle of the arm, in degrees. Horizontal is 0 degrees, down is negative, up is positive. 
  public double bendAngle = 0; //lower angle between the first arm segment and second arm segment. Arm straight is 180, arm bent is 90. 
  public double segment2HorizonAngle; //angle of second arm relative to the horizon. 
  public double[] clawPosition = new double[2]; //currently not used
  
  public double targetX; //position of target x relative to the front of the bumper, in cm 
  public double targetY; //position of target y relative to the floor, in cm.
  public double targetDistance; //distance between target and mounting point, in cm.
  public double targetAngle; //angle of target from mount, relative to horizon. 

  public ArmSubsystem() {}

  @Override
  public void periodic() {
    raiseAngle = Constants.armMotor1.getEncoder().getPosition() * 360 * Constants.armGearRatio1; //sets current raise angle to encoder positions.
    segment2HorizonAngle = (Constants.armMotor2.getEncoder().getPosition() * 360 * Constants.armGearRatio2) + 180; //if it doesn't work check the 180
    bendAngle = segment2HorizonAngle - raiseAngle; //sets the current bend angle based on the encoder positions.
    
    SmartDashboard.putNumber("S1 Angle", raiseAngle);
    SmartDashboard.putNumber("S2 Angle", bendAngle);
    SmartDashboard.putNumber("Arm Target X", targetX);
    SmartDashboard.putNumber("Arm Target Y", targetY);
    SmartDashboard.putNumber("Arm Current X", angleToPosition(raiseAngle, bendAngle)[0]);
    SmartDashboard.putNumber("Arm Current Y", angleToPosition(raiseAngle, bendAngle)[1]);
    SmartDashboard.putNumber("Target Angle 1", positionToAngle(targetX, targetY)[0]);
    SmartDashboard.putNumber("Target Angle 2", positionToAngle(targetX, targetY)[1]);
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //call when setting target. Clamps the target from leaving effective range, clamps the target from hitting the floor, clamps the target from reaching ceiling.
  public void clampTargets(){

    if(Math.sqrt(((targetY - mountY) * (targetY - mountY)) + ((targetX - mountX) * (targetX - mountX))) > Constants.armExtendBuffer * (segment1Length + segment2Length)){
      double angle = Math.atan2(targetY - mountY, targetX - mountX);
      targetX = (Math.cos(angle) * (segment1Length + segment2Length) * Constants.armExtendBuffer) + mountX;
      targetY = (Math.sin(angle) * (segment1Length + segment2Length) * Constants.armExtendBuffer) + mountY;
    }

    targetY = Functions.Clamp(targetY, Constants.clawMinHeight, Constants.clawMaxHeight); //clamp to keep from hiting the floor, and ceiling
    targetX = Functions.Clamp(targetX, -80, 93.5); //adjust later if not working, clamp to limit extension range
  }

  //converts angle of arm segments 1 & 2 to the current position of the end of the arm.
  public double[] angleToPosition(double cRaise, double cBend){
    double cDistance = Math.sqrt((segment1Length * segment1Length) + (segment2Length * segment2Length) - (2 * segment1Length * segment2Length * Math.cos(Math.toRadians(cBend))));
    double cAngle = cRaise - Math.toDegrees(Math.asin((segment2Length * Math.sin(Math.toRadians(cBend)))/(cDistance)));
    return new double[]{((Math.cos(Math.toRadians(cAngle)) * cDistance) + mountX), ((Math.sin(Math.toRadians(cAngle)) * cDistance) + mountY)};
  }

  //takes x and y, returns bend and raise angles needed to reach it.
  public double[] positionToAngle(double positionX, double positionY){ 
    targetDistance = Math.sqrt(((positionX-mountX) * (positionX-mountX)) + ((positionY-mountY) * (positionY-mountY))); //distance between arm and target. Needs targetX, targetY, mountX, mountY
    targetAngle = -(Math.atan2((positionX-mountX), (positionY-mountY)) * (180 / Math.PI)) + 90; //the angle of the target from the robot. Needs targetX, targetY, mountX, mountY
    return new double[]{(targetAngle + Math.acos(((segment1Length*segment1Length) - (segment2Length*segment2Length) + (targetDistance*targetDistance))/(2 * segment1Length * targetDistance)) * (180/Math.PI)), (Math.acos(((segment1Length*segment1Length) + (segment2Length*segment2Length) - (targetDistance*targetDistance))/(2 * segment1Length * segment2Length)) * (180/Math.PI))};
  }

  //calibrates motor encoders and sets angles to predetermined values. 
  public void resetArmAngles(){
    Constants.armMotor1.getEncoder().setPosition((Constants.raiseRestingAngle / Constants.armGearRatio1) / 360);
    Constants.armMotor2.getEncoder().setPosition((Constants.bendRestingAngle / Constants.armGearRatio2) / 360);
  }

  //moves the motors directly, inverts motor movements if set to.
  public void moveArm(double S1, double S2)
  {
    Constants.armMotor1.set(Constants.raiseMotorInverted ? -S1 : S1);
    Constants.armMotor2.set(Constants.bendMotorInverted ? -S2 : S2);
    SmartDashboard.putNumber("S1 output", Constants.raiseMotorInverted ? -S1 : S1);
    SmartDashboard.putNumber("S2 output", Constants.bendMotorInverted ? -S2 : S2);
  }

  //moves arm to target
  public void moveArmToTarget(){
    //moveArm(Constants.armSegment1PMult * (positionToAngle(targetX, targetY)[0]-raiseAngle), Constants.armSegment2PMult * (positionToAngle(targetX, targetY)[1]-bendAngle));
    moveArmToAngle(positionToAngle(targetX, targetY)[0], positionToAngle(targetX, targetY)[1]);
  }
  
  //moves arm to angle, with PID
  public void moveArmToAngle(double A1, double A2)
  {
    moveArm(Functions.Clamp(Constants.armSegment1PMult * (Functions.DeltaAngleDegrees(A1, raiseAngle)), -Constants.maxArmSpeed, Constants.maxArmSpeed), Functions.Clamp(Constants.armSegment2PMult * (Functions.DeltaAngleDegrees(A2, bendAngle)), -Constants.maxArmSpeed, Constants.maxArmSpeed));
  }
  
  //moves target based on given ((x and y value) * (arm speed mult)) cm/sec based on a controller. Clamps target after finished.
  public void updateTarget(double x, double y){
    targetX += x * Constants.armSpeedMult * 0.02;
    targetY += y * Constants.armSpeedMult * 0.02;
    clampTargets();
  }

  // sets targets to a given x and y, both in cm. Clamps target after finished
  public void setTarget(double x, double y){
    targetX = x;
    targetY = y;
    clampTargets();
  }

  //sets target to current arm position. Essentially stops arm from moving to target.
  public void setTargetToCurrent()
  {
    targetX = angleToPosition(raiseAngle, bendAngle)[0];
    targetY = angleToPosition(raiseAngle, bendAngle)[1];
  }

  public void RockPaperScissorsPeriodic(double timeSinceStart, int id)
  {
    if(timeSinceStart < 2.0*Math.PI)
    {
      setTarget(45, 5*(Math.sin((4*timeSinceStart) - (Math.PI/2))) + 55);
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
