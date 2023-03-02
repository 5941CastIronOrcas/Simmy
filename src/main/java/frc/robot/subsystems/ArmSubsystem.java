// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public double leftStickY = 0;
  public double rightStickY = 0;
  public float controllerSensitivity = 1;

  //constants
  public double mountX = Constants.armOriginHorizontalOffset;
  public double mountY = Constants.armOriginVerticalOffset;
  public double segment1Length = Constants.armSegmentALength;
  public double segment2Length = Constants.armSegmentBLength;

  //data
  public double raiseAngle = 0;
  public double bendAngle = 0; 
  public double[] clawPosition = new double[2];
  
  public double targetX; //position of target, in cm
  public double targetY; 
  public double targetDistance; //distance of target from mount
  public double targetAngle; //angle of target from mount

  public ArmSubsystem() {}

  @Override
  public void periodic() {
    raiseAngle = Constants.armMotor1.getEncoder().getPosition() * 360 * Constants.armGearRatio1;
    bendAngle = (Constants.armMotor2.getEncoder().getPosition() * 360 * Constants.armGearRatio2) + 180; //if it doesn't work check the 180
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void clampTargets(){ //call when setting target
    //easy clamp
    //targetX = Math.max(Math.min(targetX, segment1Length + segment2Length), 0);
    //targetY = Math.max(Math.min(targetY, segment1Length + segment2Length), 0);

    /* sad I am not using these because they took forever to write.
    if(rightStickY * rightStickY > leftStickY * leftStickY){
      targetX = Math.min(targetX,((targetY - mountY) / (Math.tan(Math.asin((targetY - mountY) / (segment1Length + segment2Length))))) + mountX);
    }else{
      targetY = Math.max(Math.min(targetY, Math.tan(Math.acos((targetX - mountX)/(segment1Length + segment2Length))) * (targetX - mountX) + mountY), Math.tan(-Math.acos((targetX - mountX)/(segment1Length + segment2Length))) * (targetX - mountX) + mountY);
    }
    */
    if((targetY - mountY) * (targetY - mountY) + (targetX - mountX) * (targetX - mountX) > (segment1Length + segment2Length) * (segment1Length + segment2Length)){
      double angle = Math.atan2(targetY - mountY, targetX - mountX);
      targetX = Math.cos(angle) * (segment1Length + segment2Length) + mountX;
      targetY = Math.sin(angle) * (segment1Length + segment2Length) + mountY;
    }
    if(targetY > segment1Length - segment2Length + mountY){
      targetX = Math.max(targetX, Math.cos(Math.asin((targetY - segment1Length - mountY) / segment2Length)) * segment2Length + mountX);
    }
    targetY = Math.max(targetY, 0); //set minimum height later
  }

  //converts angle of arm segments 1 & 2 to the current position of the end of the arm.
  public double[] angleToPosition(double cRaise, double cBend){
    double cDistance = Math.sqrt((segment1Length * segment1Length) + (segment2Length * segment2Length) - (2 * segment1Length * segment2Length * Math.cos(Math.toRadians(cBend))));
    double cAngle = cRaise - Math.toDegrees(Math.asin((segment2Length * Math.sin(Math.toRadians(cBend)))/(cDistance)));
    return new double[]{((Math.cos(Math.toRadians(cAngle)) * cDistance) + mountX), ((Math.sin(Math.toRadians(cAngle)) * cDistance) + mountY)};
  }

  //Finds the new target angles given a targetX and targetY
  public double[] changeInAngle(double cRaise, double cBend, double targetX, double targetY){
    double[] angleChange = positionToAngle(targetX, targetY);
    angleChange[0] -= cRaise;
    angleChange[1] -= cBend;
    return angleChange;
  }

  public double[] positionToAngle(double positionX, double positionY){
    targetDistance = Math.sqrt(((positionX-mountX) * (positionX-mountX)) + ((positionY-mountY) * (positionY-mountY))); //distance between arm and target. Needs targetX, targetY, mountX, mountY
    targetAngle = -(Math.atan2((positionX-mountX), (positionY-mountY)) * (180 / Math.PI)) + 90; //the angle of the target from the robot. Needs targetX, targetY, mountX, mountY
    //returns raise angle, bend angle
    return new double[]{(targetAngle + Math.acos(((segment1Length*segment1Length) - (segment2Length*segment2Length) + (targetDistance*targetDistance))/(2 * segment1Length * targetDistance)) * (180/Math.PI)), (Math.acos(((segment1Length*segment1Length) + (segment2Length*segment2Length) - (targetDistance*targetDistance))/(2 * segment1Length * segment2Length)) * (180/Math.PI))};
  }

  public void resetArmAngles(){
    Constants.armMotor1.getEncoder().setPosition(0);
    Constants.armMotor2.getEncoder().setPosition(0);
  }

  public void moveArm(double S1, double S2)
  {
    Constants.armMotor1.set(S1);
    Constants.armMotor2.set(S2);
  }

  public void moveArmToTarget(){
    moveArm(Constants.armSegment1PMult * (positionToAngle(targetX, targetY)[0]-raiseAngle), Constants.armSegment2PMult * (positionToAngle(targetX, targetY)[1]-bendAngle));
  }

  public void updateTarget(double x, double y){
    targetX += x * Constants.armSpeedMult;
    targetY += y * Constants.armSpeedMult;
    clampTargets();
  }
}
