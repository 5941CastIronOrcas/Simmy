// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class ExperimentalSubsystem extends SubsystemBase {

  double XVelocity = 0;
  double ZVelocity = 0;
  boolean demo = false;
  public ExperimentalSubsystem() 
  {
    SmartDashboard.putBoolean("Demo In", demo);
  }
  @Override
  public void periodic() {
    // double[] q = new double[4];
    // short[] a = new short[3];
    // double[] g = new double[3];
    // Constants.primaryAccelerometer.get6dQuaternion(q);
    // Constants.primaryAccelerometer.getBiasedAccelerometer(a);
    // Constants.primaryAccelerometer.getGravityVector(g);
    // double XAccel = g[1];
    // XVelocity += (XAccel * 0.02);
    // double ZAccel = g[0];
    // ZVelocity += (ZAccel * 0.02);
    // SmartDashboard.putNumberArray("quater", q);
    // SmartDashboard.putNumberArray("grav", g);
    // SmartDashboard.putNumber("XAccel", XAccel);
    // SmartDashboard.putNumber("XSpeed", XVelocity);
    // SmartDashboard.putNumber("ZAccel", ZAccel);
    // SmartDashboard.putNumber("ZSpeed", ZVelocity);
    //SmartDashboard.putNumberArray("bias", (double[])a);
    //SmartDashboard.putNumber("FidId", VisionSubsystem.obtainTargets().getFiducialId());
    /*if (VisionSubsystem.getEstimatedGlobalPose().isPresent()) {
      SmartDashboard.putBoolean("Pose present", true);
      SmartDashboard.putNumber("Estimated X Position", VisionSubsystem.getEstimatedGlobalPose().get().getX());
      SmartDashboard.putNumber("Estimated Y Position", VisionSubsystem.getEstimatedGlobalPose().get().getY());
      SmartDashboard.putNumber("FR Enc Vel", Constants.frontRightDriveMotor.getEncoder().getVelocity());
      SmartDashboard.putNumber("Confield X", VisionSubsystem.conFieldX);
      SmartDashboard.putNumber("Confield Y", VisionSubsystem.conFieldY);
      SmartDashboard.putNumber("April Yaw Deg", VisionSubsystem.aprilYawAngle);
    }
    else {
      SmartDashboard.putBoolean("Pose present", false);
      SmartDashboard.putNumber("Estimated X Position", -1);
      SmartDashboard.putNumber("Estimated Y Position", -1);
    }*/
    demo = SmartDashboard.getBoolean("Demo In", false);
    SmartDashboard.putBoolean("Demo Out", demo);
    SmartDashboard.putNumber("FRM Position", Constants.frontRightDriveMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Pitch Angle", Constants.primaryAccelerometer.getRoll());
    
  }

  @Override
  public void simulationPeriodic() {}
}
