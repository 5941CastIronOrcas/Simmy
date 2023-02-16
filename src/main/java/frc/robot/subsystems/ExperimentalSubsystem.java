// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExperimentalSubsystem extends SubsystemBase {

  public ExperimentalSubsystem() {}

    double XVelocity = 0;
    double ZVelocity = 0;
  @Override
  public void periodic() {
    double[] q = new double[4];
    short[] a = new short[3];
    double[] g = new double[3];
    Constants.primaryAccelerometer.get6dQuaternion(q);
    Constants.primaryAccelerometer.getBiasedAccelerometer(a);
    Constants.primaryAccelerometer.getGravityVector(g);
    double XAccel = g[1];
    XVelocity += (XAccel * 0.02);
    double ZAccel = g[0];
    ZVelocity += (ZAccel * 0.02);
    SmartDashboard.putNumberArray("quater", q);
    SmartDashboard.putNumberArray("grav", g);
    SmartDashboard.putNumber("XAccel", XAccel);
    SmartDashboard.putNumber("XSpeed", XVelocity);
    SmartDashboard.putNumber("ZAccel", ZAccel);
    SmartDashboard.putNumber("ZSpeed", ZVelocity);
    //SmartDashboard.putNumberArray("bias", (double[])a);
  }

  @Override
  public void simulationPeriodic() {}
}
