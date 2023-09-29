
package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.RobotContainer;

import java.util.Arrays;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;


public class VisionSubsystem extends SubsystemBase {
  
  public static PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  public static double deltaX = 0;
  public static double deltaY = 0;
  public static double deltaPosition = 0;

  public static Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));

  public static AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(Arrays.asList(
    new AprilTag(1, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters( 42.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(2, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(3, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(4, new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(5, new Pose3d(Units.inchesToMeters( 14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, 0.0))),
    new AprilTag(6, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0))),
    new AprilTag(7, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0))),
    new AprilTag(8, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters( 42.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0)))
  ), Units.inchesToMeters(651.25), Units.inchesToMeters(315.5));

  public static PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);

  public static double conFieldX = 0;
  public static double conFieldY = 0;

  public static double aprilYawAngle;

  public static Pose2d estimatedGlobalPoseOld = new Pose2d();

  public VisionSubsystem() {}
  
  public static Boolean camCheck() {
    var result = camera.getLatestResult();

    return result.hasTargets();
  }

  public static PhotonTrackedTarget obtainTargets() {
    var result = camera.getLatestResult();

    if  (result.hasTargets()) {
      //Sends back the most clear target and its data
      return result.getBestTarget();
    } 
    else {
      return new PhotonTrackedTarget();
    }
  }

  public static Pose2d getEstimatedGlobalPose() {
    //var emptyTarget = new PhotonTrackedTarget();
    if (!camCheck()) {
      return estimatedGlobalPoseOld;
    }
    
    /*if (photonPoseEstimator.update().isPresent()) {
      return photonPoseEstimator.update().orElse(null).estimatedPose.toPose2d();
    }*/
    
    
    try{
      if (photonPoseEstimator.update().isPresent()) {
        return photonPoseEstimator.update().get().estimatedPose.toPose2d();
      }
    }
    catch(Exception e)
    {
      System.out.println("Caught Error: " + e);
    }
    
    return estimatedGlobalPoseOld;
  }

  public boolean isValid(Pose2d oldPose, Pose2d newPose)
  {
    double maxMovement = Constants.swerveMaxSpeed * camera.getLatestResult().getLatencyMillis() * 0.001;
    double currentMovement = Functions.Pythagorean(oldPose.getX() - newPose.getX(), oldPose.getY() - newPose.getY());
    if (currentMovement > maxMovement)
    {
      return false;
    }
    else 
    {
      return true;
    }
  }

  @Override
  public void periodic() {
    // averages the 4 raw motor angle values, multiplies by the sin & cos of the angle of the motor to get each motors x and y velocity.
    //tries to estimate the change in position as the robot moves.
    //delta XY is meters/frame
    //sin(radian(angle + f_relative)) * velocity * ratio * 1/3000 * circumfrence)
    deltaX = ((
         (Math.sin(Math.toRadians(RobotContainer.driveTrain.frontRightModule.currentAngle + RobotContainer.driveTrain.robotYawFieldRelative)) * Constants.frontRightDriveMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference)
       + (Math.sin(Math.toRadians(RobotContainer.driveTrain.frontLeftModule.currentAngle + RobotContainer.driveTrain.robotYawFieldRelative)) * -Constants.frontLeftDriveMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference)
       + (Math.sin(Math.toRadians(RobotContainer.driveTrain.backRightModule.currentAngle + RobotContainer.driveTrain.robotYawFieldRelative)) * Constants.backRightDriveMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference)
       + (Math.sin(Math.toRadians(RobotContainer.driveTrain.backLeftModule.currentAngle + RobotContainer.driveTrain.robotYawFieldRelative)) * -Constants.backLeftDriveMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference))
        / 4.0);
    deltaY = ((
         (Math.cos(Math.toRadians(RobotContainer.driveTrain.frontRightModule.currentAngle + RobotContainer.driveTrain.robotYawFieldRelative)) * Constants.frontRightDriveMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference)
       + (Math.cos(Math.toRadians(RobotContainer.driveTrain.frontLeftModule.currentAngle + RobotContainer.driveTrain.robotYawFieldRelative)) * -Constants.frontLeftDriveMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference)
       + (Math.cos(Math.toRadians(RobotContainer.driveTrain.backRightModule.currentAngle + RobotContainer.driveTrain.robotYawFieldRelative)) * Constants.backRightDriveMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference)
       + (Math.cos(Math.toRadians(RobotContainer.driveTrain.backLeftModule.currentAngle + RobotContainer.driveTrain.robotYawFieldRelative)) * -Constants.backLeftDriveMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference))
        / 4.0);
    deltaPosition = Functions.Pythagorean(deltaX, deltaY);
        //uses one module
    //double tempX = ((Math.sin(Math.toRadians(RobotContainer.driveTrain.frontLeftModule.currentAngle)) * Constants.frontLeftDriveMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference));
    //double tempY = ((Math.cos(Math.toRadians(RobotContainer.driveTrain.frontLeftModule.currentAngle)) * Constants.frontLeftDriveMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference));
    Pose2d globalPose = estimatedGlobalPoseOld;
    if (camCheck()) {
      if (getEstimatedGlobalPose() != null) {
        globalPose = getEstimatedGlobalPose();
      }
    }
    if (camCheck()) {
      if (estimatedGlobalPoseOld != globalPose && isValid(estimatedGlobalPoseOld, globalPose)) {
        // apriltags present and information updated
        conFieldX = globalPose.getX();
        conFieldY = globalPose.getY();
        aprilYawAngle = globalPose.getRotation().getDegrees();
      }
      else {
      // apriltags present, information not updated
      conFieldX += Math.abs(deltaX) > 0.0001 ? deltaX : 0;
      conFieldY += Math.abs(deltaY) > 0.0001 ? deltaY : 0;
      }
    }
    else {
      // no apriltags detected
      conFieldX += Math.abs(deltaX) > 0.0001 ? deltaX : 0;
      conFieldY += Math.abs(deltaY) > 0.0001 ? deltaY : 0;
    }
    if (camCheck() && getEstimatedGlobalPose() != null) {
      estimatedGlobalPoseOld = getEstimatedGlobalPose();
    }
    //SmartDashboard.putBoolean("isPresent", camCheck());
    //SmartDashboard.putNumber("conField Y", conFieldY);
    //SmartDashboard.putNumber("conField X", conFieldX);
    //SmartDashboard.putNumber("Latency", camera.getLatestResult().getLatencyMillis());
  }


}

