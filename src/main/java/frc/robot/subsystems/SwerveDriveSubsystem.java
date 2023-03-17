package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;

public class SwerveDriveSubsystem extends SubsystemBase {
    
    //F = front
    //R = right
    //B = back
    //L = left
    //A = angle
    //T = throttle

    double FRX, FRY, FLX, FLY, BRX, BRY, BLX, BLY;

    double FRA, FLA, BRA, BLA;
    
    double FRT, FLT, BRT, BLT;

    double FRTOutput, FLTOutput, BRTOutput, BLTOutput;

    //double LSX, LSY, RSX;

    double highestSpeed = 0;
    //boolean crouchMode = true;
    
    public double robotYawAngle = 0, robotPitchAngle = 0, robotYawFieldRelative;

    SwerveModule frontRightModule = new SwerveModule(Constants.frontRightAngleMotor, true, Constants.frontRightDriveMotor, false, Constants.frontRightEncoder);
    SwerveModule frontLeftModule = new SwerveModule(Constants.frontLeftAngleMotor, true, Constants.frontLeftDriveMotor, true, Constants.frontLeftEncoder);
    SwerveModule backRightModule = new SwerveModule(Constants.backRightAngleMotor, true, Constants.backRightDriveMotor, false, Constants.backRightEncoder);
    SwerveModule backLeftModule = new SwerveModule(Constants.backLeftAngleMotor, true, Constants.backLeftDriveMotor, true, Constants.backLeftEncoder);

    public SwerveDriveSubsystem() //Constructor (Init)
    {

    }

    @Override
    public void periodic() //Every 0.02 sec (50 FPS)
    {
        robotYawAngle = Functions.DeltaAngleDegrees(0, -Constants.primaryAccelerometer.getYaw());
        if(DriverStation.getAlliance() == DriverStation.Alliance.Red)
        {robotYawFieldRelative = Functions.DeltaAngleDegrees(0, robotYawAngle - 90);}
        else if(DriverStation.getAlliance() == DriverStation.Alliance.Blue)
        {robotYawFieldRelative = Functions.DeltaAngleDegrees(0, robotYawAngle + 90);}
        else
        {robotYawFieldRelative = robotYawAngle;}
        
        robotPitchAngle = Constants.primaryAccelerometer.getRoll();
        SmartDashboard.putNumber("Robot Yaw", robotYawAngle);
        SmartDashboard.putNumber("RobotFieldYaw", robotYawFieldRelative);
        
    }

    public void DriveTo(double x, double y, double angle)
    {
        DriveFIELDOrientedAtAngle(
            Functions.Clamp(
                Constants.swerveDriveToPMult*(x-VisionSubsystem.conFieldX), 
                -Constants.swerveDriveToMaxSpeed, Constants.swerveDriveToMaxSpeed), 
            Functions.Clamp(Constants.swerveDriveToPMult*(y-VisionSubsystem.conFieldY), 
                -Constants.swerveDriveToMaxSpeed, Constants.swerveDriveToMaxSpeed), 
            angle);
    }

    public void DriveFIELDOrientedAtAngle(double x, double y, double angle)
    {
        DriveFieldOriented(x, y, Functions.Clamp(-Constants.swerveAutoTurnPMult*Functions.DeltaAngleDegrees(angle, robotYawFieldRelative), -Constants.swerveAutoTurnMaxSpeed, Constants.swerveAutoTurnMaxSpeed));
    }

    public void DriveFIELDOriented(double x, double y, double turn)
    {
        Drive(x*Math.cos(Math.toRadians(-robotYawFieldRelative))+y*Math.sin(Math.toRadians(-robotYawFieldRelative)), y*Math.cos(Math.toRadians(-robotYawFieldRelative))+x*Math.sin(Math.toRadians(robotYawFieldRelative)), turn);
    }

    public void DriveFieldOrientedAtAngle(double LSX, double LSY, double angle)
    {
        DriveFieldOriented(LSX, LSY, Functions.Clamp(-Constants.swerveAutoTurnPMult*Functions.DeltaAngleDegrees(angle, robotYawAngle), -Constants.swerveAutoTurnMaxSpeed, Constants.swerveAutoTurnMaxSpeed));
    }

    public void DriveFieldOriented(double LSX, double LSY, double RSX)
    {
        Drive(LSX*Math.cos(Math.toRadians(-robotYawAngle))+LSY*Math.sin(Math.toRadians(-robotYawAngle)), LSY*Math.cos(Math.toRadians(-robotYawAngle))+LSX*Math.sin(Math.toRadians(robotYawAngle)), RSX);
    }
    
    public void Drive(double LSX, double LSY, double RSX)
    {   
            //set the target X and Y speeds based on controller input
            FRX = RSX + LSX;
            FRY = -RSX + LSY;
            FLX = RSX + LSX;
            FLY = RSX + LSY;
            BRX = -RSX + LSX;
            BRY = -RSX + LSY;
            BLX = -RSX + LSX;
            BLY = RSX + LSY;

            //set the target angles based on the target X and Y speeds
            FRA = Math.atan2(FRX, FRY);
            FLA = Math.atan2(FLX, FLY);
            BRA = Math.atan2(BRX, BRY);
            BLA = Math.atan2(BLX, BLY);

            //set the target speeds based on the target X and Y speeds
            FRT = Math.abs(FRX)+Math.abs(FRY);
            FLT = Math.abs(FLX)+Math.abs(FLY);
            BRT = Math.abs(BRX)+Math.abs(BRY);
            BLT = Math.abs(BLX)+Math.abs(BLY);

            //ensure none of the modules are expected to go a speed > 1.0
            highestSpeed = Math.max(Math.max(FRT, FLT), Math.max(BRT, Math.max(BLT, 1)));
            FRT = (Math.abs(FRX)+Math.abs(FRY)) / highestSpeed;
            FLT = (Math.abs(FLX)+Math.abs(FLY)) / highestSpeed;
            BRT = (Math.abs(BRX)+Math.abs(BRY)) / highestSpeed;
            BLT = (Math.abs(BLX)+Math.abs(BLY)) / highestSpeed;

            //Limit the drive motor acceleration
            double maxThrottleChange = Constants.swerveModuleMaxThrottleChange;
            FRTOutput += Functions.Clamp(FRT-FRTOutput, -maxThrottleChange, maxThrottleChange);
            FLTOutput += Functions.Clamp(FLT-FLTOutput, -maxThrottleChange, maxThrottleChange);
            BRTOutput += Functions.Clamp(BRT-BRTOutput, -maxThrottleChange, maxThrottleChange);
            BLTOutput += Functions.Clamp(BLT-BLTOutput, -maxThrottleChange, maxThrottleChange);

            //Tell the modules to angle and drive themselves
            frontRightModule.Drive(FRA, FRTOutput);
            frontLeftModule.Drive(FLA, FLTOutput);
            backRightModule.Drive(BRA, BRTOutput);
            backLeftModule.Drive(BLA, BLTOutput);
    }

    public void AutoBalance()
    {
        if(robotPitchAngle > 5)
        {
            Drive(0,0.0625,0);
        }
        else if(robotPitchAngle < -5)
        {
            Drive(0,-0.0625,0);
        }
        else
        {
            Drive(0,0,0);
        }
    }
    
}
