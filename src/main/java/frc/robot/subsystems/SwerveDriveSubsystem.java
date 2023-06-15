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

    double FRX = 0, FRY = 0, FLX = 0, FLY = 0, BRX = 0, BRY = 0, BLX = 0, BLY = 0;
    double FRA, FLA, BRA, BLA;
    double FRT, FLT, BRT, BLT;
    double FRTOutput, FLTOutput, BRTOutput, BLTOutput;
    double highestSpeed = 0;

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
        robotPitchAngle = -Constants.primaryAccelerometer.getPitch() + Constants.primaryAccelerometerPitchOffset;

        //Adjusts robotYawFieldRelative depending on what alliance we're on
        if(DriverStation.getAlliance() == DriverStation.Alliance.Red)
        {robotYawFieldRelative = Functions.DeltaAngleDegrees(0, robotYawAngle - 90);}
        else if(DriverStation.getAlliance() == DriverStation.Alliance.Blue)
        {robotYawFieldRelative = Functions.DeltaAngleDegrees(0, robotYawAngle + 90);}
        else
        {robotYawFieldRelative = robotYawAngle;}
        
        SmartDashboard.putNumber("Pitch Angle", robotPitchAngle);
        SmartDashboard.putNumber("RobotFieldYaw", robotYawFieldRelative);
        
    }

    public void DriveTo(double x, double y, double angle, double speedLimit, double turnLimit)
    {
        /*DriveFieldOrientedAtAngle(
            Functions.DeadZone(Functions.Clamp(
                Constants.swerveDriveToPMult*(x-VisionSubsystem.conFieldX)
                -Constants.swerveDriveToDMult*(VisionSubsystem.deltaX), 
                -speedLimit, speedLimit), 0.05), 
                Functions.DeadZone(Functions.Clamp(Constants.swerveDriveToPMult*(y-VisionSubsystem.conFieldY)
            -Constants.swerveDriveToDMult*(VisionSubsystem.deltaY), 
                -speedLimit, speedLimit), 0.05), 
            angle);*/
        /*double pComponent = Functions.Clamp(Constants.swerveDriveToPMult*Functions.Pythagorean(x-VisionSubsystem.conFieldX, y-VisionSubsystem.conFieldY), 0, speedLimit);
        double angleToTarget = Math.atan2(x-VisionSubsystem.conFieldX, y-VisionSubsystem.conFieldY);
        double xComponent = Functions.DeadZone(pComponent * Math.sin(angleToTarget), 0.05);
        double yComponent = Functions.DeadZone(pComponent * Math.cos(angleToTarget), 0.05);
        DriveFieldOrientedAtAngle(
            Functions.Clamp(
                (xComponent)-Constants.swerveDriveToDMult*(VisionSubsystem.deltaX), 
                -speedLimit, speedLimit), 
                Functions.Clamp(
                (yComponent)-Constants.swerveDriveToDMult*(VisionSubsystem.deltaY), 
                -speedLimit, speedLimit), 
            angle);*/
            double angleToTarget = Math.atan2(x-VisionSubsystem.conFieldX, y-VisionSubsystem.conFieldY);
            double pComponent = Constants.swerveDriveToPMult*Functions.Pythagorean(x-VisionSubsystem.conFieldX, y-VisionSubsystem.conFieldY);
            double dComponent = Constants.swerveDriveToDMult*Functions.Pythagorean(VisionSubsystem.deltaX, VisionSubsystem.deltaY);
            double output = Functions.Clamp(pComponent - dComponent, 0, speedLimit);
            double xComponent = Functions.DeadZone(output * Math.sin(angleToTarget), Constants.swerveDriveToDeadZone);
            double yComponent = Functions.DeadZone(output * Math.cos(angleToTarget), Constants.swerveDriveToDeadZone);
            DriveFieldOrientedAtAngle(xComponent, yComponent, angle, turnLimit);
    }

    public void DriveFieldOrientedAtAngle(double x, double y, double angle, double turnLimit)
    {
        /*DriveDriverOriented(DriverStation.getAlliance() == DriverStation.Alliance.Red?y:-y, 
        DriverStation.getAlliance() == DriverStation.Alliance.Red?-x:x, 
        Functions.Clamp(-Constants.swerveAutoTurnPMult*Functions.DeltaAngleDegrees(angle, robotYawFieldRelative), 
        -Constants.swerveAutoTurnMaxSpeed*Functions.Clamp(turnLimit, 0, 1), 
        Constants.swerveAutoTurnMaxSpeed*Functions.Clamp(turnLimit, 0, 1)));*/
        DriveDriverOrientedAtAngle(DriverStation.getAlliance() == DriverStation.Alliance.Red?y:-y, 
        DriverStation.getAlliance() == DriverStation.Alliance.Red?-x:x, 
        Functions.FieldToDriverAngle(angle), turnLimit);
    }

    public void DriveFieldOriented(double x, double y, double turn)
    {
        Drive(x*Math.cos(Math.toRadians(-robotYawFieldRelative))+y*Math.sin(Math.toRadians(-robotYawFieldRelative)), y*Math.cos(Math.toRadians(-robotYawFieldRelative))+x*Math.sin(Math.toRadians(robotYawFieldRelative)), turn);
    }

    public void DriveDriverOrientedAtAngle(double LSX, double LSY, double angle, double turnLimit)
    {
        DriveDriverOriented(LSX, LSY, 
        Functions.Clamp(-Constants.swerveAutoTurnPMult*Functions.DeadZone(Functions.DeltaAngleDegrees(angle, robotYawAngle), Constants.swerveAutoTurnDeadZone), 
        -Constants.swerveAutoTurnMaxSpeed*Functions.Clamp(turnLimit, 0, 1), 
        Constants.swerveAutoTurnMaxSpeed*Functions.Clamp(turnLimit, 0, 1)));
    }

    public void DriveDriverOriented(double LSX, double LSY, double RSX)
    {
        Drive(LSX*Math.cos(Math.toRadians(-robotYawAngle))+LSY*Math.sin(Math.toRadians(-robotYawAngle)), LSY*Math.cos(Math.toRadians(-robotYawAngle))+LSX*Math.sin(Math.toRadians(robotYawAngle)), RSX);
    }
    
    public void Drive(double LSX, double LSY, double RSX)
    {   
            FRX += Functions.Clamp(RSX+LSX - FRX, -Constants.swerveMaxAccel, Constants.swerveMaxAccel);
            FRY += Functions.Clamp(-RSX+LSY - FRY, -Constants.swerveMaxAccel, Constants.swerveMaxAccel);;
            FLX += Functions.Clamp(RSX+LSX - FLX, -Constants.swerveMaxAccel, Constants.swerveMaxAccel);
            FLY += Functions.Clamp(RSX+LSY - FLY, -Constants.swerveMaxAccel, Constants.swerveMaxAccel);
            BRX += Functions.Clamp(-RSX+LSX - BRX, -Constants.swerveMaxAccel, Constants.swerveMaxAccel);
            BRY += Functions.Clamp(-RSX+LSY - BRY, -Constants.swerveMaxAccel, Constants.swerveMaxAccel);
            BLX += Functions.Clamp(-RSX+LSX - BLX, -Constants.swerveMaxAccel, Constants.swerveMaxAccel);
            BLY += Functions.Clamp(RSX+LSY - BLY, -Constants.swerveMaxAccel, Constants.swerveMaxAccel);

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
        double balanceAngle = robotPitchAngle;
        /*if(balanceAngle > 7)
        {
            DriveDriverOrientedAtAngle(0, 0.0625, 0, 0.25);
        }
        else if(robotPitchAngle < -7)
        {
            DriveDriverOrientedAtAngle(0, -0.0625, 0, 0.25);
        }
        else
        {
            DriveDriverOrientedAtAngle(0, 0, 0, 0.25);
        }*/
        DriveDriverOrientedAtAngle(0, Functions.DeadZone(0.005*balanceAngle, 0.0001), 0, 0.125);
    }
    
    public void DriveToNearest(int type)
    {
        
        switch(type){
            case 0:
                DriveTo(VisionSubsystem.conFieldX, VisionSubsystem.conFieldY, robotYawFieldRelative, 1, 1);
                break;
            case 1:
                //DriveTo();
                break;
            default:
            DriveTo(VisionSubsystem.conFieldX, VisionSubsystem.conFieldY, robotYawFieldRelative, 1, 1);
                break;
        }
        
    }
}
