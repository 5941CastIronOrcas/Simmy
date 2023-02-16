package frc.robot.subsystems;

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

    double LSX, LSY, RSX;

    double highestSpeed = 0;
    boolean crouchMode = true;
    
    double robotYawAngle = 0;

    SwerveModule frontRightModule = new SwerveModule(Constants.frontRightAngleMotor, true, Constants.frontRightDriveMotor, false, Constants.frontRightEncoder);
    SwerveModule frontLeftModule = new SwerveModule(Constants.frontLeftAngleMotor, true, Constants.frontLeftDriveMotor, false, Constants.frontLeftEncoder);
    SwerveModule backRightModule = new SwerveModule(Constants.backRightAngleMotor, true, Constants.backRightDriveMotor, false, Constants.backRightEncoder);
    SwerveModule backLeftModule = new SwerveModule(Constants.backLeftAngleMotor, true, Constants.backLeftDriveMotor, false, Constants.backLeftEncoder);

    public SwerveDriveSubsystem() //Constructor (Init)
    {

    }

    @Override
    public void periodic() //Every 0.02 sec (50 FPS)
    {
        LSX = (crouchMode?Constants.swerveCrouchModeMult:1)*Functions.Exponential(Functions.DeadZone(Constants.controller.getLeftX(), Constants.controllerDeadZone));
        LSY = (crouchMode?Constants.swerveCrouchModeMult:1)*Functions.Exponential(Functions.DeadZone(-Constants.controller.getLeftY(), Constants.controllerDeadZone));
        RSX = (crouchMode?Constants.swerveCrouchModeMult:1)*Functions.Exponential(Functions.DeadZone(Constants.controller.getRightX(), Constants.controllerDeadZone));
        robotYawAngle = Functions.DeltaAngleDegrees(0, -Constants.primaryAccelerometer.getYaw());
        if(Constants.controller.getLeftBumper())
        {
            DriveFieldOriented(LSX, LSY, RSX);
        }
        else
        {
            //Kill all motors
            Functions.KillAll();
        }
        if(Constants.controller.getRightBumper())
        {
            Constants.primaryAccelerometer.setYaw(0);
        }

        SmartDashboard.putNumber("Robot Yaw", robotYawAngle);
    }
    public void DriveFieldOriented(double LSX, double LSY, double RSX)
    {
        Drive(LSX*Math.cos(Math.toRadians(-robotYawAngle))+LSY*Math.sin(Math.toRadians(-robotYawAngle)), LSY*Math.cos(Math.toRadians(-robotYawAngle))+LSX*Math.sin(Math.toRadians(robotYawAngle)), RSX);
    }
    
    public void Drive(double LSX, double LSY, double RSX)
    {
        if(Constants.controller.getLeftStickButtonPressed())
            {
                crouchMode = !crouchMode;
            }
            
            //set the target X and Y speeds based on controller input
            FRX = Constants.turnMultiplier*RSX + LSX;
            FRY = -Constants.turnMultiplier*RSX + LSY;
            FLX = Constants.turnMultiplier*RSX + LSX;
            FLY = Constants.turnMultiplier*RSX + LSY;
            BRX = -Constants.turnMultiplier*RSX + LSX;
            BRY = -Constants.turnMultiplier*RSX + LSY;
            BLX = -Constants.turnMultiplier*RSX + LSX;
            BLY = Constants.turnMultiplier*RSX + LSY;

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
            FRTOutput += Functions.Clamp(FRT-FRTOutput, -Constants.swerveModuleMaxThrottleChange, Constants.swerveModuleMaxThrottleChange);
            FLTOutput += Functions.Clamp(FLT-FLTOutput, -Constants.swerveModuleMaxThrottleChange, Constants.swerveModuleMaxThrottleChange);
            BRTOutput += Functions.Clamp(BRT-BRTOutput, -Constants.swerveModuleMaxThrottleChange, Constants.swerveModuleMaxThrottleChange);
            BLTOutput += Functions.Clamp(BLT-BLTOutput, -Constants.swerveModuleMaxThrottleChange, Constants.swerveModuleMaxThrottleChange);

            //Tell the modules to angle and drive themselves
            frontRightModule.Drive(FRA, FRTOutput);
            frontLeftModule.Drive(FLA, FLTOutput);
            backRightModule.Drive(BRA, BRTOutput);
            backLeftModule.Drive(BLA, BLTOutput);
    }
    
}
