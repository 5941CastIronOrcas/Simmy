package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;

public class SwerveDriveSubsystem extends SubsystemBase {
    
    double FRX;
    double FRY;
    double FLX;
    double FLY;
    double BRX;
    double BRY;
    double BLX;
    double BLY;

    double FRA;
    double FLA;
    double BRA;
    double BLA;
    
    double FRT;
    double FLT;
    double BRT;
    double BLT;

    double LSX;
    double LSY;
    double RSX;

    double highestSpeed = 0;
    
    SwerveModule frontRightModule = new SwerveModule(Constants.frontRightAngleMotor, Constants.frontRightDriveMotor, Constants.frontRightEncoder);
    SwerveModule frontLeftModule = new SwerveModule(Constants.frontLeftAngleMotor, Constants.frontLeftDriveMotor, Constants.frontLeftEncoder);
    SwerveModule backRightModule = new SwerveModule(Constants.backRightAngleMotor, Constants.backRightDriveMotor, Constants.backRightEncoder);
    SwerveModule backLeftModule = new SwerveModule(Constants.backLeftAngleMotor, Constants.backLeftDriveMotor, Constants.backLeftEncoder);

    public SwerveDriveSubsystem() //Constructor (Init)
    {

    }

    @Override
    public void periodic() //Every 0.02 sec (50 FPS)
    {
        if(Constants.controller.getAButton())
        {
            LSX = Functions.DeadZone(Constants.controller.getLeftX(), 0.1);
            LSY = Functions.DeadZone(-Constants.controller.getLeftY(), 0.1);
            RSX = Functions.DeadZone(Constants.controller.getRightX(), 0.1);
            

            FRX = Constants.turnMultiplier*RSX + LSX;
            FRY = -Constants.turnMultiplier*RSX + LSY;
            FLX = Constants.turnMultiplier*RSX + LSX;
            FLY = Constants.turnMultiplier*RSX + LSY;
            BRX = -Constants.turnMultiplier*RSX + LSX;
            BRY = -Constants.turnMultiplier*RSX + LSY;
            BLX = -Constants.turnMultiplier*RSX + LSX;
            BLY = Constants.turnMultiplier*RSX + LSY;

            FRA = Math.atan2(FRX, FRY);
            FLA = Math.atan2(FLX, FLY);
            BRA = Math.atan2(BRX, BRY);
            BLA = Math.atan2(BLX, BLY);

            FRT = Math.abs(FRX)+Math.abs(FRY);
            FLT = Math.abs(FLX)+Math.abs(FLY);
            BRT = Math.abs(BRX)+Math.abs(BRY);
            BLT = Math.abs(BLX)+Math.abs(BLY);

            highestSpeed = Math.max(Math.max(FRT, FLT), Math.max(BRT, Math.max(BLT, 1)));

            FRT = (Math.abs(FRX)+Math.abs(FRY)) / highestSpeed;
            FLT = (Math.abs(FLX)+Math.abs(FLY)) / highestSpeed;
            BRT = (Math.abs(BRX)+Math.abs(BRY)) / highestSpeed;
            BLT = (Math.abs(BLX)+Math.abs(BLY)) / highestSpeed;

            frontRightModule.Drive(FRA, FRT);
            frontLeftModule.Drive(FLA, FLT);
            backRightModule.Drive(BRA, BRT);
            backLeftModule.Drive(BLA, BLT);
        }

        SmartDashboard.putNumber("Target Angle", Math.toDegrees(FRA));
        SmartDashboard.putNumber("Target Throttle", FRT);
        SmartDashboard.putNumber("Current Angle", Functions.DeltaAngleDegrees(0, frontRightModule.currentAngle));
        SmartDashboard.putNumber("Current Angle Rate", Math.toDegrees(frontRightModule.currentAngleSpeed));
        SmartDashboard.putNumber("Current Angle Raw", frontRightModule.currentAngle);
        
    }

    
}
