package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class Functions {
    
    public static double Clamp(double input, double min, double max)
    {
        if(input > max)
        {
            input = max;
        }
        else if(input < min)
        {
            input = min;
        }
        return input;
    }
    public static double UnZero(double input)
    {
        if(input == 0)
        {
            input = 0.000001;
        }
        return input;
    }
    public static double DeadZone(double input, double deadZone)
    {
        if(Math.abs(input) < deadZone)
        {
            return 0;
        }
        else
        {
            return input;
        }
        
    }
    public static double Exponential(double input)
    {
        return input * Math.abs(input);
    }
    public static double DeltaAngleDegrees(double startAngle, double endAngle)
    {
        return ((((endAngle - startAngle - 180) % 360) + 360) % 360)-180;
    }
    public static double DeltaAngleRadians(double startAngle, double endAngle)
    {
        return ((((endAngle - startAngle - Math.PI) % (2*Math.PI)) + (2*Math.PI)) % (2*Math.PI))-Math.PI;
    }

    public static void KillAllSwerve()
    {
        Constants.frontRightAngleMotor.stopMotor();
        Constants.frontLeftAngleMotor.stopMotor();
        Constants.backRightAngleMotor.stopMotor();
        Constants.backLeftAngleMotor.stopMotor();
        
        Constants.frontRightDriveMotor.stopMotor();
        Constants.frontLeftDriveMotor.stopMotor();
        Constants.backRightDriveMotor.stopMotor();
        Constants.backLeftDriveMotor.stopMotor();
    }

    public static void KillAllArm()
    {
        Constants.armMotor1.stopMotor();
        Constants.armMotor2.stopMotor();
        Constants.gripperMotorA.stopMotor();
        Constants.gripperMotorB.stopMotor();
    }

    public static double DriverToFieldAngle(double angle)
    {
        if(DriverStation.getAlliance() == DriverStation.Alliance.Red)
        {angle = Functions.DeltaAngleDegrees(0, angle - 90);}
        else if(DriverStation.getAlliance() == DriverStation.Alliance.Blue)
        {angle = Functions.DeltaAngleDegrees(0, angle + 90);}
        return angle;
    }

    public static double FieldToDriverAngle(double angle)
    {
        if(DriverStation.getAlliance() == DriverStation.Alliance.Red)
        {angle = Functions.DeltaAngleDegrees(0, angle + 90);}
        else if(DriverStation.getAlliance() == DriverStation.Alliance.Blue)
        {angle = Functions.DeltaAngleDegrees(0, angle - 90);}
        return angle;
    }

    public static double Pythagorean(double x, double y)
    {
        return Math.sqrt((x * x) + (y * y));
    }

}
