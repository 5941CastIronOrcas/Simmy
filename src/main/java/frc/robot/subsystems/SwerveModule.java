package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Functions;

//import com.ctre.phoenix.sensors.AbsoluteSensorRange;
//import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenix.sensors.CANCoderConfiguration;
//import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.WPI_CANCoder;


public class SwerveModule {
    CANSparkMax angleMotor;
    CANSparkMax driveMotor;
    WPI_CANCoder encoder;
    double oldDeltaAngle = 0;
    double currentAngle;
    double currentAngleSpeed;
    boolean invertAngleMotor;
    boolean invertDriveMotor;
    boolean isBackwards = false;
    

    public SwerveModule(CANSparkMax inputAngleMotor, Boolean InvertAngleMotor, CANSparkMax inputDriveMotor, Boolean InvertDriveMotor, WPI_CANCoder inputEncoder)
    {
        angleMotor = inputAngleMotor;
        invertAngleMotor = InvertAngleMotor;
        invertDriveMotor = InvertDriveMotor;
        driveMotor =  inputDriveMotor;
        encoder = inputEncoder;

    }

    public void Drive(double angle, double speed)
    {
        if(Double.isNaN(angle)) angle = 0;
        currentAngle = encoder.getAbsolutePosition();
        if(Math.abs(Functions.DeltaAngleDegrees(Math.toDegrees(angle), currentAngle)) > 90)
        {
            angle = angle + (Math.PI);
            speed = -speed;
            isBackwards = true;
        }
        else
        {
            isBackwards = false;
        }
        driveMotor.set((invertDriveMotor?-1:1)*speed);
        currentAngleSpeed = encoder.getVelocity();
        double pComponent = Constants.swerveModulePMult * Functions.DeltaAngleDegrees(Math.toDegrees(angle), currentAngle);
        double dComponent = Constants.swerveModuleDMult * currentAngleSpeed;
        angleMotor.set((invertAngleMotor?-1:1)*(pComponent + dComponent));
    }
}
