package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Functions;

//import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenix.sensors.CANCoderConfiguration;
//import com.ctre.phoenix.sensors.CANCoderStatusFrame;


public class SwerveModule {
    CANSparkMax angleMotor;
    CANSparkMax driveMotor;
    CANCoder encoder;
    double oldDeltaAngle = 0;
    double currentAngle;
    double currentAngleSpeed;
    

    public SwerveModule(CANSparkMax inputAngleMotor, CANSparkMax inputDriveMotor, CANCoder inputEncoder)
    {
        angleMotor = inputAngleMotor;
        driveMotor =  inputDriveMotor;
        encoder = inputEncoder;

    }

    public void Drive(double angle, double speed)
    {
        currentAngle = encoder.getPosition();
        driveMotor.set(speed);
        currentAngleSpeed = encoder.getVelocity();
        //currentSpeed = ((Functions.DeltaAngle(angle, currentAngle) - oldDeltaAngle) / 0.02);
        double pComponent = Constants.swerveModulePMult * Functions.DeltaAngleRadians(angle, currentAngle);
        double dComponent = Constants.swerveModuleDMult * currentAngleSpeed;
        angleMotor.set(pComponent + dComponent);
        oldDeltaAngle = Functions.DeltaAngleRadians(angle, currentAngle);
    }
}
