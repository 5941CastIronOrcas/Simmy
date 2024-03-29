// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
//import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
//import com.revrobotics.CANEncoder;
//import com.revrobotics.CANSensor;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */



public final class Constants 
{
    public static double turnMultiplier = 0.25; //applied to the right stick to limit turn speed to a reasonable amount
    public static double swerveModulePMult = 0.01;
    public static double swerveModuleDMult = 0.000;
    public static double swerveModuleMaxThrottleChange = 0.1; //percent per frame
    public static double swerveCrouchModeMult = 0.25;
    public static double swerveMaxAccel = 0.15; //percent per frame   0.06
    public static double swerveDriveRatio = (1.0/8.14); //Swerve module gear ratio
    public static double swerveWheelCircumference = 0.096774 * Math.PI; //in m

    public static double swerveDriveToPMult = 1.0; //1.0
    public static double swerveDriveToDMult = 7.0;//7
    public static double swerveDriveToDeadZone = 0.01; //meters
    public static double swerveMaxSpeed = 3.6576; // meters per second
    public static double swerveAutoTurnPMult = 0.005;
    public static double swerveAutoTurnMaxSpeed = 0.5; //The absolute max speed (percent) that swerve is ever allowed to spin when turning on it's own
    public static double swerveAutoTurnDMult = 0;
    public static double swerveAutoTurnDeadZone = 0.5; //degrees

    public static Pose2d[] redGrid;
    public static Pose2d[] blueGrid;
    
    
    public static XboxController controller = new XboxController(0);
    public static XboxController controllerB = new XboxController(1);
    public static double controllerDeadZone = 0.1;
    
    public static CANSparkMax frontRightAngleMotor = new CANSparkMax(2, MotorType.kBrushless);
    public static CANSparkMax frontLeftAngleMotor = new CANSparkMax(3, MotorType.kBrushless);
    public static CANSparkMax backRightAngleMotor = new CANSparkMax(4, MotorType.kBrushless);
    public static CANSparkMax backLeftAngleMotor = new CANSparkMax(5, MotorType.kBrushless);

    public static CANSparkMax frontRightDriveMotor = new CANSparkMax(30, MotorType.kBrushless);
    public static CANSparkMax frontLeftDriveMotor = new CANSparkMax(7, MotorType.kBrushless);
    public static CANSparkMax backRightDriveMotor = new CANSparkMax(8, MotorType.kBrushless);
    public static CANSparkMax backLeftDriveMotor = new CANSparkMax(9, MotorType.kBrushless);

    public static CANSparkMax gripperMotorA = new CANSparkMax(10, MotorType.kBrushless); //Left
    public static CANSparkMax gripperMotorB = new CANSparkMax(6, MotorType.kBrushless); //Right

    public static double coneGripperPowerIn = 0.5;
    public static double cubeGripperPowerIn = 0.25;
    public static double cubeGripperPowerOut = 0.15;
    public static double coneGripperPowerOut = 0.3;
    public static double clawAmpLimit = 30;

    public static WPI_CANCoder  frontRightEncoder = new WPI_CANCoder(15);
    public static WPI_CANCoder  frontLeftEncoder = new WPI_CANCoder(16);
    public static WPI_CANCoder  backRightEncoder = new WPI_CANCoder(17);
    public static WPI_CANCoder  backLeftEncoder = new WPI_CANCoder(18);

    public static Pigeon2 primaryAccelerometer = new Pigeon2(19);
    public static double primaryAccelerometerPitchOffset = 0;
    
    
    public static Compressor demoCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    public static Solenoid demoSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    public static DoubleSolenoid demoDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 6);
    public static PneumaticsControlModule demoPCM = new PneumaticsControlModule();

    public static PowerDistribution PDP = new PowerDistribution(0, ModuleType.kCTRE);
    public static double currentWarningLevel = 80;
    
    public static double armGearRatio1 = (1.0/100.0); 
    public static double armGearRatio2 = (1.0/80.0);

    public static double armSegment1PMult = 1.0/30.0; // arm PID multiplier
    public static double armSegment1DMult = -0.0001;
    public static double armSegment2PMult = 1.0/30.0; // arm PID multiplier
    public static double armSegment2DMult = -0.0001;

    public static double armPreciseModeMult = 0.33; // precision mode for the arm, slows down arm by multiplier when mode is activated.
    public static boolean raiseMotorInverted = false; //inverts the motor movement on arm motor 1
    public static boolean bendMotorInverted = false; //inverts the motor movement on arm 
    public static double maxArmSpeed = 0.3; //maximum power output for the arm. 1.0 is 100%. 

    public static double armSegment1GravMult = 0;
    public static double armSegment2GravMult = 0;

    public static double armRestingAngle1 = 184.6, armRestingAngle2 = 276;
    public static double armCalibrateAngle1 = 188, armCalibrateAngle2 = 276;
    public static double armCollectAngle1 = 20, armCollectAngle2 = 0;
    public static double armDepositAngle1 = 11.5, armDepositAngle2 = 13;
    public static double armScoopAngle1 = -73, armScoopAngle2 = -36;
    public static double armBackDepositAngle1 = 135, armBackDepositAngle2 = 175;

    public static CANSparkMax armMotor1 = new CANSparkMax(13, MotorType.kBrushless); //declare arm motors
    public static CANSparkMax armMotor2 = new CANSparkMax(14, MotorType.kBrushless); 

    public static int defaultAutoSequence = 0;
}
