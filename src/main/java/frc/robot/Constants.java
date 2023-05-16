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
    public static double turnMultiplier = 0.25;
    public static double swerveModulePMult = 0.01;
    public static double swerveModuleDMult = 0.0001;
    public static double swerveModuleMaxThrottleChange = 0.1; //percent per frame
    public static double swerveCrouchModeMult = 0.25;
    public static double swerveMaxAccel = 0.043; //percent per frame
    public static double swerveDriveRatio = (1.0/8.14); //Swerve module gear ratio
    public static double swerveWheelCircumference = 0.1 * Math.PI; //in m
    public static double swerveDriveToPMult = 1.0;
    public static double swerveDriveToDMult = 7.0;
    public static double swerveDriveToMaxSpeed = 1.0; //The max speed (percent) that swerve is allowed to move when moving on it's own
    public static double swerveMaxSpeed = 3.6576; // meters per second

    public static double swerveAutoTurnPMult = 0.005;
    public static double swerveAutoTurnMaxSpeed = 0.5; //The max speed (percent) that swerve is allowed to spin when turning on it's own
    public static double swerveAutoTurnDMult = 0;

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
    
    
    public static Compressor demoCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    public static Solenoid demoSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    public static DoubleSolenoid demoDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 6);
    public static PneumaticsControlModule demoPCM = new PneumaticsControlModule();

    public static PowerDistribution PDP = new PowerDistribution(0, ModuleType.kCTRE);
    public static double currentWarningLevel = 80;
    
    public static double armOriginVerticalOffset = 90; // vertical offset of the arm mount from the floor, in cm. 
    public static double armOriginHorizontalOffset = -19; // horizontal offset of the arm mount from front of bumper, in cm
    public static double clawMinHeight = 0; // clamp on the minimum height of the claw, prevents hitting the floor (doesn't do that right now).
    public static double clawMaxHeight = 173; // bit low on the safe side, as the height exceeds frc limit. Should never reach this limit (hopefully).
    public static double armSegmentALength = 52; // length of the first arm segment, in cm
    public static double armSegmentBLength = 52; // length of second arm segment from 
    public static double armGearRatio1 = (1.0/100.0); 
    public static double armGearRatio2 = (1.0/80.0);

    public static double armSegment1PMult = 1.0/15.0; // arm PID multiplier
    public static double armSegment1DMult = 0.00005;
    public static double armSegment2PMult = 1.0/15.0; // arm PID multiplier
    public static double armSegment2DMult = 0.00005;

    public static double armPreciseModeMult = 0.33; // precision mode for the arm, slows down arm by multiplier when mode is activated.
    public static double armSpeedMult = 20; // cm/sec
    public static double bendRestingAngle = -90.0; //-90 for hanging   -139.6 for stowed
    public static double raiseRestingAngle = 0.0; //0 for hanging      224.32 for stowed
    public static boolean raiseMotorInverted = true; //inverts the motor movement on arm motor 1
    public static boolean bendMotorInverted = true; //inverts the motor movement on arm 
    public static double maxArmSpeed = 1.0; //maximum power output for the arm. 1.0 is 100%. 
    public static double armExtendBuffer = 0.9975; //set between 0 and 1, 1 for max extension range, but be careful !DO NOT SET ABOVE ONE!
    
    public static double armSegment1GravMult = 0;
    public static double armSegment2GravMult = -0.1;

    public static CANSparkMax armMotor1 = new CANSparkMax(13, MotorType.kBrushless); //declare arm motors
    public static CANSparkMax armMotor2 = new CANSparkMax(14, MotorType.kBrushless); 

    public static int defaultAutoSequence = 2;
}
