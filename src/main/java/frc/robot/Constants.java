// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.AlternateEncoderType;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

//Imports below are from SwerveMK3 file
import edu.wpi.first.wpilibj.util.Units;


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
    public static final class HIDConstants
    {
        /**
         * XBox Controller layout
         * kA = 1;
         * kB = 2;
         * kX = 3; 
         * kY = 4;
         * kLB = 5;
         * kRB = 6;
         * kSelect = 7;
         * kStart = 8;
         * kLeftStickPress = 9;
         * kRightStickPress = 10;
         */

        public static final int k_DriverControllerPort = 1;
        public static final int k_OperatorControllerPort = 2;
        public static final double kDeadBand = 0.05;
        public static final int kA = 1;
        public static final int kB = 2;
        public static final int kX = 3;
        public static final int kY = 4;
        public static final int kLB = 5;
        public static final int kRB = 6;
        public static final int kStart = 8;
        public static final int kDL = 270;
        public static final int kDR = 90;

    }

    public static final class IntakeSubsystemConstants
    {
        public static final double k_intakeMotorSpeed = -0.6;
        public static final double k_intakeReverseMotorSpeed = 0.6;
        public static final double k_intakeStopMotorSpeed = 0.0;
    }
    
    public static final class SwerveDriveModuleConstants
    {
        public static enum k_SwerveDriveModules
        {
            leftFront,
            rightFront,
            leftRear,
            rightRear,
        };

        // Distance between centers of right and left wheels on robot; unit is meters
        public static final double k_TrackWidth = 0.5842;

        // Distance between front and back wheels on robot; unit is meters 
        public static final double k_WheelBase = 0.5842;

        public static final double k_RobotRadius = .4131; //distance from center of robot to the wheel in m
        
        public static final double k_MaxSpeed = Units.feetToMeters(10); // unit is m/s ; actual max is 12
        public static final double k_MaxTeleSpeed = Units.feetToMeters(15);
        public static final double k_MaxAcceleration = Units.feetToMeters(8); // unit is m/s/s (aka, m/s^2)
        
        public static final double kXYjoystickCoefficient = .5;
        public static final double kMaxAngularSpeed = Units.feetToMeters(8) / k_RobotRadius; //Gives in radians / s (/s is implied);
        public static final double kRotCoefficient = .25;

        public static double fieldCalibration = 0;

        //Angle offsets
        public static double frontLeftOffset = 20;
        public static double frontRightOffset = 78.574219;
        public static double backLeftOffset = 160;
        public static double backRightOffset = 2;

        public static final double k_SteeringEncoderCoefficient = (Math.PI * 2) / 4096.0; // this is 2π radians, dividided by the 12-bit encoder precision 

        public static final double ks_Volts = 1;
        public static final double kv_VoltSecondsPerMeter = 0.8;
        public static final double ka_VoltSecondsSquaredPerMeter = 0.15;

        public static final boolean k_GyroReversed = false;

        public static final AlternateEncoderType k_AlternateEncoderType = AlternateEncoderType.kQuadrature;
        public static final double kEncoderTicksPerRotation = 4096;

        public static final int k_RevNEOEncoderCtsPerRev = 42; // the NEO's hall-effect encoder is 42 counts/rev

        public static final double k_WheelDiameterMeters = 0.0985;
        public static final double k_WheelCircumference = k_WheelDiameterMeters * Math.PI;
        public static final double k_MK3SwerveModuleGearRatio = 6.86;
        public static final double k_CANEncoderVelocityCoefficient = k_WheelCircumference / (60 * k_MK3SwerveModuleGearRatio); //This converts RPMs to m/s
        public static final double k_CANEncoderPositionCoefficient = k_WheelCircumference / k_MK3SwerveModuleGearRatio;

        public static final double k_DriveEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (k_WheelDiameterMeters * Math.PI) / ( (double) k_RevNEOEncoderCtsPerRev * k_MK3SwerveModuleGearRatio);
    
        public static final double k_TurningEncoderDistancePerPulse =
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            (2 * Math.PI) / (double) kEncoderTicksPerRotation; //in radians

        /**
         * The CAN Bus device IDs for devices used with the Swerve Drive: motor controllers and encoders
         */
        public static final class CANBusIDs
        {
            public static final int k_LeftFront_DriveMotor = 1; 
            public static final int frontLeftCANCoderId = 9; 
            public static final int k_LeftFront_SteeringMotor = 2;

            public static final int k_RightFront_DriveMotor = 3; 
            public static final int frontRightCANCoderId = 10; 
            public static final int k_RightFront_SteeringMotor = 4; 

            public static final int k_LeftRear_DriveMotor = 5; 
            public static final int backLeftCANCoderId = 11; 
            public static final int k_LeftRear_SteeringMotor = 6;

            public static final int k_RightRear_DriveMotor = 7; 
            public static final int backRightCANCoderId = 12; 
            public static final int k_RightRear_SteeringMotor = 8; 
            
            public static final int k_FlywheelMotorOne = 13;
            public static final int k_FlywheelMotorTwo = 14;
            public static final int k_HoodID = 15;
            public static final int k_IntakeMotors = 16;
            public static final int k_IndexerID = 17; 

            public static final int k_TalonTestID = 20;
        }
        
        /**
         * The Motor controller we're using can control both brushed and brushless DC motors. 
         * We need to specify what kind of motor we're controlling.
         */
        public static final class MotorTypes
        {
            public static final MotorType k_SwerveRightFront_Drive = MotorType.kBrushless;
            public static final MotorType k_SwerveRightFront_Steering = MotorType.kBrushless;
            
            public static final MotorType k_SwerveLeftFront_Drive = MotorType.kBrushless;
            public static final MotorType k_SwerveLeftFront_Steering = MotorType.kBrushless;
    
            public static final MotorType k_SwerveRightRear_Drive = MotorType.kBrushless;
            public static final MotorType k_SwerveRightRear_Steering = MotorType.kBrushless;
    
            public static final MotorType k_SwerveLeftRear_Drive = MotorType.kBrushless;
            public static final MotorType k_SwerveLeftRear_Steering = MotorType.kBrushless;
        }        
     
        public static final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                new Translation2d(k_WheelBase / 2, k_TrackWidth / 2),    // Left Front
                new Translation2d(k_WheelBase / 2, -k_TrackWidth / 2),   // Right Front
                new Translation2d(-k_WheelBase / 2, k_TrackWidth / 2),   // Left Rear
                new Translation2d(-k_WheelBase / 2, -k_TrackWidth / 2)); // Right Rear


        public static final class DriveModulePIDValues
        {
            public static double k_driveP = 0.028; //0.0005
            public static double k_driveI = 0.0;
            public static double k_driveD = 0.025;

            public static final double k_driveIz = 0; 
            public static double k_driveFF = 1/Units.feetToMeters(14.4);
            
            public static final double k_driveMaxOutput = 1; 
            public static final double k_driveMinOutput = -1;
            public static final double k_driveMaxRPM = 5600; 

            public static final double k_driveMaxVel = 5600; // rpm
            public static final double k_driveMinVel = 0;
            public static final double k_driveMaxAcc = 1500;

            public static final double k_driveAllowedError = 0.05; // allowedErr - The allowed deviation for your setpoint vs actual position in rotations
            public static final int k_driveSmartMotionSlot = 0; // Is the gain schedule slot, the value is a number between 0 and 3. Each slot has its own set of gain values and can be changed in each control frame using SetReference().
        }

        public static final class SteeringControllerPIDValues
        {
           public static final double k_steerP[] = 
           {
               0.00095, //LeftFront
               0.001, //RightFront
               0.00095, //LeftRear
               0.001, //RightRear
           };
            public static final double k_steerI= 0;
            public static final double k_steerD[] = 
            {
                0.000007,
                0.00001,
                0.000007,
                0.00001,
            };

            public static final double k_steerIz = 0; 
            public static final double k_steerFF = 0; 

            public static final double k_steerMaxOutput = 1; 
            public static final double k_steerMinOutput = -1;
            public static final double k_steerMaxRPM = 5600;

            public static final double k_steerMaxVel = 2000; // rpm
            public static final double k_steerMinVel = 0;
            public static final double k_steerMaxAcc = 1500;

            public static final double k_steerDeadband = 0.02; // Deadband on the motor controller
            public static final double k_ToleranceInTicks = 5;
        }

        public static final class AutoChooser
        {
            // public static final double k_RedAMax = 50;
            // public static final double k_RedAMin = -50;

            // public static final double k_RedBMax = 200;
            // public static final double k_RedBMin = 100;

            // public static final double k_BlueAMax = -999;
            // public static final double k_BlueAMin = -900;

            // public static final double k_BlueBMax = -200;
            // public static final double k_BlueBMin = -100;

            public static final double k_RedA = 1.95;
            public static final double k_RedB = 10.25;
            public static final double k_BlueA = -4.4;
            public static final double k_BlueB = -9.55;
            public static final double k_autoTolerance = 2.5;


        }

        public static final class FlywheelPIDValues
        {
            public static final double k_flywheelP = 0.0005;
            public static final double k_flywheelI = 0.0;
            public static final double k_flywheelD = 0.00007;
            public static final double k_flywheelFF =  0.000179;
            public static final double k_minFlywheelOutput = 0.0;
            public static final double k_maxFlywheelOutput = 1.0;
            public static final double k_rampTime = 0.35;

            public static final double k_aimingP = 1.1;
            public static final double k_aimingI = 0.0;
            public static final double k_aimingD = 1;
            public static final double k_aimTolerance = .5 * (Math.PI/180);
        }
        
        public static final class HexEncoderInputs
        {
            public static final int k_absoluteInput = 0;
            public static final int k_quadratureA = 1;
            public static final int k_quadratureB = 2;
            public static final int k_indexInput = 3;
        }

        public static final class ShooterConstants
        {
            //public static final double shooterRotationRight = 0.02;
            //public static final double shooterRotationLeft = -0.02;
            //public static final double flywheelMaxSpeed = .7;
            public static final double indexerMaxSpeed = 1;
            public static final double k_rampRate = 0.25;
        }

        public static final class HoodConstants
        {
            public static final double k_hoodP = .0025;
            public static final double k_hoodI = 0;
            public static final double k_hoodD = 0.005;
            public static final double k_hoodTolerance = 5;
            public static final double k_hoodAutoSpeed = .5;

            public static final double k_hoodExtendSpeed = .2;
            public static final double k_hoodRetractSpeed = -.2;

            public static final double limelightAngle = 32.4;// angle that the limelight is set at in degrees
        
            public static final double limelightHeight = 20.5; //from ground to limelight in inches
            public static final double goalHeight = 91.5; // LL crosshair height, found empirically
            public static final double heightDifference = goalHeight - limelightHeight;

            public static final double k_maxDistance = 270; //in
            public static final double k_minDistance = 12; //in

            //Colors represent zones
            public static final double k_redZoneDistance = 210;
            public static final double k_blueZoneDistance = 150;
            public static final double k_yellowZoneDistance = 90;
            public static final double k_greenZoneDistance = 0;
            public static final double k_closestZoneDistance = 0;

            public static final double k_redEncoder = 1761; //ABS encoder = 1.56;
            public static final double k_blueEncoder = 1536; //ABS encoder = 1.45;
            public static final double k_yellowEncoder = 1106; //ABS encoder = 1.24;
            public static final double k_greenEncoder = 512; //ABS encoder = 0.95;
            public static final double k_closestEncoder = 307; //ABS encoder = 0.85;

            public static final double k_retractAbsSetpoint = .1; //Rotations
            public static final double k_retractQuadSetpoint = 0; //Ticks
            public static final double k_extendQuadSetpoint = 3750; //Ticks
            public static final double k_quadTicksPerRotation = 2048; 
        }

        public static final class rotationPID{
            public static final double k_rotationP = .01;
            public static final double k_rotationI = 0;
            public static final double k_rotationD = 0;
            public static final double k_rotationTolerance = 2 * (Math.PI/180);
        }

        // Path Following
    public static final double DRIVE_POS_ERROR_CONTROLLER_P = 17; // 10
    public static final double DRIVE_POS_ERROR_CONTROLLER_I = 0;
    public static final double DRIVE_POS_ERROR_CONTROLLER_D = .05;
    public static final double DRIVE_HEADING_ERROR_CONTROLLER_P = 0; // 1.05
    public static final double DRIVE_HEADING_ERROR_CONTROLLER_I = 0;
    public static final double DRIVE_HEADING_ERROR_CONTROLLER_D = 0; // 0.02
    public static final double DRIVE_ROTATION_CONTROLLER_P = 10;// 9
    public static final double DRIVE_ROTATION_CONTROLLER_I = 0;
    public static final double DRIVE_ROTATION_CONTROLLER_D = 0;
    public static final double DRIVE_TARGETING_CONTROLLER_P = 13;// 9
    public static final double DRIVE_TARGETING_CONTROLLER_I = 0;
    public static final double DRIVE_TARGETING_CONTROLLER_D = 0.5;
    public static final double DRIVE_ROTATION_MIN_VELOCITY = 25;
    public static final double DRIVE_TARGETING_I_ZONE = 2;

    }

    public final class talonPID
    {
        public final static double k_talonP = 0.01;
        public final static double k_talonI = 0.0;
        public final  static double k_talonD = 0.0;
        public final  static double k_talonFF = 0.0;

    }
}
