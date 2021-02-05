// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Mat;

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






    public static final class SwerveDriveModuleConstants
    {
        public static final double k_MaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double k_MaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

        public static final double k_SteeringEncoderCoefficient = (Math.PI * 2) / 4096.0; // this is 2π radians, dividided by the 12-bit encoder precision 


        public static final int k_RevNEOEncoderCtsPerRev = 42; // the NEO's hall-effect encoder is 42 counts/rev
        public static final double k_WheelDiameterMeters = 0.15;
        public static final double k_MK3SwerveModuleGearRatio = 6.86;
        public static final double k_DriveEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (k_WheelDiameterMeters * Math.PI) / ( (double) k_RevNEOEncoderCtsPerRev * 6.86);
    
        public static final double kTurningEncoderDistancePerPulse =
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            (2 * Math.PI) / (double) k_RevNEOEncoderCtsPerRev;

        public static final class CANBusIDs
        {
            public static final int k_SwerveRightFront_Drive = 1;
            public static final int k_SwerveRightFront_Turning = 2;
    
            public static final int k_SwerveLeftFront_Drive = 3;
            public static final int k_SwerveLeftFront_Turning = 4;
    
            public static final int k_SwerveRightRear_Drive = 5;
            public static final int k_SwerveRightRear_Turning = 6;
    
            public static final int k_SwerveLeftRear_Drive = 7;
            public static final int k_SwerveLeft_Turning = 8;
        }
        
        public static final class MotorTypes
        {
            public static final MotorType k_SwerveRightFront_Drive = MotorType.kBrushless;
            public static final MotorType k_SwerveRightFront_Turning = MotorType.kBrushless;
            
            public static final MotorType k_SwerveLeftFront_Drive = MotorType.kBrushless;
            public static final MotorType k_SwerveLeftFront_Turning = MotorType.kBrushless;
    
            public static final MotorType k_SwerveRightRear_Drive = MotorType.kBrushless;
            public static final MotorType k_SwerveRightRear_Turning = MotorType.kBrushless;
    
            public static final MotorType k_SwerveLeftRear_Drive = MotorType.kBrushless;
            public static final MotorType k_SwerveLeftRear_Turning = MotorType.kBrushless;
        }

        public static final class TurningController
        {
            public static final double k_Prop = 1;
            public static final double k_Inter = 0;
            public static final double k_Diff = 0;
        }

        public static final class DriveModule
        {
            public static final double k_Proportional = 1;
            public static final double k_Intergral = 0;
            public static final double k_Differential = 0;
        }

    }
}
