// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Written by Team 5470

package frc.robot.pathsOLD;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

public class SlalomPath
{
    private static String trajectoryName = "Slalom Path";

    private static TrajectoryConfig trajectoryConfiguration = new TrajectoryConfig(Constants.SwerveDriveModuleConstants.k_MaxSpeed, 3)
                                                            .setKinematics(Constants.SwerveDriveModuleConstants.kinematics);


                                            // x    y      heading
    private static Pose2d start = new Pose2d(1.016, 0.762, new Rotation2d(0.0));

    private static Translation2d point1 = new Translation2d(3.048, 2.286 );
    private static Translation2d point2 = new Translation2d(6.096, 2.287 );
    private static Translation2d point3 = new Translation2d(7.366, 0.762 );
    private static Translation2d point4 = new Translation2d(8.636, 1.778 );
    private static Translation2d point5 = new Translation2d(7.366, 2.54 );
    private static Translation2d point6 = new Translation2d(6.35, .762 );
    private static Translation2d point7 = new Translation2d(3.048, .763 );
    private static Translation2d point8 = new Translation2d(1.778, 2.032 );

    private static Pose2d end =    new Pose2d(1.016, 2.54, new Rotation2d(0.0));

    /**
     * Generates the Trajectory
     * @return the generated trajectory
     */
    public static Trajectory getTrajectory()
    {
        return TrajectoryGenerator.generateTrajectory(
           start, List.of( point1, point2, point3, point4, point5, point6, point7, point8), end,
            trajectoryConfiguration);
    }

    /**
     * Provides the name of the Trajectory
     * @return The name of the Trajectory
     */
    public static String getTrajectoryName()
    {
        return trajectoryName;
    }
}