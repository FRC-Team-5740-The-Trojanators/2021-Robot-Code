// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Written by Team 5470

package frc.robot.paths;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

public class BouncePath3
{
    private static String trajectoryName = "Bounce Path 3";

    private static TrajectoryConfig trajectoryConfiguration = new TrajectoryConfig(Constants.SwerveDriveModuleConstants.k_MaxSpeed, 3)
                                                            .setKinematics(Constants.SwerveDriveModuleConstants.kinematics);


                                            // x    y      heading
    private static Pose2d start = new Pose2d(4.7, 3.5, new Rotation2d(0));

    private static  Translation2d point8 = new  Translation2d(4.95, .4);
    private static  Translation2d point9 = new  Translation2d(6.59, .51);
    private static  Translation2d point10 = new  Translation2d(6.79, 1.5);
    private static  Translation2d point11 = new  Translation2d(6.81, 2.7);
    private static  Translation2d point12 = new  Translation2d(6.85, 3.3);

    private static Pose2d end = new Pose2d(6.85, 3.3, new Rotation2d(0));

    /**
     * Generates the Trajectory
     * @return the generated trajectory
     */
    public static Trajectory getTrajectory()
    {
        return TrajectoryGenerator.generateTrajectory(
           start, List.of(point8, point9, point10, point11, point12), end,
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