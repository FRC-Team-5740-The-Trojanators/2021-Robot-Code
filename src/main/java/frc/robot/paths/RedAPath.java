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

public class RedAPath
{
    private static String trajectoryName = "Red A Path";

    private static TrajectoryConfig trajectoryConfiguration = new TrajectoryConfig(Constants.SwerveDriveModuleConstants.k_MaxSpeed, 3)
                                                            .setKinematics(Constants.SwerveDriveModuleConstants.kinematics);


                                            // x    y      heading
    private static Pose2d start = new Pose2d(0.381, 2.54, new Rotation2d(0));

    private static Translation2d point1 = new Translation2d(2.286, 2.286);
    private static Translation2d point2 = new Translation2d(3.81, 1.524);
    private static Translation2d point3 = new Translation2d(4.2, 2.8);
    private static Translation2d point4 = new Translation2d(4.35, 3.35);
    private static Translation2d point5 = new Translation2d(4.572, 3.81);
    private static Translation2d point6 = new Translation2d(5.4, 3.81);
    private static Translation2d point7 = new Translation2d(6.7, 3.81);


    private static Pose2d end =   new Pose2d(8.992, 3.81, new Rotation2d(0));

    /**
     * Generates the Trajectory
     * @return the generated trajectory
     */
    public static Trajectory getTrajectory()
    {
        return TrajectoryGenerator.generateTrajectory(
           start, List.of(point1, point2 ,point3, point4, point5, point6, point7), end,
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