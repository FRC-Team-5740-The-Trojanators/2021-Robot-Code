// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Written by Team 5470

package frc.robot.paths;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

public class FiveMeterPath
{
    private static String trajectoryName = "Five Meter Path";

    private static TrajectoryConfig trajectoryConfiguration = new TrajectoryConfig(Constants.SwerveDriveModuleConstants.k_MaxSpeed, 1)
                                                            .setKinematics(Constants.SwerveDriveModuleConstants.kinematics);


                                            // x    y      heading
    private static Pose2d start = new Pose2d(0.0, 0.0, new Rotation2d(0));

    private static Pose2d point1 = new Pose2d(1, 0, new Rotation2d(0));
    private static Pose2d point2 = new Pose2d(2, 0, new Rotation2d(0));
    private static Pose2d point3 = new Pose2d(3, 0, new Rotation2d(0));
    private static Pose2d point4 = new Pose2d(4, 0, new Rotation2d(0));

    private static Pose2d end =    new Pose2d(5, 0, new Rotation2d(0));

    /**
     * Generates the Trajectory
     * @return the generated trajectory
     */
    public static Trajectory getTrajectory()
    {
        return TrajectoryGenerator.generateTrajectory(
            List.of(start, point1, point2 ,point3 ,point4, end), 
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