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

public class BouncePath
{
    private static String trajectoryName = "Bounce Path";

    private static TrajectoryConfig trajectoryConfiguration = new TrajectoryConfig(Constants.SwerveDriveModuleConstants.k_MaxSpeed, 1)
                                                            .setKinematics(Constants.SwerveDriveModuleConstants.kinematics);


                                            // x    y      heading
    private static Pose2d start = new Pose2d(1.016, 2.286, new Rotation2d(0));

    private static  Translation2d point1 = new  Translation2d(2.032, 2.54);
    private static  Translation2d point2 = new  Translation2d(2.35, 3.81);
    private static  Translation2d point3 = new  Translation2d(3.35, 1.010);
    private static  Translation2d point4 = new  Translation2d(4.62, 1.10);
    private static  Translation2d point5 = new  Translation2d(4.64, 2.4);
    private static  Translation2d point6 = new  Translation2d(4.66, 2.9);
    private static  Translation2d point7 = new  Translation2d(4.7, 3.5);
    private static  Translation2d point8 = new  Translation2d(4.95, .49);
    private static  Translation2d point9 = new  Translation2d(6.59, .51);
    private static  Translation2d point10 = new  Translation2d(6.79, 1.5);
    private static  Translation2d point11 = new  Translation2d(6.81, 2.7);
    private static  Translation2d point12 = new  Translation2d(6.82, 3.3);
    private static  Translation2d point13 = new  Translation2d(7.11, 1.91);

    private static Pose2d end = new Pose2d(8.4, 1.8, new Rotation2d(0));

    /**
     * Generates the Trajectory
     * @return the generated trajectory
     */
    public static Trajectory getTrajectory()
    {
        return TrajectoryGenerator.generateTrajectory(
           start, List.of(point1, point2, point3, point4, point5, point6, point7, point8, point9, point10, point11, point12, point13), end,
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