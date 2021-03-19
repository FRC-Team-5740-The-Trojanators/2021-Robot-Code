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

public class BarrelRacePath
{
    private static String trajectoryName = "Barrel Race Path";

    private static TrajectoryConfig trajectoryConfiguration = new TrajectoryConfig(Constants.SwerveDriveModuleConstants.k_MaxSpeed, 1)
                                                            .setKinematics(Constants.SwerveDriveModuleConstants.kinematics);


                                            // x    y      heading
    private static Pose2d start = new Pose2d(0.0, 0.0, new Rotation2d(Math.PI/2));

    private static Pose2d point1 = new Pose2d(2.3, 1.5, Rotation2d.fromDegrees(0));
    private static Pose2d point2 = new Pose2d(2.5, 2.8, Rotation2d.fromDegrees(0));
    private static Pose2d point3 = new Pose2d(2.3, 3.8, Rotation2d.fromDegrees(0));
    private static Pose2d point4 = new Pose2d(1.8, 4.2, Rotation2d.fromDegrees(0));
    private static Pose2d point5 = new Pose2d(1.3, 4.3, Rotation2d.fromDegrees(0));
    private static Pose2d point6 = new Pose2d(1, 3.8, Rotation2d.fromDegrees(0));
    private static Pose2d point7 = new Pose2d(1.3, 3.3, Rotation2d.fromDegrees(0));
    private static Pose2d point8 = new Pose2d(2, 3.6, Rotation2d.fromDegrees(0));
    private static Pose2d point9 = new Pose2d(2.3, 4.3, Rotation2d.fromDegrees(0));
    private static Pose2d point10 = new Pose2d(2.5, 5.6, Rotation2d.fromDegrees(0));
    private static Pose2d point11 = new Pose2d(2.8, 6.6, Rotation2d.fromDegrees(0));
    private static Pose2d point12 = new Pose2d(3.4, 6.6, Rotation2d.fromDegrees(0));
    private static Pose2d point13 = new Pose2d(3.4, 5.3, Rotation2d.fromDegrees(0));
    private static Pose2d point14 = new Pose2d(1.8, 6.1, Rotation2d.fromDegrees(0));
    private static Pose2d point15 = new Pose2d(1.3, 7.4, Rotation2d.fromDegrees(0));
    private static Pose2d point16 = new Pose2d(1.3, 8, Rotation2d.fromDegrees(0));
    private static Pose2d point17 = new Pose2d(1.8, 8.4, Rotation2d.fromDegrees(0));
    private static Pose2d point18 = new Pose2d(2.2, 8.1, Rotation2d.fromDegrees(0));
    private static Pose2d point19 = new Pose2d(2.3, 7.9, Rotation2d.fromDegrees(0));
    private static Pose2d point20 = new Pose2d(2.8, 4.1, Rotation2d.fromDegrees(0));
    private static Pose2d point21 = new Pose2d(2.8, 1.5, Rotation2d.fromDegrees(0));

    private static Pose2d end =    new Pose2d(3.25, 3.00, new Rotation2d(Math.PI));

    /**
     * Generates the Trajectory
     * @return the generated trajectory
     */
    public static Trajectory getTrajectory()
    {
        return TrajectoryGenerator.generateTrajectory(
            List.of(start, point1, point2, point3, point4, point5, point6, point7, point8, point9, point10, point11, point12, point13, point14, point15, point16, point17, point18, point19, point20, point21, end), 
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