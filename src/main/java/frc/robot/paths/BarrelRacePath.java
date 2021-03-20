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

public class BarrelRacePath
{
    private static String trajectoryName = "Barrel Race Path";

    private static TrajectoryConfig trajectoryConfiguration = new TrajectoryConfig(Constants.SwerveDriveModuleConstants.k_MaxSpeed, 1)
                                                            .setKinematics(Constants.SwerveDriveModuleConstants.kinematics);


                                            // x    y      heading
    private static Pose2d start = new Pose2d(.9652, 2.08, new Rotation2d(0));

    private static Translation2d point1 = new Translation2d(3.81, 2.286 );
    private static Translation2d point2 = new Translation2d(4.318, 1.27 );
    private static Translation2d point3 = new Translation2d(3.302, 1.22 );
    private static Translation2d point4 = new Translation2d(3.81, 2.08 );
    private static Translation2d point5 = new Translation2d(5.89, 2.49 );
    private static Translation2d point6 = new Translation2d(6.5, 3.429 );
    private static Translation2d point7 = new Translation2d(5.41, 3.3 );
    private static Translation2d point8 = new Translation2d(5.89, 2.2 );
    private static Translation2d point9 = new Translation2d(7.37, 1.09 );
    private static Translation2d point10 = new Translation2d(8.255, 1.524 );
    private static Translation2d point11 = new Translation2d(7.366, 2.26 );
    private static Translation2d point12 = new Translation2d(4.06, 2.57 );

    private static Pose2d end = new Pose2d(.965, 2.54, new Rotation2d(0));

    /**
     * Generates the Trajectory
     * @return the generated trajectory
     */
    public static Trajectory getTrajectory()
    {
        return TrajectoryGenerator.generateTrajectory(
            start, List.of( point1, point2, point3, point4, point5, point6, point7, point8, point9, point10, point11,
            point12), end,
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