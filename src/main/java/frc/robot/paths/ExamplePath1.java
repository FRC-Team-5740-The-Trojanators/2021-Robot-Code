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

/**
 * An example of a Trajectory Generator. Replace the name of the Trajectory and the points to match 
 * we actually need to follow.
 */
public class ExamplePath1
{
    private static String trajectoryName = "Example Trajectory 1"; //!!!! Change this for each new trajectory!

    private static double maxVelocityMetersPerSecond = Constants.SwerveDriveModuleConstants.k_MaxSpeed;
    private static double maxAccelerationMetersPerSecondSq = Constants.SwerveDriveModuleConstants.k_MaxAcceleration;

    private static TrajectoryConfig trajectoryConfiguration = new TrajectoryConfig(
                        maxVelocityMetersPerSecond, 
                        maxAccelerationMetersPerSecondSq)
                        .setKinematics(Constants.SwerveDriveModuleConstants.kinematics);

                                    // x    y      heading
    private static Pose2d start = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

    private static Pose2d point1 = new Pose2d(1.00, 0.00, Rotation2d.fromDegrees( 0.0));
    private static Pose2d point2 = new Pose2d(2.50, 0.50, Rotation2d.fromDegrees(45.0));
    private static Pose2d point3 = new Pose2d(3.75, 1.00, Rotation2d.fromDegrees(90.0));
    private static Pose2d point4 = new Pose2d(4.50, 1.00, Rotation2d.fromDegrees(135.0));
    private static Pose2d point5 = new Pose2d(5.25, 0.50, new Rotation2d(Math.PI));

    private static Pose2d end =    new Pose2d(6.00, 0.00, new Rotation2d(Math.PI));

    /**
     * Generates the Trajectory
     * @return the generated trajectory
     */
    public static Trajectory getTrajectory()
    {
        return TrajectoryGenerator.generateTrajectory(
            List.of(start, point1, point2, point3, point4, point5, end), 
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