// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.paths;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Constants;



/** The Trajectory class expects a List<State> when being instantiated.
 * This class converts the arrays loaded in from the PathPlanner tool 
 * and creates a List<State> object to pass into the Trajectory 
 * class constructor.
 */
public class TrajectoryMaker 
{
    private final static int TIME = 0;
    private final static int VELOCITY = 1;
    private final static int ACCEL = 2;
    private final static int X = 3;
    private final static int Y = 4;
    private final static int HOLOHEADING = 5; //Needs converted to radians
    private final static int CURVERAD = 6;


    public static Trajectory MakeATrajectory()
    {
        List<Trajectory.State> states = new ArrayList<>();
        for(int i = 0; i < Y5MeterPath.Y5MeterPath.length - 1 ; i++)
        {

            double timeAtI = Y5MeterPath.Y5MeterPath[i][TIME];
            double velocityAtI = Y5MeterPath.Y5MeterPath[i][VELOCITY];
            double accelerationAtI = Y5MeterPath.Y5MeterPath[i][ACCEL];
            double xAtI = Y5MeterPath.Y5MeterPath[i][X];
            double yAtI = Y5MeterPath.Y5MeterPath[i][Y];
            double headingAtI = Y5MeterPath.Y5MeterPath[i][HOLOHEADING] * Math.PI/ 180;
            double curveAtI = Y5MeterPath.Y5MeterPath[i][CURVERAD] * Math.PI/ 180 / Constants.SwerveDriveModuleConstants.k_RobotRadius;

            Rotation2d rotation = new Rotation2d(headingAtI);
            Pose2d Pose = new Pose2d(xAtI, yAtI, rotation);
        
            Trajectory.State stateI = new Trajectory.State(timeAtI, velocityAtI, accelerationAtI, Pose, curveAtI);
            states.add(stateI);
        }

        return new Trajectory(states);
    }
}
