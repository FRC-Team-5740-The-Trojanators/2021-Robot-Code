// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.paths;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;



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
        for(int i = 0; i < RotateInPlacePath.RotateInPlacePath.length - 1 ; i++)
        {

            double timeAtI = RotateInPlacePath.RotateInPlacePath[i][TIME];
            double velocityAtI = RotateInPlacePath.RotateInPlacePath[i][VELOCITY];
            double accelerationAtI = RotateInPlacePath.RotateInPlacePath[i][ACCEL];
            double xAtI = RotateInPlacePath.RotateInPlacePath[i][X];
            double yAtI = RotateInPlacePath.RotateInPlacePath[i][Y];
            double headingAtI = RotateInPlacePath.RotateInPlacePath[i][HOLOHEADING] * Math.PI/ 180;
            double curveAtI = RotateInPlacePath.RotateInPlacePath[i][CURVERAD] * Math.PI/ 180;

            Rotation2d rotation = new Rotation2d(headingAtI);
            Pose2d Pose = new Pose2d(xAtI, yAtI, rotation);
        
            Trajectory.State stateI = new Trajectory.State(timeAtI, velocityAtI, accelerationAtI, Pose, curveAtI);
            states.add(stateI);
        }

        return new Trajectory(states);
    }
}
