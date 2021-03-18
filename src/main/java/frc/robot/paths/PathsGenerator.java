
package frc.robot.paths;

import java.util.List;

import com.fasterxml.jackson.core.JsonProcessingException;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.robot.Constants;

public class PathsGenerator {

    private Trajectory m_trajectory;

    private TrajectoryConfig m_config;

    // m_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new
    // Rotation2d(0)), List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
    // new Pose2d(3, 0, new Rotation2d(0)), config);

    // m_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new
    // Rotation2d(0)), List.of(new Translation2d(0, 1), new Translation2d(0, 1.5),
    // new Translation2d(0, 2), new Translation2d(0, 2.5)), new Pose2d(0, 3, new
    // Rotation2d(0)), config);

    public PathsGenerator() 
    {
        m_config = new TrajectoryConfig(Constants.SwerveDriveModuleConstants.kMaxSpeed, 1)
                .setKinematics(Constants.SwerveDriveModuleConstants.kinematics);

        m_trajectory = TrajectoryGenerator
                .generateTrajectory(List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(2.8, 0, new Rotation2d(0)),
                        new Pose2d(3, 0, new Rotation2d(0)), new Pose2d(3, 1, new Rotation2d(0))), m_config);

    }

    public Trajectory getTrajectory() {
        return m_trajectory;
    }

    public void exportTrajectory() //throws JsonProcessingException
    {
        String traj = new String();
        //Time, Velocity, Acceleration, X, Y, rotation, curvature
    

        try
        {
            traj = TrajectoryUtil.serializeTrajectory(m_trajectory);
        }
        catch (JsonProcessingException e)
        {
            System.out.println("Exception found: " + e.getMessage());
        }

        try(PrintWriter writer = new PrintWriter(new File("test.csv")))
        {
            var states = m_trajectory.getStates();
            StringBuilder SB = new StringBuilder("Time, Velocity, Acceleration, X, Y, rotation, curvature");
            //BufferedWriter writer = new BufferedWriter(new FileWriter("testTraj.txt"));
            // SB.append('\n');
            // SB.append(states.get(0));
            // SB.append('\n');
            // SB.append(states.get(1));
            // SB.append('\n');

            for (State state : states) {
                SB.append("\n");
                SB.append(state);
            }

            writer.write(SB.toString());

            writer.close();
        }
        catch (IOException iox)
        {
            System.out.println("A buffered Writer issue occured: " + iox.getMessage());
        }
    }
    



}
