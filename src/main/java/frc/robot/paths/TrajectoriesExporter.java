
package frc.robot.paths;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

/**
 * Contains static methods to export a Trajectory to a CSV or a human readable text filename.
 */
public class TrajectoriesExporter 
{
    /**
     * Exports the Trajectory file to a CVS. Uses the name of the Class as the CSV file.
     * @param trajectory The Trajectory to export.
     */
    public static void exportTrajectoryToCSV(Trajectory trajectory)
    {
        exportTrajectoryToCSV(trajectory, trajectory.getClass().getName());
    }

    /**
     * Exports the Trajectory file to a CVS. Uses the provided filename for the CSV filename.
     * @param trajectory The Trajectory to export.
     * @param filename The filename to export to.
     */
    public static void exportTrajectoryToCSV(Trajectory trajectory, String filename)
    {
        try (PrintWriter writer = new PrintWriter(new File(filename + ".csv")))
        {
            var states = trajectory.getStates();
            StringBuilder sb = new StringBuilder("Time (s), Velocity (m/s), Acceleration (m/s/s), X (m), Y (m), Rotation (rads), Rotation (deg), Curvature (rads/m)\n\n");
            
            for (State state : states)
            {
                sb.append(String.format("%.2f", state.timeSeconds));
                sb.append(",");
                sb.append(String.format("%.2f", state.velocityMetersPerSecond));
                sb.append(",");
                sb.append(String.format("%.2f", state.accelerationMetersPerSecondSq));
                sb.append(",");
                sb.append(String.format("%.2f", state.poseMeters.getX()));
                sb.append(",");
                sb.append(String.format("%.2f", state.poseMeters.getY()));
                sb.append(",");
                sb.append(String.format("%.4f", state.poseMeters.getRotation().getRadians()));
                sb.append(",");
                sb.append(String.format("%.2f", state.poseMeters.getRotation().getDegrees()));
                sb.append(",");
                sb.append(String.format("%.2f", state.curvatureRadPerMeter));
                sb.append("\n");
            }

            writer.write(sb.toString());
            writer.close();
        }
        catch (IOException iox)
        {
            System.out.println("A Buffered Writer issue occured: " + iox.getMessage());
        }
    }


    /**
     * Exports the Trajectory file to a Human readable text file. Uses the name of the Class as the txt filename.
     * @param trajectory The Trajectory to export.
     */
    public static void exportTrajectoryToHumanReadable(Trajectory trajectory)
    {
        exportTrajectoryToHumanReadable(trajectory, trajectory.getClass().getName());
    }

    /**
     * Exports the Trajectory file to a human readable text file. Uses the provided filename for the text filename.
     * @param trajectory The Trajectory to export.
     * @param filename The filename to export to.
     */
    public static void exportTrajectoryToHumanReadable(Trajectory trajectory, String filename)
    {
        try (PrintWriter writer = new PrintWriter(new File(filename + ".txt")))
        {
            var states = trajectory.getStates();
            var sb = new StringBuilder("Time (s), Velocity (m/s), Acceleration (m/s/s), X (m), Y (m), Rotation (rads), Rotation (deg), Curvature (rads/m)\n\n");

            for (State state : states)
            {
                sb.append("Time: ");
                sb.append(String.format("%.2f", state.timeSeconds));
                sb.append(" (s),   ");
                
                sb.append("Velocity: ");
                sb.append(String.format("%.2f", state.velocityMetersPerSecond));
                sb.append(" (m/s),   ");

                sb.append("Accel: ");
                sb.append(String.format("%6.2f", state.accelerationMetersPerSecondSq)); //6.2f provides padding for full range of accel values
                sb.append(" (m/s/s),   ");
                
                sb.append("X: ");
                sb.append(String.format("%5.2f", state.poseMeters.getX()));
                sb.append(" (m),   ");

                sb.append("Y: ");
                sb.append(String.format("%5.2f", state.poseMeters.getY()));
                sb.append(" (m),   ");

                sb.append("Rot: ");
                sb.append(String.format("%7.4f", state.poseMeters.getRotation().getRadians()));
                sb.append(" (rads),   ");

                sb.append("Rot: ");
                sb.append(String.format("%7.2f", state.poseMeters.getRotation().getDegrees()));
                sb.append(" (deg),   ");

                sb.append("Curve: ");
                sb.append(String.format("%7.2f", state.curvatureRadPerMeter));
                sb.append(" (rads/m)\n");
            }

            sb.append("\n");
            writer.write(sb.toString());
            writer.close();
        }
        catch (IOException iox)
        {
            System.out.println("A Buffered Writer issue occured: " + iox.getMessage());
        }
    }
}
