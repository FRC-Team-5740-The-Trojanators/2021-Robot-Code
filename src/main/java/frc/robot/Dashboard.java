package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;

public class Dashboard {

public DriveSubsystem driver;
public Constants constants;
public SwerveModule swerve;

private NetworkTableEntry gyro;
private NetworkTableEntry steeringPID;
private NetworkTableEntry drivePID;

public Dashboard(DriveSubsystem m_DriveSubsystem, Constants m_Constants, SwerveModule m_SwerveModule ){
    this.driver = m_DriveSubsystem;
    this.constants = m_Constants;
    this.swerve = m_SwerveModule;

    TeleopDashboard();
}
    


public void TeleopDashboard() {

final ShuffleboardTab Teleop_Dashboard = Shuffleboard.getTab("TeleopDash");

this.drivePID = Teleop_Dashboard.add("Drive PID", 0).withWidget(BuiltInWidgets.kPIDController).getEntry();
this.steeringPID = Teleop_Dashboard.add("Steering PID", 0).withWidget(BuiltInWidgets.kPIDController).getEntry();
this.gyro = Teleop_Dashboard.add("Gyro", 0).withWidget(BuiltInWidgets.kGyro).getEntry();

}

public void dashboardData() throws NullPointerException {

    try {
        
    } catch (final Exception e) {

    e.printStackTrace();
}  

}}
