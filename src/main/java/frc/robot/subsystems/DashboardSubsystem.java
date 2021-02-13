// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


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

public class DashboardSubsystem extends SubsystemBase {
  /** Creates a new DashboardSubsytems. */
  public DriveSubsystem driver; 

 // private NetworkTableEntry gyroAngle;

  //this.gyroAngle = NetworkTableEntry.add("Drivetrain Gyro", 0).withWidget(BuiltInWidgets.kGyro).putEntry("GyroAngle").putDouble(driver.gyroAngle);


 //public void TeleopDashboard(){
 //final ShuffleboardTab Teleop_Dashboard = Shuffleboard.getTab("TeleopDash");
 //}

  public DashboardSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
