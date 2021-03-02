package frc.robot.commands;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.Constants.SwerveDriveModuleConstants;

public class SwerveDriveCommand extends CommandBase {

private final DriveSubsystem drivetrain;
private final XboxController controller;

        
private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(6);
private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(6);
private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6);


public SwerveDriveCommand(DriveSubsystem drivetrain, XboxController controller) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.controller = controller;
}

@Override
public void execute() {
  // Get the x speed. We are inverting this because Xbox controllers return
  // negative values when we push forward.
  final var xSpeed =
    xspeedLimiter.calculate(controller.getY(GenericHID.Hand.kLeft))
      * SwerveDriveModuleConstants.kMaxSpeed;

  // Get the y speed or sideways/strafe speed. We are inverting this because
  // we want a positive value when we pull to the left. Xbox controllers
  // return positive values when you pull to the right by default.
  final var ySpeed =
    yspeedLimiter.calculate(controller.getX(GenericHID.Hand.kLeft))
      * SwerveDriveModuleConstants.kMaxSpeed;

  // Get the rate of angular rotation. We are inverting this because we want a
  // positive value when we pull to the left (remember, CCW is positive in
  // mathematics). Xbox controllers return positive values when you pull to
  // the right by default.
  final var rot =
    -rotLimiter.calculate(controller.getX(GenericHID.Hand.kRight))
      * SwerveDriveModuleConstants.kMaxAngularSpeed;

  boolean calibrate = controller.getBumper(GenericHID.Hand.kLeft);

  drivetrain.drive(xSpeed, ySpeed, rot, true);
  
}

}
