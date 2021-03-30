package frc.robot.commands;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.Constants.HIDConstants;
import frc.robot.Constants.SwerveDriveModuleConstants;

public class SwerveDriveCommand extends CommandBase 
{
    private final DriveSubsystem drivetrain;
    private final XboxController controller;

            
    private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(6);
    private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(6);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(15);


    public SwerveDriveCommand(DriveSubsystem drivetrain, XboxController controller) 
    {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        this.controller = controller;
    }

    private double getJoystickWithDeadBand(double stickValue)
    {
        if(stickValue > HIDConstants.kDeadBand || stickValue < -HIDConstants.kDeadBand)
        {
            if(stickValue < 0)
            {
                return stickValue = -Math.pow(stickValue, 2);
            }
            else
            {
                return stickValue = Math.pow(stickValue, 2);
            }
        } 
        else 
        {
            return 0;
        }
    } 

    @Override
    public void execute()
    {
        // Get the x speed.
        final var xSpeed =
            -xspeedLimiter.calculate(getJoystickWithDeadBand(controller.getY(GenericHID.Hand.kLeft))
            * SwerveDriveModuleConstants.k_MaxSpeed * SwerveDriveModuleConstants.kXYjoystickCoefficient);
            //SmartDashboard.putNumber("xspeed", xSpeed);

        // Get the y speed or sideways/strafe speed. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed =
            -yspeedLimiter.calculate(getJoystickWithDeadBand(controller.getX(GenericHID.Hand.kLeft))
            * SwerveDriveModuleConstants.k_MaxSpeed * SwerveDriveModuleConstants.kXYjoystickCoefficient);
            //SmartDashboard.putNumber("yspeed", ySpeed);

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot =
            -rotLimiter.calculate(getJoystickWithDeadBand(controller.getX(GenericHID.Hand.kRight))
            * SwerveDriveModuleConstants.kMaxAngularSpeed * SwerveDriveModuleConstants.kRotCoefficent);
            SmartDashboard.putNumber("rot", rot);

        boolean calibrate = controller.getBumper(GenericHID.Hand.kLeft);

        drivetrain.drive(xSpeed, ySpeed, rot, true);
       //drivetrain.drive(1, 0, 0, true);

        SmartDashboard.putNumber("Commanded X Velocity", xSpeed);
        SmartDashboard.putNumber("Commanded Y Velocity", ySpeed);
       // SmartDashboard.putNumber("Commanded Rot Speed", rot); Already put on above

        SmartDashboard.putNumber("Reading X Velocity LeftFront", drivetrain.getModules()[0].getDriveVelocity());
        SmartDashboard.putNumber("Reading X Velocity RightFront", drivetrain.getModules()[1].getDriveVelocity());
        SmartDashboard.putNumber("Reading X Velocity LeftRear", drivetrain.getModules()[2].getDriveVelocity());
        SmartDashboard.putNumber("Reading X Velocity RightRear", drivetrain.getModules()[3].getDriveVelocity());

        // SmartDashboard.putNumber("Reading Y Velocity", ySpeed);
        // SmartDashboard.putNumber("Reading Rot Speed", rot);

        SmartDashboard.putNumber("Current X Position", drivetrain.getPose().getX());
        SmartDashboard.putNumber("Current Y Position", drivetrain.getPose().getY());

        SmartDashboard.putNumber("DRIVE P", drivetrain.modules[0].getDrivePIDF("p"));

        SmartDashboard.putNumber(("Get Position LeftFront"), drivetrain.getModules()[0].getSparkMaxPosition());
        SmartDashboard.putNumber(("Get Position RightFront"), drivetrain.getModules()[1].getSparkMaxPosition());
        SmartDashboard.putNumber(("Get Position LeftRear"), drivetrain.getModules()[2].getSparkMaxPosition());
        SmartDashboard.putNumber(("Get Position RightRear"), drivetrain.getModules()[3].getSparkMaxPosition());

        SmartDashboard.putNumber(("Get Rotation LeftFront"), drivetrain.getModules()[0].getRotationDegrees());
        SmartDashboard.putNumber(("Get Rotation RightFront"), drivetrain.getModules()[1].getRotationDegrees());
        SmartDashboard.putNumber(("Get Rotation LeftRear"), drivetrain.getModules()[2].getRotationDegrees());
        SmartDashboard.putNumber(("Get Rotation RightRear"), drivetrain.getModules()[3].getRotationDegrees());


        
    }
}
