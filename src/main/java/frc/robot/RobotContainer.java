// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HIDConstants;
import frc.robot.commands.AutonomousDrive;

import frc.robot.commands.IntakeFlip;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.SwerveDriveCommand;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.paths.BarrelRacePath;
import frc.robot.paths.BlueAPath;
import frc.robot.paths.BlueBPath;
import frc.robot.paths.BouncePath;
import frc.robot.paths.FiveMeterPath;
import frc.robot.paths.RedAPath;
import frc.robot.paths.RedBPath;
import frc.robot.paths.SlalomPath;
import frc.robot.paths.TrajectoriesExporter;



import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ForceExtendHood;
import frc.robot.commands.ForceRetractHood;
import frc.robot.commands.HoodAndFlywheelCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TargetCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

   // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
    
    private final DriveSubsystem m_robotDrive = new DriveSubsystem(false);

    //private final SwerveModule m_Module = new SwerveModule();

    private final AutonomousDrive m_autonomousDrive = new AutonomousDrive(m_robotDrive);

    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();

    private final IndexerSubsystem m_indexer = new IndexerSubsystem();

    //private final SwerveModule m_Module = new SwerveModule();

    // The driver's controller
    XboxController m_driverController = new XboxController(HIDConstants.k_DriverControllerPort);

    private final IntakeRun m_intakeRun = new IntakeRun(m_intake);
    private final IntakeStop m_intakeStop = new IntakeStop(m_intake);
    private final IntakeReverse m_intakeReverse = new IntakeReverse(m_intake);

    //private final SwerveModule m_Module = new SwerveModule();

    //The Button Binding Names
    public static JoystickButton intakeFlip, intakeRun, intakeStop, intakeReverse;

    private final TargetCommand m_target = new TargetCommand(m_shooter,m_robotDrive, m_driverController);

    private final IndexerCommand m_index = new IndexerCommand(m_robotDrive, m_driverController, m_indexer);

    private final HoodAndFlywheelCommand m_hood = new HoodAndFlywheelCommand(m_shooter);

    private final SequentialCommandGroup TargetAndHood = new SequentialCommandGroup(m_target, m_hood);

    private final ForceExtendHood m_forceExtend = new ForceExtendHood(m_shooter);
    private final ForceRetractHood m_forceRetract = new ForceRetractHood(m_shooter);



    JoystickButton indexerRun, prepareShooter, actuateHood;
    POVButton  forceExtendHood, forceRetractHood;
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    { 
        // var traj = BlueBPath.getTrajectory();
        // TrajectoriesExporter.exportTrajectoryToCSV(traj, BlueBPath.getTrajectoryName());
        // TrajectoriesExporter.exportTrajectoryToHumanReadable(traj, BlueBPath.getTrajectoryName());

        configureButtonBindings();
       m_robotDrive.setDefaultCommand(new SwerveDriveCommand(m_robotDrive, m_driverController));

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        m_robotDrive.resetIMU();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        intakeFlip = new JoystickButton(m_driverController , HIDConstants.kA);
        intakeRun = new JoystickButton(m_driverController , HIDConstants.kB);
        //intakeReverse = new JoystickButton(m_driverController, HIDConstants.kStart);
        indexerRun = new JoystickButton(m_driverController, HIDConstants.kX);
        prepareShooter = new JoystickButton(m_driverController, HIDConstants.kRB);
        forceExtendHood = new POVButton(m_driverController, HIDConstants.kDL);
        forceRetractHood = new POVButton(m_driverController, HIDConstants.kDR);

        intakeFlip.toggleWhenPressed(new StartEndCommand(m_intake::extendIntake, m_intake::retractIntake, m_intake));
        intakeRun.whileHeld(m_intakeRun);
        intakeReverse.whileHeld(m_intakeReverse);

        forceExtendHood.whileHeld(m_forceExtend);
        forceRetractHood.whileHeld(m_forceRetract);

        prepareShooter.whileHeld(TargetAndHood);

        indexerRun.whileHeld(m_index);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An ExampleCommand will run in autonomous
        return m_autonomousDrive;

    }
}