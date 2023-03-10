
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auton.AutoBalanceCommand;
import frc.robot.commands.*;
import frc.robot.math.vec2;
//import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ObstructionSensor;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.swerve.SwerveMovement;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.NavX;
import frc.robot.network.vision.LimeLight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    public final OI oi = new OI();
    public final NavX navx = new NavX();
    public final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(navx);
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    public final ObstructionSensor clawObstructionSensor = new ObstructionSensor(1); //subject to change port to analog
    public final ArmPivotSubsystem armPivotSubsystem = new ArmPivotSubsystem();
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem();

    public final WristSubsystem wristSubsystem = new WristSubsystem();
    private final Trigger clawObstructedTrigger;
    private double lastClawOpenTime = Double.NEGATIVE_INFINITY;

    
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        enableCompressor();
    
        swerveDriveSubsystem.setDefaultCommand(new DriveCommand(swerveDriveSubsystem, oi, navx));
        elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem, oi));
        wristSubsystem.setDefaultCommand(new WristCommand(wristSubsystem, oi));
        telescopeSubsystem.setDefaultCommand(new TelescopeCommand(telescopeSubsystem, oi));
        armPivotSubsystem.setDefaultCommand(new ArmCommand(armPivotSubsystem, oi));

        clawObstructedTrigger = new Trigger(() ->
            clawObstructionSensor.isObstructed()
            && Timer.getFPGATimestamp() > lastClawOpenTime + Constants.Timing.CLAW_DELAY_AFTER_OPEN
        );

        configureButtonBindings();
        // configureClaw();
    }

  
    public void enableCompressor() {
        Compressor pcmCompressor = new Compressor(Constants.IDs.PNEUMATIC_HUB, PneumaticsModuleType.REVPH);
        // Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

        pcmCompressor.enableDigital();
        // pcmCompressor.disable();

    }

    /*
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // configureClaw();

        // oi.getButton(0, Constants.Buttons.Y_BUTTON)
        //         .onTrue(new RunCommand(() -> elevatorSubsystem.setSpeed(0), elevatorSubsystem));

        // oi.getPovButton(0, 90).whileTrue(new RunCommand(() -> elevatorSubsystem.setSpeed(elevatorSubsystem.getEncoderPos() + .03), elevatorSubsystem));
        // oi.getPovButton(0, 270).whileTrue(new RunCommand(() -> elevatorSubsystem.setSpeed(elevatorSubsystem.getEncoderPos() - .03), elevatorSubsystem));

        oi.getButton(0, Constants.Buttons.A_BUTTON).whileTrue(new AutoBalanceCommand(swerveDriveSubsystem, navx));

        // oi.getButton(1, Constants.Buttons.B_BUTTON).whileTrue(new ArmCommand(armPivotSubsystem, oi));
        oi.getButton(1, Constants.Buttons.A_BUTTON).onTrue(new OpenClawCommand(clawSubsystem));
        oi.getButton(1, Constants.Buttons.X_BUTTON).onTrue(new CloseClawCommand(clawSubsystem));
    }

    private void configureClaw() {
        clawObstructedTrigger.onTrue(new CloseClawCommand(clawSubsystem).andThen(() -> {
            lastClawOpenTime = Double.POSITIVE_INFINITY;
        }, clawSubsystem));
        oi.getButton(1, Constants.Buttons.A_BUTTON).onTrue(
            new OpenClawCommand(clawSubsystem)
            .andThen(() -> lastClawOpenTime = Timer.getFPGATimestamp(), clawSubsystem)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
//     String trajectoryJSON = "pathplanner/generatedJSON/TestTestPath.wpilib.json";
// Trajectory trajectory = new Trajectory();

    public Command getAutonomousCommand() {
                PathPlannerTrajectory traj1 = PathPlanner.generatePath(
    new PathConstraints(0.5, 0.5), 
    new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position, heading(direction of travel), holonomic rotation
    new PathPoint(new Translation2d(-1.4,0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(10))
    // new PathPoint(new Translation2d(1,0.5), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
    );

         PIDController xController = new PIDController(0.001, 0, 0);
         PIDController yController = new PIDController(0.001, 0, 0);
        //  PIDController thetaController = new PIDController(0.001, 0, 0);
         ProfiledPIDController thetaController = new ProfiledPIDController(
                1, 0.7, 0, new TrapezoidProfile.Constraints(1, 1));
         thetaController.enableContinuousInput(-180 ,180);
// \
         SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                traj1,
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem.getKinematics(),
                xController,
                yController,
                thetaController,
                swerveDriveSubsystem::setModulesStatess,
                swerveDriveSubsystem
                );


                // return swerPveControllerCommand;
                return new AutonCubeExtend(wristSubsystem,clawSubsystem, elevatorSubsystem, armPivotSubsystem, telescopeSubsystem).withTimeout(3).andThen(swerveControllerCommand);
                // return null;
                // return null;
                // .andThen(new OpenClawCommand(clawSubsystem)).withTimeout(0.1);
                // andThen(new AutonCubeRetract(wristSubsystem, clawSubsystem, elevatorSubsystem, armPivotSubsystem, telescopeSubsystem)).withTimeout(3);
                // andThen(swerveControllerCommand);

    }
}
