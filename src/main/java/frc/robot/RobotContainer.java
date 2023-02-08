
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmTestCommand;
import frc.robot.commands.CloseClawCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.OpenClawCommand;
import frc.robot.commands.navXCommand;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ObstructionSensor;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.navXSubsystem;
import frc.robot.util.network.vision.LimeLight;

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
    private final OI oi = new OI();

    // The robot's subsystems and commands are defined here...
    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    private final ClawSubsystem claw = new ClawSubsystem();
    private final ObstructionSensor sensor = new ObstructionSensor(0);
    private final ArmPivotSubsystem arm = new ArmPivotSubsystem();

    private final navXSubsystem navx = new navXSubsystem();
    private Trigger clawObstructedTrigger;
    private double lastClawOpenTime = Double.NEGATIVE_INFINITY;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        swerveDriveSubsystem.setDefaultCommand(new DriveCommand(swerveDriveSubsystem, oi));
        arm.setDefaultCommand(new ArmTestCommand(arm, oi));
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        clawObstructedTrigger = new Trigger(() -> {
            return sensor.isObstructed() 
                && Timer.getFPGATimestamp() > lastClawOpenTime + Constants.Timing.CLAW_DELAY_AFTER_OPEN;
        });

        clawObstructedTrigger.onTrue(new CloseClawCommand(claw).andThen(() -> {
            lastClawOpenTime = Double.POSITIVE_INFINITY;
        }, claw));

        oi.getButton(0, Constants.Buttons.B_BUTTON).onTrue(
            new OpenClawCommand(claw)
            .andThen(() -> {
                lastClawOpenTime = Timer.getFPGATimestamp();
            }, claw)
        );

        oi.getButton(0, Constants.Buttons.X_BUTTON).onTrue(
            new navXCommand(swerveDriveSubsystem, navx)
        );
        
        oi.getButton(0, Constants.Buttons.A_BUTTON).onTrue(
            new CloseClawCommand(claw)
        );

        oi.getButton(0, Constants.Buttons.Y_BUTTON).whileTrue(new CommandBase(){
            {
                addRequirements(swerveDriveSubsystem);
            }


            @Override
            public void execute(){
                swerveDriveSubsystem.setModules(new SwerveModuleState(
                    1,
                    new Rotation2d()
                ));
            }
        });
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

    public void periodic(){
        LimeLight.getOffsetFromCenteredAprilTag();
    }
}
