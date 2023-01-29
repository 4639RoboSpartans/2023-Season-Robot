
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ArmCommand;
import frc.robot.Commands.ArmTestCommand;
import frc.robot.Commands.CloseClawCommand;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.OpenClawCommand;
import frc.robot.Subsystem.ArmPivotSubsystem;
import frc.robot.Subsystem.ClawSubsystem;
import frc.robot.Subsystem.DriveSubsystem;
import frc.robot.Subsystem.ObstructionSensor;
import frc.robot.Constants;

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
    // The robot's subsystems and commands are defined here...
    private final OI oi = new OI();
    public DriveSubsystem m_drive = new DriveSubsystem();
    public DriveCommand drive_cmd = new DriveCommand(m_drive, oi);

    private final ClawSubsystem claw = new ClawSubsystem();
    // private final ArmPivotSubsystem arm = new ArmPivotSubsystem();
    private final ObstructionSensor sensor = new ObstructionSensor(1);

    private Trigger clawObstructedTrigger;
    private double lastClawOpenTime = Double.NEGATIVE_INFINITY;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        m_drive.setDefaultCommand(drive_cmd);
        // arm.setDefaultCommand(new ArmTestCommand(arm, oi));
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

        clawObstructedTrigger.onTrue(new CloseClawCommand(claw));

        oi.getButton(0, Constants.Buttons.B_BUTTON).onTrue(
            new OpenClawCommand(claw)
            .andThen(() -> {
                lastClawOpenTime = Timer.getFPGATimestamp();
            }, claw)
        );

        oi.getButton(0, Constants.Buttons.A_BUTTON).onTrue(
            new CloseClawCommand(claw)
        );

        oi.getButton(0, Constants.Buttons.LEFT_STICK).onTrue(new RunCommand(() -> {
            
            SmartDashboard.putNumber("Encoder1", m_drive.SwerveMod1FrontRight.getRotationInDegrees());
            SmartDashboard.putNumber("Encoder2", m_drive.SwerveMod2FrontLeft.getRotationInDegrees());
            SmartDashboard.putNumber("Encoder3", m_drive.SwerveMod3RearLeft.getRotationInDegrees());
            SmartDashboard.putNumber("Encoder4", m_drive.SwerveMod4RearRight.getRotationInDegrees());
            SmartDashboard.putNumber("RobotHeading", m_drive.getHeading());
            SmartDashboard.putNumber("LeftStickX", oi.getAxis(0, Constants.Axes.LEFT_STICK_X));
            SmartDashboard.putNumber("LeftStickY", oi.getAxis(0, Constants.Axes.LEFT_STICK_Y));
            SmartDashboard.putNumber("RightStickX", oi.getAxis(0, Constants.Axes.RIGHT_STICK_X));
            SmartDashboard.putNumber("RightStickY", oi.getAxis(0, Constants.Axes.RIGHT_STICK_Y));
            
            m_drive.resetAnglesAndPositions();
        }, m_drive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

    public void periodic(){
        SmartDashboard.putNumber("TestKey: System Time", System.currentTimeMillis());

        SmartDashboard.putBoolean("ClawObstructed", clawObstructedTrigger.getAsBoolean());
        SmartDashboard.putBoolean("ClawObstructedRaw", sensor.isObstructed());
    }
}
