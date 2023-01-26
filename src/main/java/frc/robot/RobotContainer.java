
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.OI;
import frc.robot.Commands.ArmCommand;
import frc.robot.Commands.ParhamExtendCommand;
import frc.robot.Commands.ParhamRetractCommand;
import frc.robot.Commands.PneumaticsExtendCommand;
import frc.robot.Commands.PneumaticsRetractCommand;
import frc.robot.Subsystem.ArmPivotSubsystem;
import frc.robot.Subsystem.ParhamClawSubsystem;
import frc.robot.Subsystem.PneumaticSubsystem;
import frc.robot.Subsystem.SensorSubsystem;
import frc.robot.Constants;
/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {

  private static final boolean useParhamClaw = false;

 // The robot's subsystems and commands are defined here...
 private final OI oi = new OI();
 //private final PneumaticSubsystem pneumatics= new PneumaticSubsystem();
 private final Subsystem claw = useParhamClaw ? new ParhamClawSubsystem() : new PneumaticSubsystem();
  private final ArmPivotSubsystem  arm = new ArmPivotSubsystem();
  private final SensorSubsystem sensor = new SensorSubsystem();
  

 /** The container for the robot. Contains subsystems, OI devices, and commands. */
 public RobotContainer() {
   // Configure the button bindings
   configureButtonBindings();
 }


 /**
  * Use this method to define your button->command mappings. Buttons can be created by
  * instantiating a {@link GenericHID} or one of its subclasses ({@link
  * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
  * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
  */
 private void configureButtonBindings() {

    if(useParhamClaw){
      oi.getButton(0,Constants.Buttons.B_BUTTON).whileTrue(new ParhamRetractCommand((ParhamClawSubsystem)claw));
      oi.getButton(0,Constants.Buttons.A_BUTTON).whileTrue(new ParhamExtendCommand((ParhamClawSubsystem)claw));
    }
    else{
      oi.getButton(0,Constants.Buttons.B_BUTTON).onTrue(new PneumaticsRetractCommand((PneumaticSubsystem) claw));
      oi.getButton(0,Constants.Buttons.A_BUTTON).onTrue(new PneumaticsExtendCommand((PneumaticSubsystem) claw));
    }

    arm.setDefaultCommand(new ArmCommand(arm, oi));
    
  } 


 /**
  * Use this to pass the autonomous command to the main {@link Robot} class.
  *
  * @return the command to run in autonomous
  */
 public Command getAutonomousCommand() {
   // An ExampleCommand will run in autonomous
   return null;
 }
}


