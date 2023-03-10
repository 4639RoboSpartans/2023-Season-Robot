// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.fasterxml.jackson.databind.PropertyNamingStrategies.SnakeCaseStrategy;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonCommand;

  private RobotContainer robotContainer;
  
  
  // public DigitalInput IR = new DigitalInput(5);
  ///private AprilTagReader aprilTagReader;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   private AddressableLED m_led1;
   private AddressableLEDBuffer m_ledBuffer1;
   private int starterLED;
  //  private AddressableLEDBuffer m_ledBuffer2;
  @Override
  public void robotInit() {
    // m_colorSensor.configureProximitySensor(10 , 10);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
     m_led1 = new AddressableLED(1);
    m_ledBuffer1 = new AddressableLEDBuffer(60);
    m_led1.setLength(m_ledBuffer1.getLength());
    m_led1.setData(m_ledBuffer1);
    m_led1.start();
    starterLED = 0;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("DIstance", robotContainer.clawObstructionSensor.getRaw());
    SmartDashboard.putNumber("WristEncoderValue", robotContainer.wristSubsystem.getEncoderPos());
    SmartDashboard.putNumber("ArmEncoderValue", robotContainer.armPivotSubsystem.getEncoderPos());
    SmartDashboard.putNumber("TelescopeEncoderValue", robotContainer.telescopeSubsystem.getEncoderPos());
    // SmartDashboard.putNumber("ElevatorEncoderValue", robotContainer.elevatorSubsystem.getEncoderPos());
    // SmartDashboard.putNumber("WristCPR", robotContainer.wristSubsystem.getCPR());
    SmartDashboard.putNumber("ArmVoltage", robotContainer.armPivotSubsystem.getVoltage());
    // SmartDashboard.putNumber("WristRawEncoder", robotContainer.wristSubsystem.getRawEncoderPos());
    // SmartDashboard.putNumber("TelescopeRawEncoder", robotContainer.telescopeSubsystem.getRawEncoderPos());
    // SmartDashboard.putNumber("ElevatorRawEncoder", robotContainer.elevatorSubsystem.getRawEncoderPos());
    SmartDashboard.putNumber("ArmPivotRawEncoder", robotContainer.armPivotSubsystem.getRawEncoderPos());

    SmartDashboard.putBoolean("ArmClosing", Constants.objectIn);

  //   if(robotContainer.swerveDriveSubsystem.AprilTagDetected()){
  //   SmartDashboard.putNumber("3dposX", robotContainer.swerveDriveSubsystem.FieldD3Coords()[0]);
  //   SmartDashboard.putNumber("3dposY", robotContainer.swerveDriveSubsystem.FieldD3Coords()[1]); 
  //   SmartDashboard.putNumber("3dposYaw", robotContainer.swerveDriveSubsystem.FieldD3Coords()[2]);

  // }
  
    // SmartDashboard.putBoolean("Controller y activated", robotContainer.oi.getButton(1, Constants.Buttons.Y_BUTTON).getAsBoolean());
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    // SmartDashboard.putBoolean("Table set", table);
    // table.getEntry("pipeline").setNumber(1);
    // SmartDashboard.putNumber("XOFfset", robotContainer.swerveDriveSubsystem.getAprilXOffset());

    SmartDashboard.putNumber("DistanceSensorRaw", robotContainer.clawObstructionSensor.getRaw());
    SmartDashboard.putBoolean("DistanceSensorObstruction", robotContainer.clawObstructionSensor.isObstructed());
    if(Constants.LEDInfo.LEDStatus!=0){
    for (var i = 0; i < m_ledBuffer1.getLength(); i++) {
      if(Constants.LEDInfo.LEDStatus==1){
        m_ledBuffer1.setRGB(i, Constants.LEDInfo.GreenR, Constants.LEDInfo.GreenG,Constants.LEDInfo.GreenB);
      }
      else if(Constants.LEDInfo.LEDStatus==2){
        m_ledBuffer1.setRGB(i, Constants.LEDInfo.PurpleR, Constants.LEDInfo.PurpleG,Constants.LEDInfo.PurpleB);
      }
      else if(Constants.LEDInfo.LEDStatus==3){
        m_ledBuffer1.setRGB(i, Constants.LEDInfo.YellowR, Constants.LEDInfo.YellowG,Constants.LEDInfo.YellowB);
      }
      else if(Constants.LEDInfo.LEDStatus==4){
        m_ledBuffer1.setRGB(i, Constants.LEDInfo.RedR, Constants.LEDInfo.RedG,Constants.LEDInfo.RedB);
      }
    }
  }else{
    boolean blue = false;
    for(var i =0;i<m_ledBuffer1.getLength();i++){
      if(i%7==0){
        blue = !blue;
      }if(blue){
        m_ledBuffer1.setRGB((starterLED+i)%60, Constants.LEDInfo.BlueR, Constants.LEDInfo.BlueG,Constants.LEDInfo.BlueB);
      }else{
        m_ledBuffer1.setRGB((starterLED+i)%60, Constants.LEDInfo.OrangeR, Constants.LEDInfo.OrangeG,Constants.LEDInfo.OrangeB);
      }
    }
    starterLED++;
  }
   m_led1.setData(m_ledBuffer1);
    // SmartDashboard.putNumber("X LL Distance", robotContainer.swerveDriveSubsystem.getXoffset());
    // SmartDashboard.putNumber("X April Offset", robotContainer.swerveDriveSubsystem.getAprilXOffset());
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    Constants.LEDInfo.LEDStatus = 0;
    ElevatorSubsystem.motors.forEach(motor -> motor.setNeutralMode(NeutralMode.Coast));
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonCommand = robotContainer.getAutonomousCommand();
    if (autonCommand != null) {
      autonCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Constants.LEDInfo.LEDStatus = 4;
    ElevatorSubsystem.motors.forEach(motor -> motor.setNeutralMode(NeutralMode.Brake));
    robotContainer.wristSubsystem.resetEncoder();
    // m_robotContainer.m_shroud.resetEncoder();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonCommand != null) {
      autonCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
      // robotContainer.retract.initialize();
      // robotContainer.enableCompressor();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
