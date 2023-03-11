package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutonCubeExtend extends CommandBase{
    private WristSubsystem m_wrist;
    private ClawSubsystem m_claw;
    private ElevatorSubsystem m_ElevatorSubsystem;
    private ArmPivotSubsystem m_arm;
    private TelescopeSubsystem m_telescope;
    public AutonCubeExtend(WristSubsystem m_wrist, ClawSubsystem m_claw, ElevatorSubsystem m_ele, ArmPivotSubsystem m_arm, TelescopeSubsystem m_telescope){
        this.m_wrist = m_wrist;
        this.m_claw = m_claw;
        this.m_ElevatorSubsystem = m_ele;
        this.m_arm = m_arm;
        this.m_telescope = m_telescope;
        addRequirements(m_wrist, m_claw, m_ElevatorSubsystem, m_arm, m_telescope);
    }
    @Override
    public void initialize(){
        m_claw.close();
        m_wrist.setMotorPos(-2);
        m_ElevatorSubsystem.setMotorPos(20);
        m_arm.setMotorPos(0, true);
        m_telescope.setMotorPos(0);
    }
    @Override
    public void execute(){
        
        m_wrist.setMotorPos(Constants.SetPoints.TCOWrist);
        m_ElevatorSubsystem.setMotorPos(Constants.SetPoints.TCOElevator);
        m_arm.setMotorPos(Constants.SetPoints.TCOArmPivot, true);
        m_telescope.setMotorPos(Constants.SetPoints.TCOTelescope+5);
        if(m_telescope.getEncoderPos()<-20){
            m_claw.open();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // m_claw.open();
        m_wrist.setMotorPos(Constants.SetPoints.MWrist);
        m_ElevatorSubsystem.setMotorPos(Constants.SetPoints.MElevator);
        m_arm.setMotorPos(Constants.SetPoints.MArmPivot, true);
        m_telescope.setMotorPos(Constants.SetPoints.MTelescope);
    }
}
