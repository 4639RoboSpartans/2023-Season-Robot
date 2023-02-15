package frc.robot.subsystems;

public class ArmAndWristSubsystem {
    private final ArmPivotSubsystem arm;
    private final WristSubsystem wrist;

    public ArmAndWristSubsystem(){
        arm = new ArmPivotSubsystem();
        wrist = new WristSubsystem();
    }

    public void stop(){
        arm.stop();
        wrist.stop();
    }
}
