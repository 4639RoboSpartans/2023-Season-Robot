package frc.robot.swerve.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.HashMap;
import java.util.Map;

public class AutonPath {
    private final PathPlannerTrajectory traj;

    private static final Map<String, Command> eventMap = new HashMap<>();

    public AutonPath(String file){
        traj = PathPlanner.loadPath(file, new PathConstraints(
            Constants.RobotInfo.MAX_VELOCITY,
            Constants.RobotInfo.MAX_ACCELERATION
        ));
    }

    public Command getCommand(SwerveDriveSubsystem swerve) {
        SwerveAutoBuilder builder = new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetPose,
            swerve.getKinematics(),
            new PIDConstants(
                Constants.RobotInfo.Auton.POSITION_KP,
                Constants.RobotInfo.Auton.POSITION_KI,
                0.0
            ),
            new PIDConstants(
                Constants.RobotInfo.Auton.ROTATION_KP,
                0.0,
                0.0
            ),
            (SwerveModuleState... states) -> swerve.setModules(states[0], states[1], states[2], states[3]),
            eventMap,
            true,
            swerve
        );
        // TODO
        // Link to some prob helpful docs: https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage
        // Search up stuff ig

        return null; // Replace with something meaningful
    }

    public static void registerEvents (Event... events){
        for(Event event : events) registerEvent(event);
    }

    public static void registerEvent(Event event) {
        registerEvent(event.eventName, event.command);
    }

    public static void registerEvent(String eventName, Command command){
        eventMap.put(eventName, command);
    }

    public record Event(String eventName, Command command){}
}
