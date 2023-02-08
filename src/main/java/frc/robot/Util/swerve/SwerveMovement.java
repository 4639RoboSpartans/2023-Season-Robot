package frc.robot.util.swerve;

import frc.robot.util.math.math;
import frc.robot.util.math.vec2;

/**
 * A struct containing a desired translation and a rotation of the swerve drive.
 * Positive strideMovement is to the left of the robot
 */
public record SwerveMovement(double forwardMovement, double strideMovement, double rotation){
    public SwerveMovement(vec2 movement, double rotation){
        this(movement.x(), movement.y(), rotation);
    }

    /**
     * Converts a field-centric {@link SwerveMovement} to a robot-centric.
     * That is, this method converts a swerve movement measured relative to the field
     * to a swerve movement relative to the robot
     * @param robotHeadingDegrees The direction that the robot is pointing, measured counterclockwise in degrees
     * @return
     */
    public SwerveMovement toRobotCentric(double robotHeadingDegrees) {
        vec2 rawMovement = getMovementVector();
        double rawRotation = rotation();

        vec2 movement = math.rotateCW(rawMovement, robotHeadingDegrees);

        return new SwerveMovement(movement, rawRotation);
    }

    /**
     * @return The movement represented by this swerve movement. +X is forward
     */
    public vec2 getMovementVector(){
        return new vec2(forwardMovement(), strideMovement());
    }
}