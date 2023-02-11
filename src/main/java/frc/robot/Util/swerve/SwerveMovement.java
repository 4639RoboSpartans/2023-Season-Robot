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
     * @return The movement represented by this swerve movement. +X is forward
     */
    public vec2 movement(){
        return new vec2(forwardMovement(), strideMovement());
    }
}