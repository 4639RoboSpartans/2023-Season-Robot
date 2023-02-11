package frc.robot.Util.swerve;

import frc.robot.Util.math.math;
import frc.robot.Util.math.vec2;

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