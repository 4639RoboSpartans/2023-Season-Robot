// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class IDs{
		public static final int ARM_PIVOT1_ID = 6;
		public static final int ARM_PIVOT2_ID = 6;

		public static final int CLAW_SOLENOID_CHANNEL = 3;	

		public static class SwerveModuleConfig {
			public final int driveMotorID, rotaterMotorID, encoderID;
			public final double rotationOffset;

			public SwerveModuleConfig(int driveMotorID, int rotaterMotorID, int encoderID, double rotationOffset) {
				this.driveMotorID = driveMotorID;
				this.rotaterMotorID = rotaterMotorID;
				this.encoderID = encoderID;
				this.rotationOffset = rotationOffset;
			}
		}

		public static final SwerveModuleConfig MODULE_FRONT_LEFT  = new SwerveModuleConfig(1, 2, 9 , 124.277);
		public static final SwerveModuleConfig MODULE_FRONT_RIGHT = new SwerveModuleConfig(3, 4, 10, 233.877);
		public static final SwerveModuleConfig MODULE_BACK_LEFT   = new SwerveModuleConfig(5, 6, 11, 9.668);
		public static final SwerveModuleConfig MODULE_BACK_RIGHT  = new SwerveModuleConfig(7, 8, 12, 50.400);

	}

	public static final class RobotInfo {
		public static final double trackwidth = 0.44;
		public static final double wheelbase =0.44;
		public static final double SWERVE_KI = 0.1;
		public static final double SWERVE_KP = 0.1;

		public static final double MOVEMENT_SPEED = 0.2; // 0 - 1
	}

	public static final class Timing {
		public static final double CLAW_DELAY_AFTER_OPEN = 0.5;
	}
	
    public static final double DEADZONE_VALUE = 0.01;
	public static final int NUMBER_OF_CONTROLLERS = 2;

    public enum Axes {
		LEFT_STICK_X(0), LEFT_STICK_Y(1), 
		LEFT_TRIGGER(2), RIGHT_TRIGGER(3), 
		RIGHT_STICK_X(4), RIGHT_STICK_Y(5);

		private final int value;

		Axes(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}
	}

	public enum Buttons {
		A_BUTTON(1), B_BUTTON(2), X_BUTTON(3), Y_BUTTON(4), LEFT_BUMPER(5), RIGHT_BUMPER(6), BACK_BUTTON(
				7), START_BUTTON(8), LEFT_STICK(9), RIGHT_STICK(10);

		private final int value;

		private Buttons(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}

	}   
}
