// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    public static final class DriveConstants {
        // motor ids
        public static final int LEFT_FRONT_MOTOR_ID = 8;
        public static final int RIGHT_FRONT_MOTOR_ID = 2;
        public static final int LEFT_MIDDLE_MOTOR_ID = 7;
        public static final int RIGHT_MIDDLE_MOTOR_ID = 3;
        public static final int BACK_LEFT_MOTOR_ID = 6;
        public static final int BACK_RIGHT_MOTOR_ID = 4;

        //solenoids
        public static final int SHIFTER_HOT_SOLENOID_PORT = 2;
	    public static final int SHIFTER_DANGEROUS_SOLENOID_PORT= 3;
        public static final int PCM_ADDRESS = 3;
        public static final boolean SHIFTER_SOLENOID_DANGEROUS= true;
	    public static final boolean SHIFTER_SOLENOID_HOT= true;
        public static final double SHIFT_TIMER = 0.5;
    }

    public static final class JoystickConstants {
        // joystick ports
        public static final int JOYSTICK_PORT = 1;
        public static final int XBOX_PORT = 0;

        public static final int LEFT_DRIVETRAIN = 1;
        public static final int RIGHT_DRIVETRAIN = -1;

        public static final int kArcadeDriveSpeedAxis = 1;
        public static final int kArcadeDriveTurnAxis = 4; 

        public static final int kShiftHot = 5; 
        public static final int kShiftDangerous = 6; 

        public static final int INTAKE = 2; 

        public static final int INDEXER = 3;

    }  

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 1;
        public static final int INTAKESPEED = 1;
    }

    public static final class IndexerConstants {
        public static final int INDEXER_MOTOR_ID = 5;
        public static final int INDEXERSPEED = 1;
    }

}

