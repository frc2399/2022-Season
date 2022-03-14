

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
        public static final int RIGHT_FRONT_MOTOR_ID = 13;
        public static final int RIGHT_MIDDLE_MOTOR_ID = 14;
        public static final int RIGHT_BACK_MOTOR_ID = 15;
        public static final int LEFT_FRONT_MOTOR_ID = 16;
        public static final int LEFT_MIDDLE_MOTOR_ID = 7;
        public static final int LEFT_BACK_MOTOR_ID = 8;

        //solenoids
        public static final int SHIFTER_HOT_SOLENOID_PORT = 0;
	    public static final int SHIFTER_DANGEROUS_SOLENOID_PORT= 1;
        public static final int PCM_ADDRESS = 13;
        public static final boolean SHIFTER_SOLENOID_DANGEROUS= true;
	    public static final boolean SHIFTER_SOLENOID_HOT= true;
        public static final double SHIFT_TIMER = 0.5;

        //follow vision target cmd speeds
        public static final double TARGET_FOLLOWING_SPEED = 0.5;
        public static final double BUTTERY_FOLLOWING_SPEED = 0.035;
    }

    public static final class JoystickConstants {
        // joystick ports
        public static final int JOYSTICK_PORT = 1;

        public static final int FORWARD_JOYSTICK_INVERT = 1;
        public static final int TURN_JOYSTICK_INVERT = 1;

        public static final int SHIFT_HIGH_SPEED = 5; 
        public static final int SHIFT_HIGH_GEAR = 6; 

        public static final int INTAKE = 2; 
        public static final int INTAKE_ARM_RETRACT = 11; 
        public static final int INTAKE_ARM_EXTEND = 12; 

        public static final int SHOOTER_BTN = 3; 

        public static final int INDEXER_FWD = 7;
        public static final int INDEXER_BACK = 8;

        public static final int TURN_TO_N = 99; // TODO need to make buttons for turn to 90 left, 90 right, 180

        public static final int CLIMBER_UP = 9;
        public static final int CLIMBER_DOWN = 10;

    }  

    public static final class XboxConstants {
        public static final int XBOX_PORT = 0;

        public static final int ARCADE_DRIVE_SPEED_AXIS = 1;
        public static final int ARCADE_DRIVE_TURN_AXIS = 4; 

        public static final double FORWARD_DEADBAND = 0.05;
        public static final double TURN_DEADBAND = 0.05;

        public static final double JOYSTICK_SENSITIVITY = 0.5;

        public static final int TURN_RIGHT = 2;
        public static final int TURN_LEFT = 3;
        public static final int TURN_180 = 1;

    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 21;
        public static final double INTAKESPEED = 0.5;
        public static final double INTAKE_SLEW = 5;

        //intake arm
        public static final int EXTEND_INTAKE_ARM = 2;
        public static final int RETRACT_INTAKE_ARM = 3;
    }

    public static final class ShooterConstants {
        public static final int PID_IDX = 0;
        public static final int CAN_TIMEOUT = 10;
        public static final int ENCODER_TICKS_PER_REVOLUTION = 4096;
        public static final double GEAR_RATIO = 84.0 / 54.0;
        public static final double TALON_100MS_IN_1S = 10.0;
        public static final int TOP_MOTOR_ID = 10; 
        public static final int BOTTOM_MOTOR_ID = 9;
        public static final double SHOOTER_SPEED_INCREMENT = 0;

        public static final double SHOOTER_SPEED = 0.06;

        public static final double TOP_SETPOINT = 700;
        public static final double BOTTOM_SETPOINT = 800;
    }

    public static final class IndexerConstants {
        public static final int INDEXER_MOTOR_ID = 2;
        public static final int INDEXERSPEED = 1;
        public static final int LIMIT_SWITCH_ID = 0;
    }

    // public static final class PrototypeDriveConstants {
    //     public static final int LEFT_FRONT_MOTOR_ID = 8;
    //     public static final int LEFT_MIDDLE_MOTOR_ID = 7;
    //     public static final int LEFT_BACK_MOTOR_ID = 6;

    // }
  
    public static final class ClimberConstants {
        public static final int LEFT_CLIMBER_MOTOR_ID = 12;
        public static final int RIGHT_CLIMBER_MOTOR_ID = 11;
        public static final double CLIMBER_SPEED = 0.5;
        public static final double MAX_HEIGHT = 0.9;
        public static final double MIN_HEIGHT = 0;

    }

    public static final class LimelightConstants {
        public static final int HEIGHT_INCHES = 18;
    }

    public static final class PhotonLimelightConstants {
        public static final double CAMERA_HEIGHT_FEET = 45/12;
        public static final double TARGET_HEIGHT_FEET = 91/12;
        public static final double HUB_RADIUS_FEET = 2.25;
        public static final double TILT_DEGREES = -5.3;
        // public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
        // public static final double GOAL_RANGE_METERS = Units.feetToMeters(3);
    }
}




