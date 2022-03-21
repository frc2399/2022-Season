package frc.robot.commands.drivetrain;

// import edu.wpi.first.wpilibj.interfaces.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;

public class DriveStraightGivenDistance extends CommandBase {

    //insantiate global variables
    double speed;
    double currentPosition;
    double targetDistance;
    double newTargetDistance;
    DriveTrain m_driveTrain;
    double startAngle;
    double currentAngle;
    
	public DriveStraightGivenDistance(double speed, double targetDistance, DriveTrain subsystem) {
        
        //initialize variables
        this.speed = speed;
        this.targetDistance = targetDistance;
        m_driveTrain = subsystem;
        addRequirements(m_driveTrain);

        //set command to be interruptible
		//setInterruptible(true);
    }
    

	// Called just before this Command runs the first time
    @Override
    public void initialize() {
        // Sets the current position to where robot is starting
        currentPosition = (m_driveTrain.getLeftEncoderPosition() + m_driveTrain.getRightEncoderPosition()) / 2;
        System.out.println("starting current position " + currentPosition);
        
        // find distance robot needs to travel to from its current position
        newTargetDistance = currentPosition + targetDistance;

        startAngle = m_driveTrain.getAngle();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

        // Get the average position between leftEncoder and rightEncoder
        currentPosition = (m_driveTrain.getLeftEncoderPosition() + m_driveTrain.getRightEncoderPosition()) / 2;
        currentAngle = m_driveTrain.getAngle();

        double distanceError = newTargetDistance - currentPosition;
        double angleError = currentAngle - startAngle;

        double outputSpeed = m_driveTrain.kP * distanceError;
        outputSpeed = MathUtil.clamp(outputSpeed, -0.5, 0.5);

        double straightCorrection = DriveTrain.angleErrorPValue.getDouble(0.01) * angleError;
        straightCorrection = MathUtil.clamp(straightCorrection, -0.5 * Math.abs(outputSpeed), 0.5 * Math.abs(outputSpeed));

        m_driveTrain.setMotors(outputSpeed - straightCorrection, outputSpeed + straightCorrection);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        double butteryErrorTolerance = DriveTrain.angleErrorTolerance.getDouble(0.5);
        // SmartDashboard.getNumber("Error Tolerance Distance", 0.5);
        // SmartDashboard.putNumber("distance bt td and cp", Math.abs(targetDistance - currentPosition));
        // System.out.println("distance bt td and cp " +  Math.abs(td - cp));

        if (Math.abs(newTargetDistance - currentPosition) <= butteryErrorTolerance)
        {
            return true;
        }
        return false;
        
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    
        m_driveTrain.setMotors(0, 0);

        System.out.println("DriveForwardGivenDistance ended");
    }
}
