package frc.robot.commands.drivetrain;

// import edu.wpi.first.wpilibj.interfaces.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Timer;


public class DriveForwardGivenTime extends CommandBase {

    //insantiate global variables
	double turnPercent, forwardPercent;
    Timer timer;
    double sp;
    double tm;
    DriveTrain m_driveTrain;
    
 
	public DriveForwardGivenTime(double speed, double time, DriveTrain subsystem) {
        
        //initialize variables
        sp = speed;
        tm = time;
        m_driveTrain = subsystem;
        addRequirements(m_driveTrain);
        //set command to be interruptible
		//setInterruptible(true);

        timer = new Timer();
    }
    

	// Called just before this Command runs the first time
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_driveTrain.setMotors(sp, sp);
       
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return timer.get() >= tm; 
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }
}
