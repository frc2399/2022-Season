
package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Shooter;

//import frc.robot.Robot;

public class ShootForNSeconds extends CommandBase {

    private final Shooter shooterSubsystem;
    private final double topSpeed;
    private final double bottomSpeed;

    //private double m_time;
    
    Timer timer;
    double tm;


    public ShootForNSeconds (Shooter shooterSubsystem, double topSpeed, double bottomSpeed, double time) {

        //m_time = time;

        //initialize variables
    

        tm = time;

        timer = new Timer();

        this.shooterSubsystem = shooterSubsystem;
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
        addRequirements(shooterSubsystem);
        
        //set command to be interruptible
		//setInterruptible(true);

    }

    // Called when the command is initially scheduled. (when it runs the first time)
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        System.out.println("ShootForNSeconds Initialized");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.shooterSubsystem.setMotors(topSpeed, bottomSpeed);
        //System.out.println("ForwardDrive Executed");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("ShootForNSeconds Ended");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //System.out.println("ForwardDrive Finished");
        return timer.get() >= tm;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}
