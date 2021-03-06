package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

//runs the intake at a given speed for a given amount of time

public class IntakeCmdForGivenTime extends CommandBase {

    private final Intake intakeSubsystem;
    private final double speed;
    Timer timer;
    double tm;

    public IntakeCmdForGivenTime(Intake intakeSubsystem, double speed, double time) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        tm = time;
        timer = new Timer();
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("intake time started!");
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        this.intakeSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Intake time ended!");
        this.intakeSubsystem.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= tm)
        {
            return true;
        }
        return false;
    }
}
