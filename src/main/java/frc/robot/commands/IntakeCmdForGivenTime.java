package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

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
            //System.out.println("intake time end");
            return true;
        }
        return false;
    }
}