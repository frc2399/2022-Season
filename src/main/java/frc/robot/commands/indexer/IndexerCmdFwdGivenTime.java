package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class IndexerCmdFwdGivenTime extends CommandBase {

    private final Indexer IndexerSubsystem;
    private final double speed;
    Timer timer;
    double tm;

    public IndexerCmdFwdGivenTime(Indexer IndexerSubsystem, double speed, double time) {
        this.IndexerSubsystem = IndexerSubsystem;
        this.speed = speed;
        tm = time;
        timer = new Timer();
        addRequirements(IndexerSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Indexer time started!");
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        this.IndexerSubsystem.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Indexer time ended!");
        this.IndexerSubsystem.setSpeed(0);
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
