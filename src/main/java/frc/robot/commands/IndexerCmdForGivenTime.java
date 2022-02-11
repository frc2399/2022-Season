package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IndexerCmdForGivenTime extends CommandBase {

    private final Indexer indexerSubsystem;
    private final double speed;
    private static Timer timer;
    double time;

    public IndexerCmdForGivenTime(Indexer indexerSubsystem, double speed, double time) {
        this.indexerSubsystem = indexerSubsystem;
        this.speed = speed;
        this.time = time;
        timer = new Timer();
        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("IndexerCmdForGivenTime started!");
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        this.indexerSubsystem.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("IndexerCmdForGivenTime ended!");
        this.indexerSubsystem.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= time)
        {
            return true;
        }
        return false;
    }
}
