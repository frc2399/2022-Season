package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

public class IndexerDefaultCmd extends CommandBase {

    private final Indexer indexerSubsystem;

    public IndexerDefaultCmd(Indexer indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("IndexerDefaultCmd started!");
    }

    @Override
    public void execute() {
        // double speed = Indexer.indexSpeedEntry.getDouble(0.3);
        // this.indexerSubsystem.setSpeed(speed);
        this.indexerSubsystem.setSpeed(0);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("IndexerDefaultCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
