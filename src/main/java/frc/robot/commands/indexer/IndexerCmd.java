package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class IndexerCmd extends CommandBase {

    private final Indexer indexerSubsystem;
    private final double speed;

    public IndexerCmd(Indexer indexerSubsystem, double speed) {
        this.indexerSubsystem = indexerSubsystem;
        this.speed = speed;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("IndexerCmd started!");
    }

    @Override
    public void execute() {
        double speed = Indexer.indexSpeed.getDouble(0.3);
        this.indexerSubsystem.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("IndexerCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
