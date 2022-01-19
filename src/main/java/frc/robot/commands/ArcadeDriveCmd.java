package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDriveCmd extends CommandBase {

    private final DriveTrain driveSubsystem;
    private final Supplier<Double> speedFunction, turnFunction;

    public ArcadeDriveCmd(DriveTrain driveSubsystem, //
            Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ArcadeDriveCmd started!");
    }

    @Override
    public void execute() {
        double realTimeSpeed = speedFunction.get();
        double realTimeTurn = turnFunction.get();

        double left = realTimeSpeed + realTimeTurn;
        double right = realTimeSpeed - realTimeTurn;
        this.driveSubsystem.setMotors(left, right);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ArcadeDriveCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
