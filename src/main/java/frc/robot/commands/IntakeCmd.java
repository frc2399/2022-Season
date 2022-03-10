package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends CommandBase {

    private final Intake intakeSubsystem;
    private final double speed;

    public IntakeCmd(Intake intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Intake cmd started!");
    }

    @Override
    public void execute() {
        this.intakeSubsystem.setMotor(speed);
        //SmartDashboard.putNumber("Intake speed ", speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Intake cmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
