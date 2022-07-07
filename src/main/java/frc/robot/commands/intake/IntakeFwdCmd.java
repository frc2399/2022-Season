package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

//runs the intake at a speed given in the smart dashboard, never used
//note: this command is exactly the same as the IntakeBackCmd

public class IntakeFwdCmd extends CommandBase {

    private final Intake intakeSubsystem;

    public IntakeFwdCmd(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Intake fwd cmd started!");
    }

    @Override
    public void execute() {
        double speedFromDashboard = Intake.intakeSpeedEntry.getDouble(0.5);
        // this.intakeSubsystem.setMotor(speed);
        this.intakeSubsystem.setMotor(speedFromDashboard);
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
