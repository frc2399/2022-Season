package frc.robot.commands.intakearm;

import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * Extend or retract Intake Arm with pneums
 */

public class ExtendIntakeArm extends CommandBase {
	
    private static Intake m_intake;

	public ExtendIntakeArm(Intake intake) {
		m_intake = intake;
	}

	@Override
	public void initialize() {
		System.out.println("Extend Intake initialized");

	}

	
	@Override
	public void execute() {
		m_intake.extendArm();
	}

	
	@Override
	public boolean isFinished() {
        return true;
	}

	/**
	 * Called once after isFinished returns true
	 */
	@Override
	public void end(boolean interrupted) {
	}
}
