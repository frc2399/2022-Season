package frc.robot.commands;

import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * Extend or retract Intake Arm with pneums
 */

public class RetractIntakeArm extends CommandBase {
	
    private static Intake m_intake;

	public RetractIntakeArm(Intake intake) {
		m_intake = intake;
	}

	@Override
	public void initialize() {

	}

	
	@Override
	public void execute() {
		m_intake.retractArm();
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
