package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Shifter;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Shift to dangerous (high) speed or hot (low) speed with pneums
 */

public class Shift extends CommandBase {
	
	private Shifter shifter = RobotContainer.m_shifter;
	private boolean shiftHotValue;
	private boolean shiftDangerousValue;

	/**
	 * @param shiftHotValue: Should be set to either the RobotMap value (on) or !RobotMap value (off)
	 * @param shiftDangerousValue: Should be set to either the RobotMap value (on) or !RobotMap value (off)
	 */
	public Shift(boolean shiftHotValue, boolean shiftDangerousValue) {
		System.out.println("Hello" + shifter);
		this.shiftHotValue = shiftHotValue;
		this.shiftDangerousValue = shiftDangerousValue;
        addRequirements(shifter);
		//setInterruptible(true);
	}

	/**
	 * Sets how long the timer should run for
	 */
	@Override
	public void initialize() {
		//withTimeout(DriveConstants.SHIFT_TIMER);
		SmartDashboard.putBoolean("   ", shiftHotValue);
		System.out.println("Shift initialized");
		SmartDashboard.putBoolean("    ", shiftDangerousValue);
	}

	/**
	 *  Sets the Solenoids to values passed to the constructor
	 *  Called repeatedly when this Command is scheduled to run
	 */
	@Override
	public void execute() {
		shifter.setShifterHotSolenoid(shiftHotValue);
		System.out.println("Shift executed");
		shifter.setShifterDangerousSolenoid(shiftDangerousValue);
	}

	/**
	 * If the command returns true (the timer has run out), stop executing the command
	 * Make this return true when this Command no longer needs to run execute()
	 */
	@Override
	public boolean isFinished() {
		//return isTimedOut();
        return true;
	}

	/**
	 * Called once after isFinished returns true
	 */
	@Override
	public void end(boolean interrupted) {
		System.out.println("Shift ended");
	}
}
