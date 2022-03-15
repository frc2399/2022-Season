package frc.robot.commands.drivetrain;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Shifter;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Shift to dangerous (high) speed or hot (low) speed with pneums
 */

public class Shift extends CommandBase {
	
	private Shifter shifter = RobotContainer.m_shifter;
	private boolean shiftHighSpeedValue;
	private boolean shiftHighGearValue;

	/**
	 * @param shiftHighSpeedValue: Should be set to either the RobotMap value (on) or !RobotMap value (off)
	 * @param shiftHighGearValue: Should be set to either the RobotMap value (on) or !RobotMap value (off)
	 */
	public Shift(boolean shiftHighSpeed, boolean shiftHighGear) {
		System.out.println("Hello" + shifter);
		shiftHighSpeed = shiftHighSpeedValue;
		shiftHighGear = shiftHighGearValue;
        addRequirements(shifter);
		//setInterruptible(true);
	}

	/**
	 * Sets how long the timer should run for
	 */
	@Override
	public void initialize() {
		//withTimeout(DriveConstants.SHIFT_TIMER);
		// SmartDashboard.putBoolean("   ", shiftHighSpeedValue);
		System.out.println("Shift initialized");
		// SmartDashboard.putBoolean("    ", shiftHighGearValue);
	}

	/**
	 *  Sets the Solenoids to values passed to the constructor
	 *  Called repeatedly when this Command is scheduled to run
	 */
	@Override
	public void execute() {
		// shifter.setShifterHotSolenoid(shiftHotValue);
		shifter.setShifterHot();
		System.out.println("Shift executed");
		// shifter.setShifterDangerousSolenoid(shiftDangerousValue);
		// shifter.setShifterDangerous();
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
