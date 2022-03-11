
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Shifter extends SubsystemBase {

	private Solenoid shifterHighSpeedSolenoid;
	private Solenoid shifterHighGearSolenoid;

	public Shifter() {
		/**
		 * Assigns Solenoids to correct PCM address and port
		 */
		shifterHighSpeedSolenoid = new Solenoid(DriveConstants.PCM_ADDRESS, PneumaticsModuleType.CTREPCM, DriveConstants.SHIFT_HIGH_SPEED_SOLENOID_PORT);
		shifterHighGearSolenoid = new Solenoid(DriveConstants.PCM_ADDRESS, PneumaticsModuleType.CTREPCM, DriveConstants.SHIFT_HIGH_GEAR_PORT);
	}

	/**
	 * Method accepts a value of a boolean, the Solenoid object is then set to that value
	 * @param setShifter
	 */
	public void setShifterHotSolenoid(boolean setShifterHot) {
		shifterHighSpeedSolenoid.set(setShifterHot);
	}
	
	public void setShifterDangerousSolenoid(boolean setShifterDangerous) {
		shifterHighGearSolenoid.set(setShifterDangerous);
	}

	/**
	 * Gets value of Solenoid for testing purposes
	 * @return
	 */
	public boolean getShifterHotSolenoid() {
		return shifterHighSpeedSolenoid.get();
	}
	
	public boolean getShifterDangerousSolenoid() {
		return shifterHighGearSolenoid.get();
	}

	/**
	 * Default state of subsystem is neither hot nor dangerous
	 */
	// public void initDefaultCommand() {
	// 	setDefaultCommand(new Shift(!RobotMap.SHIFTER_SOLENOID_HOT,!RobotMap.SHIFTER_SOLENOID_DANGEROUS));
	// }
} 
