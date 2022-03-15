
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class ShifterSingleSolenoid extends SubsystemBase {

	private Solenoid shifterHotSolenoid;
	private Solenoid shifterDangerousSolenoid;

	public ShifterSingleSolenoid() {
		/**
		 * Assigns Solenoids to correct PCM address and port
		 */
		System.out.println("Shifter subsystem starting");
		shifterHotSolenoid = new Solenoid(DriveConstants.PCM_ADDRESS, PneumaticsModuleType.CTREPCM, DriveConstants.SHIFTER_HOT_SOLENOID_PORT);
		shifterDangerousSolenoid = new Solenoid(DriveConstants.PCM_ADDRESS, PneumaticsModuleType.CTREPCM, DriveConstants.SHIFTER_DANGEROUS_SOLENOID_PORT);
	}

	/**
	 * Method accepts a value of a boolean, the Solenoid object is then set to that value
	 * @param setShifter
	 */
	public void setShifterHotSolenoid(boolean setShifterHot) {
		shifterHotSolenoid.set(setShifterHot);
	}
	
	public void setShifterDangerousSolenoid(boolean setShifterDangerous) {
		shifterDangerousSolenoid.set(setShifterDangerous);
	}

	/**
	 * Gets value of Solenoid for testing purposes
	 * @return
	 */
	public boolean getShifterHotSolenoid() {
		return shifterHotSolenoid.get();
	}
	
	public boolean getShifterDangerousSolenoid() {
		return shifterDangerousSolenoid.get();
	}

	/**
	 * Default state of subsystem is neither hot nor dangerous
	 */
	// public void initDefaultCommand() {
	// 	setDefaultCommand(new Shift(!RobotMap.SHIFTER_SOLENOID_HOT,!RobotMap.SHIFTER_SOLENOID_DANGEROUS));
	// }
} 
