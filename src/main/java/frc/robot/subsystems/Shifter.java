
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Shifter extends SubsystemBase {

	private DoubleSolenoid shifter;

	public Shifter() {
		/**
		 * Assigns Solenoids to correct PCM address and port
		 */
		System.out.println("Shifter subsystem starting");	
		shifter = new DoubleSolenoid(DriveConstants.PCM_ADDRESS, PneumaticsModuleType.CTREPCM, DriveConstants.SHIFT_HIGH_SPEED_SOLENOID_PORT, 
																							   DriveConstants.SHIFT_HIGH_TORQUE_PORT);
	}

	/**
	 * Method accepts a value of a boolean, the Solenoid object is then set to that value
	 * @param setShifter
	 */
	public void setShifterHot() {
		shifter.set(Value.kForward);

	}
	
	public void setShifterDangerous() {
		shifter.set(Value.kReverse);
	}

	/**
	 * Gets value of Solenoid for testing purposes
	 * @return
	 */
	// public boolean getShifterHotSolenoid() {
	// 	return shifter.get();
	// }
	
	// public boolean getShifterDangerousSolenoid() {
	// 	return shifterDangerousSolenoid.get();
	// }

	/**
	 * Default state of subsystem is neither hot nor dangerous
	 */
	// public void initDefaultCommand() {
	// 	setDefaultCommand(new Shift(!RobotMap.SHIFTER_SOLENOID_HOT,!RobotMap.SHIFTER_SOLENOID_DANGEROUS));
	// }
} 
