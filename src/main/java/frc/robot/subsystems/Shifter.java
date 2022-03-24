
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Shifter extends SubsystemBase {

	public static NetworkTableEntry highSpeedEntry, highTorqueEntry;

	private DoubleSolenoid shifter;

	public Shifter() {
		/**
		 * Assigns Solenoids to correct PCM address and port
		 */
		System.out.println("Shifter subsystem starting");	
		shifter = new DoubleSolenoid(DriveConstants.PCM_ADDRESS, PneumaticsModuleType.CTREPCM, DriveConstants.SHIFT_HIGH_SPEED_SOLENOID_PORT, 
																							   DriveConstants.SHIFT_HIGH_TORQUE_PORT);
		ShuffleboardLayout gearsLayout = Shuffleboard.getTab("Driver")
			.getLayout("Gears", BuiltInLayouts.kList)
			.withSize(1, 2)
			.withPosition(1, 1)
		;

		highSpeedEntry = gearsLayout.add("High Speed", false).getEntry();
    	highTorqueEntry = gearsLayout.add("High Torque", false).getEntry();  

	}

	/**
	 * Method accepts a value of a boolean, the Solenoid object is then set to that value
	 * @param setShifter
	 */
	public void setShifterHighSpeed() {
		shifter.set(Value.kForward);
		highSpeedEntry.setBoolean(true);
		highTorqueEntry.setBoolean(false);

		DriveTrain.leftEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_SPEED_REVOLUTION_TO_INCH_CONVERSION);
        DriveTrain.rightEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_SPEED_REVOLUTION_TO_INCH_CONVERSION);
	}
	
	public void setShifterHighTorque() {
		shifter.set(Value.kReverse);
		highTorqueEntry.setBoolean(true);
		highSpeedEntry.setBoolean(false);

		DriveTrain.leftEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_TORQUE_REVOLUTION_TO_INCH_CONVERSION);
        DriveTrain.rightEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_TORQUE_REVOLUTION_TO_INCH_CONVERSION);
	}

	/**
	 * Gets value of Solenoid for testing purposes
	 * @return
	 */
	public boolean isHighSpeed() {
		return shifter.get() == Value.kForward;
	}


	/**
	 * Default state of subsystem is neither hot nor dangerous
	 */
	// public void initDefaultCommand() {
	// 	setDefaultCommand(new Shift(!RobotMap.SHIFTER_SOLENOID_HOT,!RobotMap.SHIFTER_SOLENOID_DANGEROUS));
	// 
} 
