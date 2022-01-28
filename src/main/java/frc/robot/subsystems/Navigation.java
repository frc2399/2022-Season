package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;



public class Navigation extends SubsystemBase {
    AHRS ahrs;

	public Navigation() {
        ahrs = new AHRS(SPI.Port.kMXP);

    }
	
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        System.out.println ("Yaw angle:" + ahrs.getAngle());

    }


	/**
	 * Default state of subsystem is neither hot nor dangerous
	 */
	// public void initDefaultCommand() {
	// 	setDefaultCommand(new Shift(!RobotMap.SHIFTER_SOLENOID_HOT,!RobotMap.SHIFTER_SOLENOID_DANGEROUS));
	// }
} 
  

