// RobotBuilder Version: 4.0
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;
//import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {

private CANSparkMax intakeMotorController;


SlewRateLimiter filter;


    public Intake() {

        //Define all Motor Controllers
        intakeMotorController = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushed);
        SmartDashboard.putNumber("Intake Slew Rate", SmartDashboard.getNumber ("Intake Slew Rate", IntakeConstants.INTAKE_SLEW));
        filter = new SlewRateLimiter(SmartDashboard.getNumber("Intake Slew Rate", IntakeConstants.INTAKE_SLEW));
        System.out.println ("SlewRateLimiter " + SmartDashboard.getNumber("Intake Slew Rate", IntakeConstants.INTAKE_SLEW));

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setMotor(double intakeSpeed) {
        intakeSpeed = filter.calculate(intakeSpeed);
        intakeMotorController.set(intakeSpeed);
        SmartDashboard.putNumber("Intake Speed", intakeSpeed);
        }

}

