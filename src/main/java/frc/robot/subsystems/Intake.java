package frc.robot.subsystems;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
//import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


public class Intake extends SubsystemBase {

    public static final NetworkTableEntry slewRate = Shuffleboard.getTab("Params").add("Slew Rate Limiter", 0).getEntry();

private CANSparkMax intakeMotorController;
private DoubleSolenoid intakeArm;


SlewRateLimiter filter;


    public Intake() {

        //Define all Motor Controllers
        intakeMotorController = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushed);
        intakeMotorController.restoreFactoryDefaults();



        filter = new SlewRateLimiter(slewRate.getDouble(0));

        //Define Double Solenoid
        intakeArm = new DoubleSolenoid(DriveConstants.PCM_ADDRESS, PneumaticsModuleType.CTREPCM, IntakeConstants.EXTEND_INTAKE_ARM, IntakeConstants.RETRACT_INTAKE_ARM);

        this.retractArm();

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
    }


    public void extendArm () {
        intakeArm.set(Value.kForward);
    }

     public void retractArm () {
        intakeArm.set(Value.kReverse);
    }
}

