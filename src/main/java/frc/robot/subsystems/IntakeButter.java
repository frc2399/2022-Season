
package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;
//import frc.robot.commands.*;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeButter implements IntakeBase {
private CANSparkMax intakeMotorController;

    public IntakeButter() {

        //Define all Motor Controllers
        intakeMotorController = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushed);
    }

    public void setMotor(double intakeSpeed) {
        intakeMotorController.set(intakeSpeed);
        System.out.println("intake setspeed" + intakeSpeed);
        SmartDashboard.putNumber("Intake speed ", intakeSpeed);
        }

}

