
package frc.robot.subsystems;
//import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.LimelightConstants;

//import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {


SlewRateLimiter filter;


    public Limelight() {


    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double y_angle = Robot.ty.getDouble(1);
        // SmartDashboard.putNumber("y_angle", Robot.ty.getDouble(1));
        double y_angle_radians = Math.toRadians(y_angle);
        // SmartDashboard.putNumber("y_angle_radians", y_angle_radians);
        double y_distance = LimelightConstants.HEIGHT_INCHES/(Math.tan(y_angle_radians));
        // SmartDashboard.putNumber("y_distance", y_distance);

        double x_angle = Robot.tx.getDouble(1);
        // SmartDashboard.putNumber("x_angle", Robot.tx.getDouble(1));
        double x_angle_radians = Math.toRadians(x_angle);
        // SmartDashboard.putNumber("x_angle_radians", x_angle_radians);
        double x_distance = y_distance * Math.sin(x_angle_radians);
        // SmartDashboard.putNumber("x_distance", x_distance);
    




    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setMotor(double intakeSpeed) {

        }

}
