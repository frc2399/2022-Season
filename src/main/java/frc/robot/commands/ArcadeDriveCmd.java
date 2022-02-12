package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDriveCmd extends CommandBase {

    private final DriveTrain driveSubsystem;
    private final Supplier<Double> speedFunction, turnFunction;

    public ArcadeDriveCmd(DriveTrain driveSubsystem, //
            Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ArcadeDriveCmd started!");
    }

    @Override
    public void execute() {
        double realTimeSpeed;
        double realTimeTurn;

        // have deadband to prevent joystick drifting
        if (Math.abs(speedFunction.get()) <= JoystickConstants.FORWARD_DEADBAND) {
            realTimeSpeed = 0;
        } 
        else {
            realTimeSpeed = speedFunction.get() * JoystickConstants.FORWARD_JOYSTICK_INVERT;
        }

        // have deadband to prevent joystick drifting
        if (Math.abs(turnFunction.get()) <= JoystickConstants.TURN_DEADBAND) {
            realTimeTurn = 0;
        } 
        else {
            realTimeTurn = turnFunction.get() * JoystickConstants.TURN_JOYSTICK_INVERT;
        }

        SmartDashboard.putNumber("realTimeSpeed", realTimeSpeed);
        SmartDashboard.putNumber("realTimeTurn", realTimeTurn);

        double a = SmartDashboard.getNumber("a value", 0);

        // altering driving joystick sensitivity
        realTimeSpeed = ((1 - a) * realTimeSpeed) + (a * Math.pow(realTimeSpeed, 3));

        double left = realTimeSpeed + realTimeTurn;
        double right = realTimeSpeed - realTimeTurn;

        SmartDashboard.putNumber("left", left);
        SmartDashboard.putNumber("right", right);

        this.driveSubsystem.setMotors(left, right);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ArcadeDriveCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
