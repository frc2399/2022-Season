package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class FollowTarget extends CommandBase {

    private final DriveTrain driveSubsystem;


    public FollowTarget(DriveTrain driveSubsystem) {

        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("FollowTarget started!");
    }

    @Override
    public void execute() {
        double x_angle = Robot.tx.getDouble(1);
        if (x_angle > 0){
            this.driveSubsystem.setMotors(DriveConstants.TARGET_FOLLOWING_SPEED, -DriveConstants.TARGET_FOLLOWING_SPEED);
            SmartDashboard.putString("Turning", "Turning Right");

        }
        else if (x_angle < 0){
            this.driveSubsystem.setMotors(-DriveConstants.TARGET_FOLLOWING_SPEED, DriveConstants.TARGET_FOLLOWING_SPEED);
            SmartDashboard.putString("Turning", "Turning Left");

        }
        else{
            this.driveSubsystem.setMotors(0, 0);
            SmartDashboard.putString("Turning", "Not Turning");

        }
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