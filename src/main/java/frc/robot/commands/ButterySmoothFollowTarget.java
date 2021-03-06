package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

public class ButterySmoothFollowTarget extends CommandBase {

    private final DriveTrain driveSubsystem;


    public ButterySmoothFollowTarget(DriveTrain driveSubsystem) {

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

        this.driveSubsystem.setMotors(DriveConstants.BUTTERY_FOLLOWING_SPEED * x_angle, -DriveConstants.BUTTERY_FOLLOWING_SPEED * x_angle);
   


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