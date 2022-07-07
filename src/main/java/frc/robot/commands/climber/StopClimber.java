package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

//sets climbers' speeds equal to zero, default command for th climber subsystem

public class StopClimber extends CommandBase {

    private final Climber m_climber;
   


    public StopClimber (Climber climber) {
        this.m_climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
        this.m_climber.setLeftSpeed(0);
        this.m_climber.setRightSpeed(0);
    }

    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished() {
        return false;

    }
}