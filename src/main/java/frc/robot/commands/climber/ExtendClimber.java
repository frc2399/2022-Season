package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ExtendClimber extends CommandBase {

    private final Climber m_climber;
    private final double speed;


    public ExtendClimber (Climber climber, double speed) {
        this.m_climber = climber;
        this.speed = speed;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
       if (m_climber.isLeftExtended()){
        this.m_climber.setLeftSpeed(0);   
       }
       else {
        this.m_climber.setLeftSpeed(speed);
       };

       if (m_climber.isRightExtended()){
        this.m_climber.setRightSpeed(0);   
       }
       else {
        this.m_climber.setRightSpeed(speed);
       };
    }

    @Override
    public void end(boolean interrupted) {
        this.m_climber.setLeftSpeed(0);
        this.m_climber.setRightSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if (m_climber.isLeftExtended() && m_climber.isRightExtended()) {
            return true;
        }
       return false;
    }
}