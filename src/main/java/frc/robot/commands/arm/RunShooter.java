package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.ShooterSubsystem;

public class RunShooter extends Command {
    
    private final ShooterSubsystem shooter;
    private boolean reverse;

    public RunShooter(ShooterSubsystem s, boolean r)
    {
        shooter = s;
        reverse = r;
    }

    public void initialize() 
    {
        shooter.brake();
    }

    public void execute()
    {
        shooter.shoot(reverse);
    }

    public void end(boolean interrupted)
    {

    }

    public boolean isFinished()
    {
        return false;
    }

}
