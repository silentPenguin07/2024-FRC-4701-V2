package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.ShooterSubsystem;

public class RevShooterCommand extends Command {
    
    private final ShooterSubsystem shooter;

    public RevShooterCommand(ShooterSubsystem shooter)
    {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    public void initialize() {}

    public void execute()
    {
        shooter.shoot(false);
    }

    public void end(boolean interrupted) {}

    public boolean isFinished()
    {
        return false;
    }

}
