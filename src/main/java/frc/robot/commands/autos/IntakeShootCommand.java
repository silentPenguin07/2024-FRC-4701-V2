package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.IntakeSubsystem;
import frc.robot.robot_subsystems.ShooterSubsystem;

public class IntakeShootCommand extends Command {
    
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;

    public IntakeShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake)
    {
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(shooter, intake);
    }

    public void initialize() {}

    public void execute()
    {
        intake.intake(false);
        shooter.shoot(false);
    }


    public void end(boolean interrupted) {}

    public boolean isFinished()
    {
        return false;
    }

}
