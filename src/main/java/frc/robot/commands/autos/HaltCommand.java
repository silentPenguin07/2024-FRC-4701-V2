package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.IntakeSubsystem;
import frc.robot.robot_subsystems.ShooterSubsystem;

public class HaltCommand extends Command {
    
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;

    public HaltCommand(ShooterSubsystem shooter, IntakeSubsystem intake)
    {
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(shooter, intake);
    }

    public void initialize() {}

    public void execute()
    {
        shooter.brake();
        intake.brake();
    }

    public void end(boolean interrupted) {}

    public boolean isFinished()
    {
        return false;
    }
}
