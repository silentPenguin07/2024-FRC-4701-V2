package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.IntakeSubsystem;
import frc.robot.robot_subsystems.ShooterSubsystem;

public class RunIntake extends Command {
    
    private final IntakeSubsystem intake;
    private boolean reverse;

    public RunIntake(IntakeSubsystem i, boolean r)
    {
        intake = i;
        reverse = r;
    }

    public void initialize() 
    {
        intake.brake();
    }

    public void execute()
    {
        intake.intake(reverse);
    }

    public void end(boolean interrupted)
    {

    }

    public boolean isFinished()
    {
        return false;
    }

}