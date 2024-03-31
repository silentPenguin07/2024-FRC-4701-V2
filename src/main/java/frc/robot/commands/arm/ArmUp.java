package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.ArmSubsystem;

public class ArmUp extends Command {
    
    private ArmSubsystem arm;

    public ArmUp(ArmSubsystem arm)
    {
        this.arm = arm;
        addRequirements(arm);
    }

    public void initialize()
    {
        arm.stop();
    }

    public void execute()
    {
        arm.up();
    }

    public void end(boolean interrupted)
    {
        arm.stop();
    }

    public boolean isFinished()
    {
        return false;
    }

}
