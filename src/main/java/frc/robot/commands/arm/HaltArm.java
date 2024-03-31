package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.ArmSubsystem;

public class HaltArm extends Command {
    
    private ArmSubsystem arm;
    
    public HaltArm(ArmSubsystem arm)
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

    }

    public void end(boolean interrupted)
    {

    }

    public boolean isFinished()
    {
        return false;
    }

}
