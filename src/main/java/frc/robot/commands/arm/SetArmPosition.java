package frc.robot.commands.arm;

import frc.robot.RobotConstants.ArmConstants;
import frc.robot.robot_subsystems.ArmSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class SetArmPosition extends Command {
    
    // instance data
    private ArmSubsystem arm;
    private double targetPosition_rad;

    /**
	 * Command to set the arm position, cancels the command and moves on 
	 * after being within the range given by {@link ArmConstants#accuracyTolerance_deg accuracyTolerance}
	 * @param arm Arm subsystem
	 * @param targetPosition_deg Position in degrees to set the arm to (0 is horizontal to robot base)
	 */
	public SetArmPosition(ArmSubsystem arm, double targetPosition_deg) {
		this.arm = arm;
		this.targetPosition_rad = Units.degreesToRadians(targetPosition_deg);
		addRequirements(arm);
	}

    @Override
    public void initialize()
    {
        arm.setPosition(targetPosition_rad);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted)
    {
        // only stops if interrupted. Otherwise, continues adjusting arm position and moves on to another command
        if (interrupted)
        {
            arm.stop();
        }
    }

    // returns true when command should end
    public boolean isFinished()
    {
        return arm.closeEnough();
    }

}
