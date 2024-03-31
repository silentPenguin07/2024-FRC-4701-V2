package frc.robot.commands.arm;

import frc.robot.RobotConstants.ArmConstants;
import frc.robot.robot_subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SetArmPosition extends Command {
    
    private ArmSubsystem arm;
	protected double targetPosition_deg; // did I do this right?

	/**
	 * Command to set the arm position, cancels the command and moves on 
	 * after being within the range given by {@link ArmConstants#accuracyTolerance_deg accuracyTolerance}
	 * @param arm Arm subsystem
	 * @param targetPosition_deg Position in degrees to set the arm to (0 is horizontal to robot base)
	 */
	public SetArmPosition(ArmSubsystem arm, double targetPosition_deg) {
		this.arm = arm;
		this.targetPosition_deg = targetPosition_deg;
		addRequirements(arm);
	}

	@Override
	public void initialize() {
		
		arm.setPosition(targetPosition_deg);
		
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {
		// only stop if interrupted. Otherwise, continue adjusting arm position just move on to another command
		if (interrupted) {
			arm.stop();
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return arm.closeEnough();
	}

}
