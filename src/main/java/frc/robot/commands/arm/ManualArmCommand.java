package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.robot_subsystems.ArmSubsystem;

public class ManualArmCommand extends Command {
    
    public static final double MAX_SPEED_PERCENT = .4; // don't trust the operator to be gentle

    private final ArmSubsystem arm;
    private DoubleSupplier speedSupplier;

    public ManualArmCommand(ArmSubsystem arm, DoubleSupplier speedSupplier)
    {
        this.arm = arm;
        this.speedSupplier = speedSupplier;
        addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute()
    {
        
        // apply deadband to input. Run stationary feedfordward ("stopped") if banded input is 0
        double deadbandInput = scaleAxis(speedSupplier.getAsDouble());


        if (deadbandInput == 0)
        {
            //arm.stop();
        }
        else
        {
            System.out.println(-(MAX_SPEED_PERCENT * deadbandInput) * 12);
            arm.setMotorVoltage(-(MAX_SPEED_PERCENT * deadbandInput) * 12);
        }

    }

    public void end(boolean interrupted)
    {
        arm.stop();
    }

    public boolean isFinished()
    {
        return false;
    }

    // helper method to scale joystick axis
	public static double scaleAxis(double value) {
		value = MathUtil.applyDeadband(value, 0.3); // second para is deadband value
		return Math.copySign(value * value, value);
	}

}
