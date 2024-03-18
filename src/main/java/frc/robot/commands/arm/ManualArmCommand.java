package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.robot_subsystems.ArmSubsystem;

public class ManualArmCommand extends Command {
    
    public static final double MAX_SPEED_PERCENT = .5; // don't trust the operator to be gentle

    private final ArmSubsystem arm;
    private final XboxController joystick;

    public ManualArmCommand(ArmSubsystem arm, XboxController joystick)
    {
        this.arm = arm;
        this.joystick = joystick;
        addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute()
    {
        // calculations (just copy paste)
        double yRaw = joystick.getLeftY();
        
        double yConstrained = MathUtil.applyDeadband(MathUtil.clamp(yRaw, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT),
                RobotConstants.Ports.CONTROLLER.ARM_JOYSTICK_AXIS_THRESHOLD);

        double ySquared = Math.copySign(yConstrained * yConstrained, yConstrained);

        if (ySquared < 0)
        {
            arm.stop();
        }

        // TODO: properly scale joystick input to arm output voltage (yay for NEOs)
        arm.setMotorVoltage(-ySquared);
    }



}
