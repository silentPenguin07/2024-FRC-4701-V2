package frc.robot.OI;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.CommandFactory;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.XCommand;
import frc.robot.commands.arm.SetArmPosition;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OperatorInterface {

        private final XboxController driveJoystick = new XboxController(RobotConstants.Ports.CONTROLLER.DRIVER_JOYSTICK);
        private final XboxController operatorController = new XboxController(RobotConstants.Ports.CONTROLLER.ARM_JOYSTICK);

        public OperatorInterface(CommandFactory commandFactory, RobotContainer robotContainer) {
                

                //TODO: Button numbers need to be changed
                new JoystickButton(driveJoystick, 2).onTrue(commandFactory.gyroResetCommand());
                new JoystickButton(driveJoystick, 3).onTrue(new XCommand());
                robotContainer.getDriveSubsystem()
                                .setDefaultCommand(new DriveCommand(robotContainer.getDriveSubsystem(), driveJoystick));

                new JoystickButton(operatorController, 1).onTrue(new SetArmPosition(robotContainer.getArmSubsystem(), 5)); // low for speaker
                new JoystickButton(operatorController, 3).onTrue(new SetArmPosition(robotContainer.getArmSubsystem(), 90)); // directly upwards for defense
                new JoystickButton(operatorController, 4).onTrue(new SetArmPosition(robotContainer.getArmSubsystem(), 48)); // high for amp

        }
}
