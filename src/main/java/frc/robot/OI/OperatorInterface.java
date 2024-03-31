package frc.robot.OI;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.CommandFactory;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotConstants.ArmConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.XCommand;
import frc.robot.commands.arm.ArmUp;
import frc.robot.commands.arm.HaltArm;
import frc.robot.commands.arm.RunIntake;
import frc.robot.commands.arm.RunShooter;
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

                // TODO: fix these angles
                /* 
                robotContainer.getArmSubsystem()
                                .setDefaultCommand(new ManualArmCommand(robotContainer.getArmSubsystem(), operatorController::getLeftY));
                */
                new JoystickButton(operatorController, 1).onTrue(new SetArmPosition(robotContainer.getArmSubsystem(), ArmConstants.LOW_deg - ArmConstants.armOffset_deg)); // A-low for speaker
                new JoystickButton(operatorController, 2).onTrue(new HaltArm(robotContainer.getArmSubsystem()));
                new JoystickButton(operatorController, 4).whileTrue(new ArmUp(robotContainer.getArmSubsystem())); // Y-high for amp

                new JoystickButton(operatorController, 5).whileTrue(new RunIntake(robotContainer.getIntakeSubsystem(), false));
                new JoystickButton(operatorController, 6).whileTrue(new RunShooter(robotContainer.getShooterSubsystem(), false));

                new JoystickButton(operatorController, 9).whileTrue(new RunIntake(robotContainer.getIntakeSubsystem(), true));
                //new JoystickButton(operatorController, 2).whileTrue(new RunShooter(robotContainer.getShooterSubsystem(), true));

                // TODO: implement joystick controlled arm!
        }
}