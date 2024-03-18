package frc.robot.robot_subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    private Spark controller;

    public IntakeSubsystem()
    {
        controller = new Spark(2);
    }

    public void intake(boolean reverse)
    {
        if (!reverse)
        {
            controller.set(.3);
        }
        else
        {
            controller.set(-.3);
        }
    }

    public void brake()
    {
        controller.set(0);
    }

}
