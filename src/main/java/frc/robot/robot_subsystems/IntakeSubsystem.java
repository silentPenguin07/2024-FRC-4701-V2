package frc.robot.robot_subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    private Spark controller;

    public IntakeSubsystem()
    {
        controller = new Spark(1);
    }

    public void intake(boolean reverse)
    {
        if (!reverse)
        {
            controller.set(.7);
        }
        else
        {
            controller.set(-.8);
        }
    }

    public void brake()
    {
        controller.set(0);
    }

}
