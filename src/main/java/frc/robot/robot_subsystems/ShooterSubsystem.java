package frc.robot.robot_subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    CANSparkMax leader, follower;

    public ShooterSubsystem()
    {
        leader = new CANSparkMax(10, MotorType.kBrushless);
        follower = new CANSparkMax(11, MotorType.kBrushless);

        follower.follow(leader, true);
    }

    // Precondition: reverse determines whether shooter spins forward or backward
    public void shoot(boolean reverse)
    {
        if (!reverse)
        {
            leader.set(1);
        }
        else
        {
            leader.set(-1);
        }
    }

    public void brake()
    {
        leader.set(0);
    }

}
