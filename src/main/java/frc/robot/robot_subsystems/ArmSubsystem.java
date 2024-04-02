package frc.robot.robot_subsystems;

import static frc.robot.RobotConstants.ArmConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    // instance data

    private CANSparkMax leadMotor = new CANSparkMax(leadMotorId, MotorType.kBrushless);
	private CANSparkMax followerMotor = new CANSparkMax(followerMotorId, MotorType.kBrushless);
	private DutyCycleEncoder absEncoder = new DutyCycleEncoder(0);
	private RelativeEncoder relativeEncoder = leadMotor.getEncoder();

	private ArmFeedforward feedforwardController = new ArmFeedforward(kS, kG, kV, kA);

	private ProfiledPIDController pidController =
		new ProfiledPIDController(kP, kI, kD, 
			new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL));

    // constructor

    public ArmSubsystem()
    {
        leadMotor.restoreFactoryDefaults();
		leadMotor.setIdleMode(IdleMode.kBrake);
		leadMotor.setInverted(false);
		leadMotor.setSmartCurrentLimit(40); // sets current limit in amps
		leadMotor.enableVoltageCompensation(12);

		
		followerMotor.restoreFactoryDefaults();
		followerMotor.setIdleMode(IdleMode.kBrake);
		followerMotor.setSmartCurrentLimit(40);
		followerMotor.enableVoltageCompensation(12);
		followerMotor.follow(leadMotor, true);

		// revolutions * deg / rev = deg
		relativeEncoder.setPositionConversionFactor(360 / 35);
		// rev / sec * sec / min = RPM
		relativeEncoder.setVelocityConversionFactor(60);
		// TODO: ERROR: relativeEncoder.setInverted(true);
		relativeEncoder.setPosition(absEncoder.getAbsolutePosition() - ARM_OFFSET);

		pidController.setTolerance(ARM_TOLERANCE);

		absEncoder.setPositionOffset(ARM_OFFSET);

		leadMotor.burnFlash();
		followerMotor.burnFlash();
    }


	@Override

	// SmartDashboard output for troubleshooting
	public void periodic() {

		useOutput(pidController.calculate(getMeasurement()), pidController.getSetpoint());

		SmartDashboard.putNumber("Lead Output Current", leadMotor.getOutputCurrent());
		SmartDashboard.putNumber("Lead Bus Voltage", leadMotor.getBusVoltage());
		SmartDashboard.putNumber("Lead Applied Voltage", 12 * leadMotor.getAppliedOutput());
		SmartDashboard.putNumber("Lead Motor Temperature", leadMotor.getMotorTemperature());
		SmartDashboard.putNumber("Lead Motor Percent Output", leadMotor.getAppliedOutput());

		SmartDashboard.putNumber("Follower Motor Output Current", followerMotor.getOutputCurrent());
		SmartDashboard.putNumber("Follower Motor Bus Voltage", followerMotor.getBusVoltage());
		SmartDashboard.putNumber("Follower Motor Applied Voltage", 12 * followerMotor.getAppliedOutput());
		SmartDashboard.putNumber("Follower Motor Motor Temperature", followerMotor.getMotorTemperature());
		SmartDashboard.putNumber("Follower Motor Motor Percent Output", followerMotor.getAppliedOutput());

		SmartDashboard.putNumber("Rel Encoder Position Deg", relativeEncoder.getPosition());
		SmartDashboard.putNumber("Abs Encoder Position Deg", absEncoder.getAbsolutePosition() * 360); // * 360 to convert from 0-1 range to degrees


	}

	public double getMeasurement()
	{
		return Math.abs(absEncoder.getAbsolutePosition() * 360);
	}

	public void setPosition(double position_deg) 
	{
		pidController.setGoal(new TrapezoidProfile.State(position_deg, 0));
	}

	private void useOutput(double output, TrapezoidProfile.State setPoint)
	{
		double ff = feedforwardController.calculate(setPoint.position, setPoint.velocity);

		double voltageOut = output + ff;

		leadMotor.setVoltage(voltageOut);
	}

	public void stop() {
		/*
		pidController.setReference(0, ControlType.kDutyCycle, 0,
			feedforwardController.calculate(Units.degreesToRadians(relativeEncoder.getPosition() - relativeEncoderOffset_deg), 0));
		// Set*/
		leadMotor.set(0);
		followerMotor.set(0);
	}

	public boolean closeEnough() {
		return pidController.atGoal();
	}
    

}
