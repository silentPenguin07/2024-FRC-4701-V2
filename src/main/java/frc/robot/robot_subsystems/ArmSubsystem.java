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
	private SparkPIDController pidController = leadMotor.getPIDController();
	//private RelativeEncoder motorEncoder = leadMotor.getEncoder(); // built-in encoder in the lead NEO
	// throughbore encoder on hex shaft
	private DutyCycleEncoder absEncoder = new DutyCycleEncoder(0);
	private RelativeEncoder relativeEncoder = leadMotor.getEncoder();

	private ArmFeedforward feedforwardController = new ArmFeedforward(kS, kG, kV, kA);


    private double targetPosition_deg = 0;

    // constructor

    public ArmSubsystem()
    {
        leadMotor.restoreFactoryDefaults();
		leadMotor.setIdleMode(IdleMode.kBrake);
		leadMotor.setInverted(false);
		leadMotor.setSmartCurrentLimit(40); // sets current limit in amps
		leadMotor.setSecondaryCurrentLimit(40);
		leadMotor.enableVoltageCompensation(12);

		leadMotor.setSoftLimit(SoftLimitDirection.kForward, (float) (Units.degreesToRotations(100)));
		leadMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) (Units.degreesToRotations(40)));
		leadMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
		leadMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
		
		followerMotor.restoreFactoryDefaults();
		followerMotor.setIdleMode(IdleMode.kBrake);
		followerMotor.setSmartCurrentLimit(40);
		followerMotor.setSecondaryCurrentLimit(40);
		followerMotor.enableVoltageCompensation(12);
		followerMotor.follow(leadMotor, true);

		// revolutions * deg / rev = deg
		relativeEncoder.setPositionConversionFactor(360 / 35);
		// rev / sec * sec / min = RPM
		relativeEncoder.setVelocityConversionFactor(60);
		// TODO: ERROR: relativeEncoder.setInverted(true);
		relativeEncoder.setPosition(absEncoder.getAbsolutePosition() - ArmConstants.armOffset_deg);

		pidController.setFeedbackDevice(relativeEncoder);
		// Smart motion applies a velocity and acceleration limiter as it travels to the target position. More info can be
		// found here:
		// https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
		pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
		pidController.setSmartMotionMaxAccel(maxAccel_rpmps, 0);
		pidController.setSmartMotionMaxVelocity(maxVelocity_rpm, 0);
		pidController.setSmartMotionAllowedClosedLoopError(accuracyTolerance_deg, 0);
		pidController.setOutputRange(minOutput, maxOutput);
		// since we are using smartmotion, the PID numbers are for velocity control, not position.
		pidController.setP(kP);
		pidController.setI(kI);
		pidController.setD(kD);
		// Treats 0 and 360 degrees as the same number, so going from one side of 0 to the other doesnt make it do a 360
		pidController.setPositionPIDWrappingEnabled(false);

		// if (tuningMode) {
		// new AutoSetterTunableNumber("Arm/kP", kP, (value) -> pidController.setP(value));
		// new AutoSetterTunableNumber("Arm/kD", kD, (value) -> pidController.setD(value));
		// }

		leadMotor.burnFlash();
		followerMotor.burnFlash();
    }


	@Override

	// SmartDashboard output for troubleshooting
	public void periodic() {
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


	// TODO: Review these functions
	public void setPosition(double position_deg) {
		targetPosition_deg = position_deg;
		pidController.setReference(targetPosition_deg, ControlType.kPosition, 0,
			feedforwardController.calculate(Units.degreesToRadians(targetPosition_deg), 0));
	}
	/* 
	public void setMotorVoltage(double voltage_V) {
		SmartDashboard.putNumber("Arm/targetVoltage_V", voltage_V);
		pidController.setReference(voltage_V, ControlType.kVoltage, 0,
			feedforwardController.calculate(Units.degreesToRadians(relativeEncoder.getPosition() - relativeEncoderOffset_deg), 0));
	}*/

	public void up()
	{
		leadMotor.set(.15);
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
		return MathUtil.isNear(targetPosition_deg, relativeEncoder.getPosition(), accuracyTolerance_deg);
	}
    

}
