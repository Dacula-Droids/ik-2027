// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShoulderSubsystem extends SubsystemBase {
	private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
			.withControlMode(ControlMode.CLOSED_LOOP)
			// Feedback Constants (PID Constants)
			.withClosedLoopController(50, 0, 0)
			.withTrapezoidalProfile(DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
			.withSimClosedLoopController(50, 0, 0)
			// Feedforward Constants
			.withFeedforward(new ArmFeedforward(0, 0, 0))
			.withSimFeedforward(new ArmFeedforward(0, 0, 0))
			// Telemetry name and verbosity level
			.withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
			// Gearing from the motor rotor to final shaft.
			// In this example GearBox.fromReductionStages(3,4) is the same as
			// GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
			// your motor.
			.withGearing(12)
			// Motor properties to prevent over currenting.
			.withMotorInverted(false)
			.withIdleMode(MotorMode.BRAKE)
			.withStatorCurrentLimit(Amps.of(40))
			.withClosedLoopRampRate(Seconds.of(0.25))
			.withOpenLoopRampRate(Seconds.of(0.25))
			.withSoftLimits(Degrees.of(-20), Degrees.of(10))
			.withStartingPosition(Degrees.of(-5));

	// Vendor motor controller object
	private SparkMax spark = new SparkMax(4, MotorType.kBrushless);

	// Create our SmartMotorController from our Spark and config with the NEO.
	private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

	private ArmConfig armCfg = new ArmConfig(sparkSmartMotorController)
			.withLength(Feet.of(3))
			.withMass(Pounds.of(1))
			// Telemetry name and verbosity for the arm.
			.withTelemetry("Arm", TelemetryVerbosity.HIGH);

	// Arm Mechanism
	private Arm arm = new Arm(armCfg);

	/** Creates a new ShoulderSubsystem. */
	public ShoulderSubsystem() {
	}

	public void setShoulderAngleSetpoint(Angle angle){
		arm.setMechanismPositionSetpoint(angle);
	}

	//Find out inversions
	public Command set(double dutycycle){
		return arm.set(dutycycle);
	}



	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
