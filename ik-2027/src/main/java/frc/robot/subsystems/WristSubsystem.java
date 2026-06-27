// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class WristSubsystem extends SubsystemBase {

  /** Creates a new WristSubsystem. */
  private SparkMax wristMotor = new SparkMax(Constants.WristConstants.WristMotorID, MotorType.kBrushless);
  private static WristSubsystem INSTANCE = new WristSubsystem();

  @SuppressWarnings("WeakerAccess")
  public static WristSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new WristSubsystem();
    }
    return INSTANCE;
  }

  private SmartMotorControllerConfig wristSmcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(37, 0, 0)// kP 170, 0, 1
      .withSimClosedLoopController(50, 0, 0)
      .withFeedforward(new ArmFeedforward(0.06, 0.037, 0)) // 0.3 ks, 0.25 kg, 2 kv q2 //0.23 kg, 0.32 ks, 0 kv
      .withSimFeedforward(new ArmFeedforward(0, 0, 0))
      .withTelemetry("Intake Pivot Motor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(Constants.WristConstants.WristGearRatio)) // 12:1 Gear Ratio
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(50))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withSoftLimits(Constants.WristConstants.wristLowerSoftLimit, Constants.WristConstants.wristUpperSoftLimit)
      .withStartingPosition(Constants.WristConstants.wristStartingPosition); // PID Controller, Max Velocity, Max
                                                                             // Acceleration;

  private SmartMotorController wristSmartMotorController = new SparkWrapper(wristMotor, DCMotor.getNEO(1),
      wristSmcConfig);

  private PivotConfig wristPivotConfig = new PivotConfig(wristSmartMotorController)
      .withTelemetry("Wrist Pivot", TelemetryVerbosity.HIGH);

  private Pivot wrist = new Pivot(wristPivotConfig);

  // Set angle of Wrist, but command and Wrist does not stop
  public Command setWristAngle(Angle angle) {
    return wrist.run(angle);
  }

  public void setWristVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }

  // public Command setAngleAndStop(Angle angle){
  // return intakePivot.runTo(angle);
  // }

  // Closed Loop controller for Intake
  public void setWristSetpoint(Angle angle) {
    wrist.setMechanismPositionSetpoint(angle);
  }

  // dutycycle, incase open loop needed during testing
  public Command setWristDutyCycle(double dutycycle) {
    return wrist.set(dutycycle);
  }

  // public Command sysId() {
  // return intakePivot.sysId(Volts.of(7), Volts.of(2).per(Second),
  // Seconds.of(4));
  // }

  public WristSubsystem() {

  }

  @Override
  public void periodic() {
    wrist.updateTelemetry();
    // This method will be called once per scheduler run
  }
}
