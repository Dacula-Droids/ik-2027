// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private static IntakeSubsystem INSTANCE = new IntakeSubsystem();

  @SuppressWarnings("WeakerAccess")
  public static IntakeSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new IntakeSubsystem();
    }
    return INSTANCE;
  }

  private SmartMotorControllerConfig intakeSmcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(1, 0, 0)
      .withSimClosedLoopController(1, 0, 0)
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as
      // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
      // your motor.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private SparkMax intakeMotor = new SparkMax(Constants.IntakeConstants.IntakeMotorID, MotorType.kBrushless);
  private SmartMotorController intakeSmartMotorController = new SparkWrapper(intakeMotor, DCMotor.getNEO(1),
      intakeSmcConfig);

  private final FlyWheelConfig intakeFlywheelConfig = new FlyWheelConfig(intakeSmartMotorController) // New YAMS Fix
      // Diameter of the flywheel.
      .withDiameter(Inches.of(4))
      // Telemetry name and verbosity for the arm.
      .withTelemetry("IntakeMech", TelemetryVerbosity.HIGH);
  // Intake Mechanism
  private FlyWheel intake = new FlyWheel(intakeFlywheelConfig);

  public IntakeSubsystem() {
  }

  public AngularVelocity getIntakeAngularVelocity() {
    return intake.getSpeed();
  }

  public Command setDutyCycle(double dutyCycle) {
    return intake.set(dutyCycle);
  }

  public void setIntakeVelocitySetpoint(AngularVelocity speed) {
    intake.setMechanismVelocitySetpoint(speed);
  }

  @Override
  public void periodic() {
    intake.updateTelemetry();
    // This method will be called once per scheduler run
  }
}
