// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.InverseKinematicsMath;



public class RobotContainer {
  WristSubsystem wristSubsystem = WristSubsystem.getInstance();


  public RobotContainer() {
    configureBindings();
  }

public Command getCoral(){
  Angle wristAngle = InverseKinematicsMath.calculateWristAngle(0.5, 0.5);

  return Commands.sequence(
  //void method conversion into command since sequence takes commands as parameters
    Commands.runOnce(() -> wristSubsystem.setWristSetpoint(wristAngle), wristSubsystem),
    
    Commands.waitSeconds(1)
  );
}

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
