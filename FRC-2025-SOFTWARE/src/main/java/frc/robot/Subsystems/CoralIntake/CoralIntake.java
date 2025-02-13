// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CoralIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */
  CoralIntakeIO io;

  CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

  public CoralIntake(CoralIntakeIO coralIntakeIO) {
    this.io = coralIntakeIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Coral Intake", inputs);
  }
}
