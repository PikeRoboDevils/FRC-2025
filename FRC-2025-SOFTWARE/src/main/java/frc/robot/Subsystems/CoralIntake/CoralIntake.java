// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CoralIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;
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

  public Command setVoltage(DoubleSupplier volts) {
    // StartEnd is more suitable here then run
    return startEnd(
    () -> io.setVoltage(volts.getAsDouble()),
    () -> io.setVoltage(0));
  }

  // Override for optional timeout (autonomous)
  public Command setVoltage(DoubleSupplier volts, double timeout) {
    return startEnd(
    () -> io.setVoltage(volts.getAsDouble()),
    () -> io.setVoltage(0))
    .withTimeout(timeout);
  }

  public Boolean hasCoral() {
    return io.hasCoral();
  }
  public void addCoralSim(){
    io.addCoral();
  }
}
