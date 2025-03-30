// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CoralIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

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
    return run(() -> io.setVoltage(volts.getAsDouble())).finallyDo(() -> io.setVoltage(0));
  }

  public Command runIntake(DoubleSupplier speed) {
    return run(() -> io.setVelocity(speed.getAsDouble()))
        .unless(() -> hasCoral())
        .finallyDo(() -> io.setVelocity(1)) // holding speed
        .until(() -> !hasCoral());
  }

  public Command runIntakeVolts(DoubleSupplier speed) {
    return run(() -> io.setVoltage(speed.getAsDouble()))
        .unless(() -> io.hasCoral())
        .finallyDo(() -> io.setVoltage(1))
        .until(() -> !io.hasCoral());
  }

  public Command runIntakeAuto() {
    if (Robot.isSimulation()){addCoralSim();}
    return run(() -> io.setVoltage(3)).until(() -> hasCoral());
  }

  public Command runOutakeAuto(double volt) {
    return run(() -> io.setVoltage(volt)).withTimeout(.5);
  }

  public Boolean hasCoral() {
    return io.hasCoral();
  }
  public void addCoralSim(){
    io.addCoral();
  }
}
