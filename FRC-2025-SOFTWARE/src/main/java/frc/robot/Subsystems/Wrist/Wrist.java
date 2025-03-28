// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Wrist;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Elevator.Elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  WristIO io;
  Elevator elevatorStage;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private Pose3d _wristPose;

  private boolean isBad;

  public BooleanSupplier wristDisabled = ()->isBad;

  /** Creates a new Wrist. */
  public Wrist(WristIO wristIo, Elevator elevatorStage) {
    this.io = wristIo;
    this.elevatorStage = elevatorStage;

    _wristPose =
        new Pose3d(
            new Translation3d(0, 0, 0).plus(elevatorStage.stage3Visuals.getTranslation()),
            new Rotation3d(0, 0, 0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _wristPose =
        new Pose3d(
            new Translation3d(0.065, 0.0, 0.095).plus(elevatorStage.stage3Visuals.getTranslation()),
            new Rotation3d(
                0, Units.degreesToRadians(io.getAngleDeg() + 90), Units.degreesToRadians(180)));

    Logger.recordOutput("Components/Wrist", _wristPose);


    Logger.recordOutput("WRIST BAD", wristDisabled.getAsBoolean());

    io.updateInputs(inputs);
    Logger.processInputs("wrist", inputs);
  }
  // so we can set command to null 
  public Command setAngle(DoubleSupplier angle) {
      if (wristDisabled.getAsBoolean()) { return Commands.none();}// to prevent movement
      return run(() -> io.setAngle(angle.getAsDouble()));

  }

  public Command toggle() {
    
    return Commands.runOnce(()->isBad = !isBad,this);
  }

  public boolean getIsBad(){
      return wristDisabled.getAsBoolean();
  }

  public Command home() {
    return run(() -> io.setVoltage(1))
        .withTimeout(0.5)
        .beforeStarting(() -> io.setAngle(34))
        .withTimeout(0.1);
  }

  public Command setVoltage(DoubleSupplier volts) {
    return run(() -> io.setVoltage(volts.getAsDouble()));
  }

  public void disabled() {
    io.setVoltage(0);
  }
}
