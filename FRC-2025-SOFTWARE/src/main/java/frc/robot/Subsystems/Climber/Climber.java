// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  ClimberIO io;

  ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private Pose3d _climbPose;

  public Climber(ClimberIO climberIO) {
    this.io = climberIO;
  }

  @Override
  public void periodic() {
    _climbPose =
        new Pose3d(new Translation3d(-0.065, 0.0, -0.095), new Rotation3d(0, io.getAngleRad(), 0));

    Logger.recordOutput("Components/Climber", _climbPose);

    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command setAngle(DoubleSupplier angle) {
    return run(() -> io.setAngle(angle.getAsDouble()));
  }

  public Command setVoltage(DoubleSupplier volts) {
    return run(() -> io.setVoltage(volts.getAsDouble()));
  }
}
