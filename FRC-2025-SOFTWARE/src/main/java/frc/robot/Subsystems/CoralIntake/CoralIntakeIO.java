// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CoralIntake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface CoralIntakeIO {

  @AutoLog
  public static class CoralIOInputs {
    public double IntakeVelocity = 0.0;
    public double IntakeVolt = 0.0;
    public double IntakeCurrent = 0.0;
    public boolean hasCoral = false;
  }

  /** Set voltage from 0-1 */
  public default void setVoltage(double speed) {}

  public default void updateInputs(CoralIOInputsAutoLogged inputs) {}

  public default void setVelocity(double speed) {}

  public default void addCoral() {}

  public default double getVelocity() {
    return 0;
  }

  public default double getVoltage() {
    return 0;
  }

  public default boolean hasCoral() {
    return false;
  }
}
