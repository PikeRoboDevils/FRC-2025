// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Wrist;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface WristIO {

  @AutoLog
  public static class WristIOInputs {
    public double WristEncoderAngle = 0.0;
    public double WristVelocity = 0.0;
    public double WristVolt = 0.0;
    public double WristCurrent = 0.0;
    public double WristInternalAngle = 0.0;
    public boolean WristLimit = false;
    public boolean WristAtSetpoint;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  /**
   * Set wrist angle in degrees
   *
   * @param angleDeg
   */
  public default void setAngle(double angleDeg) {}

  /** Set voltage from 0-1 */
  public default void setVoltage(double speed) {}

  public default void setVelocity(double speed) {}

  public default double getVelocity() {
    return 0;
  }

  public default double getAngleDeg() {
    return 0;
  }

  public default double getAngleRad() {
    return 0;
  }

  public default double getVoltage() {
    return 0;
  }

  
  public default boolean atSetpoint() {
    return false;
  }
}
