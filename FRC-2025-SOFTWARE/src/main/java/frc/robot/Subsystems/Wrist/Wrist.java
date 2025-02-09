// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

  WristIO io;
  /** Creates a new Wrist. */
  public Wrist(WristIO wristIo) {
    this.io = wristIo;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putData("Wrist/Mech",io.getMech());
  }

  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("Mechinisms/Wrist",io.getMech());
  }
}
