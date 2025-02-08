// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        public double ElevatorVelocity = 0.0;
        public double ElevatorVolt = 0.0;
        public double ElevatorCurrent = 0.0;
        public double ElevatorInternalAngle = 0.0;
    }
    
    public default void updateInputs(ElevatorIOInputs inputs){}

    /** Set voltage from 0-1 */
    public default void setVoltage(double speed) {}
    public default void setPosition(double position){}

    public default void setVelocity(double speed) {}
    public default double getVelocity(){return 0;}

    public default double getPosition(){return 0;}
    public default double getVoltage(){return 0;}
    public default Mechanism2d getMech(){return new Mechanism2d(0, 0);}

}
