// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

/** Add your docs here. */
public interface ClimberIO {
        @AutoLog
    public static class ClimberIOInputs{
        public double WristEncoderAngle = 0.0;
        public double WristVelocity = 0.0;
        public double WristVolt = 0.0;
        public double WristCurrent = 0.0;
        public double WristInternalAngle = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}
        /** 
     * Set wrist angle in degrees
     * 
     * @param angleDeg
    */
    public default void setAngle(double angleDeg){}

    /** Set voltage from 0-1 */
    public default void setVoltage(double speed) {}
    
    public default void setVelocity(double speed) {}
    public default double getVelocity(){return 0;}

    public default double getAngleDeg(){return 0;}
    public default double getAngleRad(){return 0;}
    public default double getVoltage(){return 0;}
    public default Mechanism2d getMech(){return new Mechanism2d(0, 0);}
}
