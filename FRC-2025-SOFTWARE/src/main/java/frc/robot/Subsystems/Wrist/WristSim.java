// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

/** Add your docs here. */
public class WristSim implements WristIO {

    private SingleJointedArmSim _wrist;

    private Pose3d _wristPose;

    private ArmFeedforward _feedforward;
    private PIDController _pid;

    public WristSim() {
        _wrist = new SingleJointedArmSim(DCMotor.getNEO(1),
         50,
        SingleJointedArmSim.estimateMOI(Units.inchesToMeters(5),
         15),
          Units.inchesToMeters(5),
          -90,
           90,
            true,
              Units.degreesToRadians(10));
        

        _feedforward = new ArmFeedforward(0, 0, 0);
        _pid = new PIDController(0, 0, 0);

        _wristPose = new Pose3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
    } 

    @Override
    public void updateInputs(WristIOInputs inputs) {
        
        _wrist.update(0.02);
        _wristPose = new Pose3d(new Translation3d(0,0,0), new Rotation3d(0,getAngleDeg(),0));
        
        inputs.WristPose = _wristPose;
        inputs.WristCurrent = _wrist.getCurrentDrawAmps();
        inputs.WristEncoderAngle = Units.radiansToDegrees(_wrist.getAngleRads());
        inputs.WristVolt = _wrist.getInput(0); //I think this is the reference for voltage. Still needs checked
        inputs.WristVelocity = Units.radiansPerSecondToRotationsPerMinute(_wrist.getVelocityRadPerSec());
    }

    @Override
    public void setAngle(double angleDeg) {
        double ffw = _feedforward.calculate(Units.degreesToRadians(angleDeg), Units.degreesToRadians(angleDeg)-_wrist.getAngleRads());
        double pid = _pid.calculate(_wrist.getAngleRads(), Units.degreesToRadians(angleDeg));

        _wrist.setInputVoltage(MathUtil.clamp(ffw+pid, -12, 12));
    }

    @Override
    public void setVoltage(double speed) {
        _wrist.setInputVoltage(speed*12);
    }

    @Override
    public double getAngleDeg() {
        return Units.radiansToDegrees(_wrist.getAngleRads());
    }
    @Override
    public double getAngleRad() {
        return _wrist.getAngleRads();
    }
    @Override
    public double getVoltage() {
        return 0;
    }

}
