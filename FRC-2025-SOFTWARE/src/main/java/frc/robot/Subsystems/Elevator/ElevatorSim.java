// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

/** Add your docs here. */
public class ElevatorSim implements ElevatorIO {

    private edu.wpi.first.wpilibj.simulation.ElevatorSim _elevator;

    private ElevatorFeedforward _feedforward;
    private ProfiledPIDController _profiledPIDController;

    public ElevatorSim() {

        double[] stdDevs = new double[2];
        stdDevs[0] = 0.000002;
        stdDevs[1] = 0.000002;
        
        _elevator = new edu.wpi.first.wpilibj.simulation.ElevatorSim(
            DCMotor.getNEO(2),
            9,
            10, 
            Units.inchesToMeters(0.75),// .75 inches not sure
            Units.inchesToMeters(0),//
            Units.inchesToMeters(74),// 84 inches //74 because of model rigging
            true,
            0, stdDevs
        );

        //TODO: fix the PID and FEED Forward to not be cursed
        //position control
        _feedforward = new ElevatorFeedforward(2, 0,0);
        _profiledPIDController = new ProfiledPIDController(6, 0,0, new Constraints(2, 1));

        _elevator.setState(0, 0);
    }
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

        _elevator.update(0.02);

        inputs.ElevatorVelocity = getVelocity();
        inputs.ElevatorVolt = getVoltage();
        inputs.ElevatorCurrent = _elevator.getCurrentDrawAmps();
        inputs.ElevatorPosition = _elevator.getPositionMeters();
    }
    

    @Override
    public void setVoltage(double speed) {
        _elevator.setInputVoltage(speed);
    }

    @Override
    public  void setPosition(double position){
        double volts = MathUtil.clamp(_feedforward.calculate(position-getPosition()) + _profiledPIDController.calculate(getPosition(), position), -12, 12);
        _elevator.setInputVoltage(volts);
    }

    @Override
    public void setVelocity(double speed) {
        double volts = MathUtil.clamp(_profiledPIDController.calculate(speed, getVelocity()), 0, 0); //im lazy (also dont know if we need a set velocity)
       _elevator.setInputVoltage(volts);
    }

    @Override
    public double getVelocity(){
        return _elevator.getVelocityMetersPerSecond();}

    @Override
    public double getPosition(){
        return _elevator.getPositionMeters();
    }

    @Override
    public double getVoltage(){
        return 0;
    } //no direct way  

}
