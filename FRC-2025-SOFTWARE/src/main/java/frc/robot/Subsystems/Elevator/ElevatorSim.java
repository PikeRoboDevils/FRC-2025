// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

/** Add your docs here. */
public class ElevatorSim implements ElevatorIO {

    private edu.wpi.first.wpilibj.simulation.ElevatorSim _elevator;
    public Mechanism2d _mech;
    private MechanismRoot2d _root;
    private MechanismLigament2d _elevatorMech;

    public ElevatorSim() {

        double[] stdDevs = new double[2];
        stdDevs[0] = 0.02;
        stdDevs[1] = 0.02;
        _elevator = new edu.wpi.first.wpilibj.simulation.ElevatorSim(
        DCMotor.getNEO(2),
        1,
        10, 
        0.01905,// .75 inches not sure
        0.2527554,// 9.951 inches
        2.1336,// 84 inches 
        true,
        2.1336, stdDevs
        );

         _mech = new Mechanism2d(0,0);
         _root = _mech.getRoot("Elevator", 0, 0);
         _elevatorMech = _root.append(new MechanismLigament2d("Elevator", 2, 90));

    }
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
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
        _elevator.setState(position,0);
    }

    @Override
    public void setVelocity(double speed) {
       _elevator.setState(getPosition(), speed);
    }

    @Override
    public double getVelocity(){
        return _elevator.getVelocityMetersPerSecond();}

    @Override
    public double getPosition(){
        return _elevator.getPositionMeters();}

    @Override
    public double getVoltage(){
        return 0;} //no direct way  
        
    @Override
    public Mechanism2d getMech(){
            return _mech;} 

}
