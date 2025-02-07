// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

/** Add your docs here. */
public class ElevatorSim implements ElevatorIO {

    private edu.wpi.first.wpilibj.simulation.ElevatorSim _elevator;
    public Mechanism2d _mech;
    private MechanismRoot2d _root;
    private MechanismLigament2d _wristMech;

    public ElevatorSim() {

        double[] stdDevs = new double[2];
        stdDevs[0] = 0.02;
        stdDevs[1] = 0.02;
        //idk any of this yet //niether do I
        _elevator = new edu.wpi.first.wpilibj.simulation.ElevatorSim(
        DCMotor.getNEO(2),
        1,
        1,
        1,
        0,
        10,
        true,
        2, stdDevs
        );

         _mech = new Mechanism2d(0,0);
         _root = _mech.getRoot("Elevator", 0, 0);
         _wristMech = _root.append(new MechanismLigament2d("Elevator", 2, 90));





    }

}
