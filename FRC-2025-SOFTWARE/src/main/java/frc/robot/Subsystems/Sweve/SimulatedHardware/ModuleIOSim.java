package frc.robot.Subsystems.Sweve.SimulatedHardware;

import static edu.wpi.first.units.Units.Amps;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

// This is only an example simulation IO implement, please change the code according to your ModuleIO interface
public class ModuleIOSim {
    private static final double GEAR_RATIO = 1;
    // reference to module simulation
    private final SwerveModuleSimulation moduleSimulation;
    // reference to the simulated drive motor
    private final SimulatedMotorController.GenericMotorController driveMotor;
    // reference to the simulated turn motor
    private final SimulatedMotorController.GenericMotorController turnMotor;

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;

        // configures a generic motor controller for drive motor
        // set a current limit of 60 amps
        this.driveMotor = moduleSimulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(Amps.of(60));
        this.turnMotor = moduleSimulation
                .useGenericControllerForSteer()
                .withCurrentLimit(Amps.of(20));
    }

     // specified by ModuleIO interface
    public void setDriveOutputVoltage(Voltage voltage) {
        this.driveMotor.requestVoltage(voltage);
    }

     // specified by ModuleIO interface
    public void setSteerOutputVoltage(Voltage voltage) {
        this.turnMotor.requestVoltage(voltage);
    }

     // specified by ModuleIO interface
    public Rotation2d getSteerFacing() {
        return this.moduleSimulation.getSteerAbsoluteFacing();
    }

     // specified by ModuleIO interface
    public Angle getSteerRelativePosition() {
        return moduleSimulation.getSteerRelativeEncoderPosition().divide(GEAR_RATIO);
    }

     // specified by ModuleIO interface
    public Angle getDriveWheelrPositiond() {
        return moduleSimulation.getDriveWheelFinalPosition();
    }
}