package frc.robot.Subsystems.Climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimberSim implements ClimberIO {
  private SingleJointedArmSim _Climber;

  private ArmFeedforward _feedforward;
  private ProfiledPIDController _profiledPIDController;

  public ClimberSim() {
    _Climber = new SingleJointedArmSim(DCMotor.getNEO(1), 2.2, 8, 8, 8, 9, true, 0);

    _feedforward = new ArmFeedforward(1, 1, 1);
    _profiledPIDController = new ProfiledPIDController(5, 3, 6, new Constraints(1, 2));
  }

  @Override
  public void setAngle(double angleDeg) {
    double ffw =
        _feedforward.calculate(
            Units.degreesToRadians(angleDeg),
            _Climber.getAngleRads() - Units.degreesToRadians(angleDeg));
    double pid =
        _profiledPIDController.calculate(_Climber.getAngleRads(), Units.degreesToRadians(angleDeg));

    _Climber.setInputVoltage(MathUtil.clamp(ffw + pid, -12, 12));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    _Climber.update(0.02);

    inputs.ClimberCurrent = _Climber.getCurrentDrawAmps();
    inputs.ClimberEncoderAngle = Units.radiansToDegrees(_Climber.getAngleRads());
    inputs.ClimberVolt =
        _Climber.getInput(
            0); // I think this is the reference for voltage. Still needs checked, it would apear to
    // be
    inputs.ClimberVelocity =
        Units.radiansPerSecondToRotationsPerMinute(_Climber.getVelocityRadPerSec());
  }
}
 