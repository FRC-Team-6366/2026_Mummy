package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
  private FlywheelSim intakeRollersMotor;
  private DCMotor INTAKE_ROLLER_GEARBOX = DCMotor.getKrakenX60(1);

  private DCMotorSim intakePivotMotor;
  private DCMotor INTAKE_PIVOT_GEARBOX = DCMotor.getKrakenX60(1);

  private double intakePivotAngleRad = 0.0;
  private double intakeRollerVolts = 0.0;

  static final double intakePivotMaxPosition = 0.5;
  double setPointTolerance = (Units.rotationsToRadians(intakePivotMaxPosition) / 100) * 5;

  public IntakeIOSim() {
    this.intakeRollersMotor = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            INTAKE_ROLLER_GEARBOX,
            1,
            3),
        INTAKE_ROLLER_GEARBOX, 0.004);

    this.intakePivotMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            INTAKE_PIVOT_GEARBOX,
            1,
            8.6),
        INTAKE_PIVOT_GEARBOX, 0.004, 0.04);
  }

  @Override
  public Rotation2d getRotations() {
    return Rotation2d.fromRadians(this.intakePivotMotor.getAngularPositionRad());
  }

  @Override
  public void intakePivotToAngle(double angle) {
    this.intakePivotAngleRad = Units.degreesToRadians(MathUtil.clamp(angle, 0.0, 138.6));
    this.intakePivotMotor.setAngle(this.intakePivotAngleRad);
  }

  @Override
  public double getIntakePivotPositionError() {
    return this.intakePivotAngleRad - this.intakePivotMotor.getAngularPositionRad();
  }

  @Override
  public boolean intakeAtPositionSetpoint() {
    return Math.abs(this.getIntakePivotPositionError()) < this.setPointTolerance;
  }

  @Override
  public void intakeResetCanCoder() {
    this.intakePivotMotor.setState(Units.rotationsToRadians(0.12), 0);
  }

  @Override
  public void setBrakeMode(boolean brakeMode) {}

  @Override
  public void rollersRunVolts(double power) {
    this.intakeRollerVolts = power * 12;
    this.intakeRollersMotor.setInputVoltage(this.intakeRollerVolts);
  }

  @Override
  public void rollersStop() {
    this.intakeRollerVolts = 0.0;
    this.intakeRollersMotor.setInputVoltage(this.intakeRollerVolts);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.connected = true;
    inputs.intakeRollersVolts = this.intakeRollersMotor.getInputVoltage();
    inputs.intakeRollersPosition = Double.POSITIVE_INFINITY;
    inputs.intakeRollersRps = this.intakeRollersMotor.getAngularVelocityRPM() / 60;
    inputs.intakeRollersCurrent = this.intakeRollersMotor.getCurrentDrawAmps();
    inputs.intakeRollersSupplyCurrent = this.intakeRollersMotor.getCurrentDrawAmps();
    inputs.intakePivotVolts = this.intakePivotMotor.getInputVoltage();
    inputs.intakePivotPosition = this.intakePivotMotor.getAngularPositionRotations();
    inputs.intakePivotRps = this.intakePivotMotor.getAngularVelocityRPM() / 60;
    inputs.intakePivotCurrent = this.intakePivotMotor.getCurrentDrawAmps();
    inputs.intakePivotSupplyCurrent = this.intakePivotMotor.getCurrentDrawAmps();
    inputs.intakePivotErrorFromSetpoint = this.getIntakePivotPositionError();
  }

}
