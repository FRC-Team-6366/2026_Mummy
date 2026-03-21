package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  private FlywheelSim intakeRollersSim;
  private DCMotor INTAKE_ROLLER_GEARBOX = DCMotor.getKrakenX60(1);
  private double intakeRollerAppliedVolts = 0.0;

  private SingleJointedArmSim intakePivotSim;
  private DCMotor INTAKE_PIVOT_GEARBOX = DCMotor.getKrakenX60(1);
  private PIDController intakePivotPID = new PIDController(
    100.0, 
    1.0, 
    0.0, 
    0.020
  );
  private double intakePivotAppliedVolts = 0.0;
  private double intakePivotRotationSetPoint = 0.0;
  

  static final double intakePivotMaxPosition = 0.5;
  private double setPointTolerance = (Units.rotationsToRadians(intakePivotMaxPosition) / 100) * 5;
  private static final double rotationOffset = 0.12;

  public IntakeIOSim() {
    this.intakeRollersSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            INTAKE_ROLLER_GEARBOX,
            0.01,
            3),
        INTAKE_ROLLER_GEARBOX, 0.004);

    this.intakePivotSim = new SingleJointedArmSim(
      LinearSystemId.createSingleJointedArmSystem(
        INTAKE_PIVOT_GEARBOX, 
        0.5, 
        8.33
      ), 
      INTAKE_PIVOT_GEARBOX, 
      8.33, 
      1, 
      0.0, 
      2.4, 
      true, 
      0.558, 
      0.001, 0.004
    );
  }

  @Override
  public Rotation2d getRotations() {
    return Rotation2d.fromRotations(Units.radiansToRotations(this.intakePivotSim.getAngleRads()) + rotationOffset);
  }

  @Override
  public void intakePivotToAngle(double angle) {
    double rotations = (MathUtil.clamp(angle, 0.0, 138.6)) / 360; // Changes angle degree to rotations
    this.intakePivotRotationSetPoint = rotations + rotationOffset; // Add 0.12 rotation offset

    // Get the voltage from the PID Controller using the current Sim Angle and the new setpoint
    this.intakePivotAppliedVolts = this.intakePivotPID.calculate(
      this.getRotations().getRotations(), 
      this.intakePivotRotationSetPoint
    );
    
    this.intakePivotAppliedVolts = MathUtil.clamp(this.intakePivotAppliedVolts, -12.0, 12.0);
  }

  @Override
  public double getIntakePivotPositionError() {
    return this.intakePivotRotationSetPoint - Units.radiansToRotations(this.intakePivotSim.getAngleRads() + rotationOffset);
  }

  @Override
  public boolean intakeAtPositionSetpoint() {
    return Math.abs(this.getIntakePivotPositionError()) < this.setPointTolerance;
  }

  @Override
  public void intakeResetCanCoder() {
    this.intakePivotSim.setState(Units.rotationsToRadians(0.12), 0);
  }

  @Override
  public void setBrakeMode(boolean brakeMode) {}

  @Override
  public void rollersRunVolts(double power) {
    double volts = power * 12;
    this.intakeRollerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void rollersStop() {
    this.intakeRollerAppliedVolts = 0.0;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    this.intakeRollersSim.setInput(this.intakeRollerAppliedVolts);
    this.intakeRollersSim.update(0.020);
    
    this.intakePivotSim.setInput(this.intakePivotAppliedVolts);
    this.intakePivotSim.update(0.020);
    
    inputs.connected = true;
    inputs.intakeRollersVolts = this.intakeRollersSim.getInputVoltage();
    inputs.intakeRollersPosition = Double.POSITIVE_INFINITY;
    inputs.intakeRollersRps = this.intakeRollersSim.getAngularVelocityRPM() / 60;
    inputs.intakeRollersCurrent = this.intakeRollersSim.getCurrentDrawAmps();
    inputs.intakeRollersSupplyCurrent = this.intakeRollersSim.getCurrentDrawAmps();
    
    inputs.intakePivotVolts = this.intakePivotAppliedVolts;
    inputs.intakePivotPosition = this.getRotations().getRotations();
    inputs.intakePivotRps = Units.radiansPerSecondToRotationsPerMinute(this.intakePivotSim.getVelocityRadPerSec()) / 60;
    inputs.intakePivotCurrent = this.intakePivotSim.getCurrentDrawAmps();
    inputs.intakePivotSupplyCurrent = this.intakePivotSim.getCurrentDrawAmps();
    inputs.intakePivotErrorFromSetpoint = this.getIntakePivotPositionError();
    inputs.intakePivotSetpoint = this.intakePivotRotationSetPoint;
    inputs.intakePivotAtSetpoint = this.intakeAtPositionSetpoint();
  }
}
