package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim implements HoodIO {
  private DCMotorSim hoodMotorRight;
  private DCMotorSim hoodMotorLeft;
  private DCMotor HOOD_GEARBOX = DCMotor.getKrakenX60(1);
  static final double hoodMinPosition = 0;
  static final double hoodMaxPosition = 5.6;
  private double angleRad = 0.0;
  private double setpointThreshold = (5.6 / 100) * 2;

  public HoodIOSim() {
    this.hoodMotorRight = new DCMotorSim(LinearSystemId.createDCMotorSystem(HOOD_GEARBOX, 1, 1), HOOD_GEARBOX, 0.004, 0.04);
    this.hoodMotorRight.setAngle(Units.degreesToRadians(15));

    this.hoodMotorLeft = new DCMotorSim(LinearSystemId.createDCMotorSystem(HOOD_GEARBOX, 1, 1), HOOD_GEARBOX, 0.004, 0.04);
    this.hoodMotorLeft.setAngle(Units.degreesToRadians(15));
  }

  @Override
  public Rotation2d getRotationsRight() {
    return Rotation2d.fromRadians(this.hoodMotorRight.getAngularPositionRad());
  }

  @Override
  public Rotation2d getRotationsLeft() {
    return Rotation2d.fromRadians(this.hoodMotorLeft.getAngularPositionRad());
  }

  @Override
  public void hoodsToAngle(double angle) {
    this.angleRad = angle * 2 * Math.PI;
    this.hoodMotorRight.setAngle(this.angleRad);
    this.hoodMotorLeft.setAngle(this.angleRad);
  }

  @Override
  public void hoodToAngleLeft(double angle) {
    this.angleRad = angle * 2 * Math.PI;
    this.hoodMotorLeft.setAngle(this.angleRad);
  }

  @Override
  public void hoodToAngleRight(double angle) {
    this.angleRad = angle * 2 * Math.PI;
    this.hoodMotorRight.setAngle(this.angleRad);
  }

  @Override
  public double getHoodPositionErrorRight() {
    return this.angleRad - this.hoodMotorRight.getAngularPositionRad();
  }

  @Override
  public double getHoodPositionErrorLeft() {
    return this.angleRad - this.hoodMotorLeft.getAngularPositionRad();
  }

  @Override
  public boolean hoodsAtPositionSetpoint() {
    return Math.abs(this.getHoodPositionErrorRight()) < this.setpointThreshold && Math.abs(this.getHoodPositionErrorLeft()) < this.setpointThreshold;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.connected = true;
    inputs.hoodCurrentRight = this.hoodMotorRight.getCurrentDrawAmps();
    inputs.hoodPositionRight = this.hoodMotorRight.getAngularPositionRotations();
    inputs.hoodPositionErrorRight = this.getHoodPositionErrorRight();
    inputs.hoodPositionSetpointRight = Units.radiansToRotations(angleRad);
    inputs.hoodRPSRight = this.hoodMotorRight.getAngularVelocityRPM() / 60;
    inputs.hoodSupplyCurrentRight = this.hoodMotorRight.getCurrentDrawAmps();
    inputs.hoodVoltsRight = this.hoodMotorRight.getInputVoltage();
    inputs.hoodAtSetpointRight = this.hoodsAtPositionSetpoint();

    inputs.hoodCurrentLeft = this.hoodMotorLeft.getCurrentDrawAmps();
    inputs.hoodPositionLeft = this.hoodMotorLeft.getAngularPositionRotations();
    inputs.hoodPositionErrorLeft = this.getHoodPositionErrorLeft();
    inputs.hoodPositionSetpointLeft = Units.radiansToRotations(angleRad);
    inputs.hoodRPSLeft = this.hoodMotorLeft.getAngularVelocityRPM() / 60;
    inputs.hoodSupplyCurrentLeft = this.hoodMotorLeft.getCurrentDrawAmps();
    inputs.hoodVoltsLeft = this.hoodMotorLeft.getInputVoltage();
    inputs.hoodAtSetpointLeft = this.hoodsAtPositionSetpoint();
  }
}
