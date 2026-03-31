package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  FlywheelSim leadShooterMotor;
  DCMotor LEAD_SHOOTER_GEARBOX = DCMotor.getKrakenX60(1);
  public static final double shooterMinVelocityRPS = 0;
  public static final double shooterMaxVelocityRPS = 100;
  private double rps = 0.0;
  private double radPerSeconds = 0.0;
  private double setpointThreshold = 1;

  public ShooterIOSim() {
    this.leadShooterMotor = new FlywheelSim(LinearSystemId.createFlywheelSystem(LEAD_SHOOTER_GEARBOX, 12, 1),
        LEAD_SHOOTER_GEARBOX, 0.004);
  }

  @Override
  public void setShooterVelocityFeetPerSecond(double feetPerSecond) {
    double rotationsPerSecond = feetPerSecond / ((4.0 / 12.0) * Math.PI);
    this.rps = MathUtil.clamp(rotationsPerSecond, shooterMinVelocityRPS, shooterMaxVelocityRPS);
    this.radPerSeconds = this.rps * 2 * Math.PI;
    this.leadShooterMotor.setAngularVelocity(radPerSeconds);
  }

  @Override
  public double getRightShooterVelocityError() {
    return this.rps - this.leadShooterMotor.getAngularVelocityRPM() / 60;
  }

  @Override
  public double getLeftShooterVelocityError() {
    return this.rps - this.leadShooterMotor.getAngularVelocityRPM() / 60;
  }

  @Override
  public boolean rightShooterAtVelocitySetPoint() {
    return Math.abs(this.getRightShooterVelocityError()) < this.setpointThreshold;
  }

   @Override
  public boolean leftShooterAtVelocitySetPoint() {
    return Math.abs(this.getRightShooterVelocityError()) < this.setpointThreshold;
  }

  @Override
  public boolean detectShot() {
    return false;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.connected = true;
    inputs.rightLeadShooterCurrent = this.leadShooterMotor.getCurrentDrawAmps();
    inputs.rightLeadShooterPosition = Double.POSITIVE_INFINITY;
    inputs.rightLeadShooterRps = this.leadShooterMotor.getAngularVelocityRPM() / 60;
    inputs.rightLeadShooterSupplyCurrent = this.leadShooterMotor.getCurrentDrawAmps();
    inputs.rightLeadShooterVolts = this.leadShooterMotor.getInputVoltage();
    inputs.rightFollowShooterCurrent = this.leadShooterMotor.getCurrentDrawAmps();
    inputs.rightFollowShooterPosition = Double.POSITIVE_INFINITY;
    inputs.rightFollowShooterRps = this.leadShooterMotor.getAngularVelocityRPM() / 60;
    inputs.rightFollowShooterSupplyCurrent = this.leadShooterMotor.getCurrentDrawAmps();
    inputs.rightFollowShooterVolts = this.leadShooterMotor.getInputVoltage();
    inputs.rightShooterVelocitySetpoint = this.leadShooterMotor.getAngularVelocityRPM();
    inputs.rightShooterVelocityError = this.getRightShooterVelocityError();
    inputs.rightShooterAtVelocitySetpoint = this.rightShooterAtVelocitySetPoint();

    inputs.leftLeadShooterCurrent = this.leadShooterMotor.getCurrentDrawAmps();
    inputs.leftLeadShooterPosition = Double.POSITIVE_INFINITY;
    inputs.leftLeadShooterRps = this.leadShooterMotor.getAngularVelocityRPM() / 60;
    inputs.leftLeadShooterSupplyCurrent = this.leadShooterMotor.getCurrentDrawAmps();
    inputs.leftLeadShooterVolts = this.leadShooterMotor.getInputVoltage();
    inputs.leftFollowShooterCurrent = this.leadShooterMotor.getCurrentDrawAmps();
    inputs.leftFollowShooterPosition = Double.POSITIVE_INFINITY;
    inputs.leftFollowShooterRps = this.leadShooterMotor.getAngularVelocityRPM() / 60;
    inputs.leftFollowShooterSupplyCurrent = this.leadShooterMotor.getCurrentDrawAmps();
    inputs.leftFollowShooterVolts = this.leadShooterMotor.getInputVoltage();
    inputs.leftShooterVelocitySetpoint = this.leadShooterMotor.getAngularVelocityRPM();
    inputs.leftShooterVelocityError = this.getLeftShooterVelocityError();
    inputs.leftShooterAtVelocitySetpoint = this.leftShooterAtVelocitySetPoint();
  }

}
