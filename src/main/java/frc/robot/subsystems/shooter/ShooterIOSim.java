package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  FlywheelSim rightFlyWheelSim;
  FlywheelSim leftFlyWheelSim;
  DCMotor SHOOTER_GEARBOX = DCMotor.getKrakenX60(2);

  SimpleMotorFeedforward rightFeedforward = new SimpleMotorFeedforward(
    0.0, 
    0.12, 
    0.0
    );

  SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(
    0.0, 
    0.12, 
    0.0
    );

  double rightFlywheelAppliedVoltage = 0.0;
  double leftFlywheelAppliedVoltage = 0.0;

  public static final double shooterMinVelocityRPS = 0;
  public static final double shooterMaxVelocityRPS = 100;
  private double rps = 0.0;
  private double radPerSeconds = 0.0;
  private double setpointThreshold = 1;

  public ShooterIOSim() {
    this.rightFlyWheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        SHOOTER_GEARBOX, 
        0.001, 
        1
      ),
      SHOOTER_GEARBOX, 
      0.004
    );
    
    this.leftFlyWheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        SHOOTER_GEARBOX, 
        0.001, 
        1
      ),
      SHOOTER_GEARBOX, 
      0.004
    );
  }

  @Override
  public void setShooterVelocityFeetPerSecond(double feetPerSecond) {
    double rotationsPerSecond = feetPerSecond / ((4.0 / 12.0) * Math.PI);
    this.rps = MathUtil.clamp(rotationsPerSecond, shooterMinVelocityRPS, shooterMaxVelocityRPS);
    this.radPerSeconds = this.rps * 2 * Math.PI;
    
    // Using Feed Forward models to convert rotations to voltage
    // and then setting applied voltage to be used in updateInputs()
    double rightVoltage = this.rightFeedforward.calculate(radPerSeconds);
    this.rightFlywheelAppliedVoltage = MathUtil.clamp(rightVoltage, -12.0, 12.0);

    double leftVoltage = this.leftFeedforward.calculate(radPerSeconds);
    this.leftFlywheelAppliedVoltage = MathUtil.clamp(leftVoltage, -12.0, 12.0);
    
  }

  @Override
  public double getRightShooterVelocityError() {
    return this.rps - this.rightFlyWheelSim.getAngularVelocityRPM() / 60;
  }

  @Override
  public double getLeftShooterVelocityError() {
    return this.rps - this.rightFlyWheelSim.getAngularVelocityRPM() / 60;
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
  public void updateInputs(ShooterIOInputs inputs) {
    // Update the applied voltage to the motors and 
    this.rightFlyWheelSim.setInput(this.rightFlywheelAppliedVoltage);
    this.leftFlyWheelSim.setInput(this.leftFlywheelAppliedVoltage);
    this.rightFlyWheelSim.update(0.020);
    this.leftFlyWheelSim.update(0.020);
    
    inputs.connected = true;
    inputs.rightLeadShooterCurrent = this.rightFlyWheelSim.getCurrentDrawAmps();
    inputs.rightLeadShooterRps = this.rightFlyWheelSim.getAngularVelocityRPM() / 60;
    inputs.rightLeadShooterSupplyCurrent = this.rightFlyWheelSim.getCurrentDrawAmps();
    inputs.rightLeadShooterVolts = this.rightFlyWheelSim.getInputVoltage();
    inputs.rightFollowShooterCurrent = this.rightFlyWheelSim.getCurrentDrawAmps();
    inputs.rightFollowShooterRps = this.rightFlyWheelSim.getAngularVelocityRPM() / 60;
    inputs.rightFollowShooterSupplyCurrent = this.rightFlyWheelSim.getCurrentDrawAmps();
    inputs.rightFollowShooterVolts = this.rightFlyWheelSim.getInputVoltage();
    inputs.rightShooterVelocitySetpoint = this.rightFlyWheelSim.getAngularVelocityRPM();
    inputs.rightShooterVelocityError = this.getRightShooterVelocityError();
    inputs.rightShooterAtVelocitySetpoint = this.rightShooterAtVelocitySetPoint();

    inputs.leftLeadShooterCurrent = this.leftFlyWheelSim.getCurrentDrawAmps();
    inputs.leftLeadShooterRps = this.leftFlyWheelSim.getAngularVelocityRPM() / 60;
    inputs.leftLeadShooterSupplyCurrent = this.leftFlyWheelSim.getCurrentDrawAmps();
    inputs.leftLeadShooterVolts = this.leftFlyWheelSim.getInputVoltage();
    inputs.leftFollowShooterCurrent = this.leftFlyWheelSim.getCurrentDrawAmps();
    inputs.leftFollowShooterRps = this.leftFlyWheelSim.getAngularVelocityRPM() / 60;
    inputs.leftFollowShooterSupplyCurrent = this.leftFlyWheelSim.getCurrentDrawAmps();
    inputs.leftFollowShooterVolts = this.leftFlyWheelSim.getInputVoltage();
    inputs.leftShooterVelocitySetpoint = this.leftFlyWheelSim.getAngularVelocityRPM();
    inputs.leftShooterVelocityError = this.getLeftShooterVelocityError();
    inputs.leftShooterAtVelocitySetpoint = this.leftShooterAtVelocitySetPoint();
  }

}
