package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class HoodIOSim implements HoodIO {
  private ElevatorSim hoodSim;
  private double hoodMotorRightAppliedVoltage = 0.0;

  private PIDController hoodPID = new PIDController(
    100.0, 
    0.0, 
    0.0, 
    0.020
  );

  private SimpleMotorFeedforward hoodFeedForward = new SimpleMotorFeedforward(
    0.0, 
    0.12, 
    0.0
  );
  private DCMotor HOOD_GEARBOX = DCMotor.getKrakenX60(1);
  static final double hoodMinPosition = 0;
  static final double hoodMaxPosition = 5.6;
  private double hoodRightSetpointRotations = 0.0;
  private double setpointThreshold = (5.6 / 100) * 1;

  public HoodIOSim() {
    this.hoodSim = new ElevatorSim(
        LinearSystemId.createElevatorSystem(
          HOOD_GEARBOX,
          0.01,
          0.01,
          3
        ), 
        HOOD_GEARBOX, 
        0.0, 
        0.127, 
        false, 
        0.0, 
        0.0001, 0.0004
      );

  }

  @Override
  public Rotation2d getRotations() {
    return Rotation2d.fromRotations(
      this.metersToRotations(
        this.hoodSim.getPositionMeters()
      )
    );
  }



  private double angleToMeters(double angle) {
    // 1. Convert Angle to rotations
    double angletoRotations = (MathUtil.clamp(angle, 15.0, 45.0) - 15.0) / ((45.0 - 15.0) / hoodMaxPosition);
    
    // 2. Get inches per rotation
    double inchesPerRotation = 5.0 / 5.6; //5 inches for 5.6 rotations = 0.8928 inches per rotation
    double metersPerRotation = Units.inchesToMeters(inchesPerRotation);
    
    // 3. Return rotations * meter per rotations
    return angletoRotations * metersPerRotation;
  }

  private double metersToRotations(double meters) {
    double rotationsPerInch = 5.6 / 5.0;
    double rotationsPerMeter = Units.inchesToMeters(rotationsPerInch);
    return meters / rotationsPerMeter;
  }

  @Override
  public void hoodsToAngle(double angle) {
    double angleToMeters = this.angleToMeters(angle);
    this.hoodRightSetpointRotations = this.metersToRotations(angleToMeters);

    double pidVoltsRight = this.hoodPID.calculate(this.hoodSim.getPositionMeters(), angleToMeters);

    double ffVolts = hoodFeedForward.calculate(0.0, 0.0);

    this.hoodMotorRightAppliedVoltage = MathUtil.clamp(pidVoltsRight + ffVolts, -12.0, 12.0);
  }


  @Override
  public double getHoodPositionError() {
    return this.hoodRightSetpointRotations - this.metersToRotations(this.hoodSim.getPositionMeters());
  }

  @Override
  public boolean hoodsAtPositionSetpoint() {
    return Math.abs(this.getHoodPositionError()) < this.setpointThreshold;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    this.hoodSim.setInput(this.hoodMotorRightAppliedVoltage);
    this.hoodSim.update(0.020);
    
    inputs.connected = true;
    inputs.hoodCurrentRight = this.hoodSim.getCurrentDrawAmps();
    inputs.hoodPositionRight = this.metersToRotations(this.hoodSim.getPositionMeters());
    inputs.hoodPositionErrorRight = this.getHoodPositionError();
    inputs.hoodPositionSetpointRight = this.hoodRightSetpointRotations;
    inputs.hoodRPSRight = this.metersToRotations(this.hoodSim.getVelocityMetersPerSecond());
    inputs.hoodSupplyCurrentRight = this.hoodSim.getCurrentDrawAmps();
    inputs.hoodVoltsRight = this.hoodMotorRightAppliedVoltage;
    inputs.hoodAtSetpointRight = this.hoodsAtPositionSetpoint();
  }

  @Override
  public double getHoodPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getHoodPosition'");
  }
}
