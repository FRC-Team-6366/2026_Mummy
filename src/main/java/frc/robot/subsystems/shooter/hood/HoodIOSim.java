package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class HoodIOSim implements HoodIO {
  private ElevatorSim hoodSimLeft;
  private ElevatorSim hoodSimRight;
  private double hoodMotorLeftAppliedVoltage = 0.0;
  private double hoodMotorRightAppliedVoltage = 0.0;

  private ProfiledPIDController hoodPPID = new ProfiledPIDController(
    100.0, 
    0.0, 
    0.0, 
    new TrapezoidProfile.Constraints(
      0.5,
      0.25)
  );

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
  private double hoodLeftSetpointRotations = 0.0;
  private double hoodRightSetpointRotations = 0.0;
  private double setpointThreshold = (5.6 / 100) * 1;

  public HoodIOSim() {
    this.hoodSimRight = new ElevatorSim(
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

      this.hoodSimLeft = new ElevatorSim(
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
  public Rotation2d getRotationsRight() {
    return Rotation2d.fromRotations(
      this.metersToRotations(
        this.hoodSimRight.getPositionMeters()
      )
    );
  }

  @Override
  public Rotation2d getRotationsLeft() {
    return Rotation2d.fromRotations(
      this.metersToRotations(
        this.hoodSimLeft.getPositionMeters()
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
    this.hoodLeftSetpointRotations = this.metersToRotations(angleToMeters);
    this.hoodRightSetpointRotations = this.metersToRotations(angleToMeters);
    
    // TrapezoidProfile.State previousSetpoint = this.hoodPPID.getSetpoint();

    // double pidVoltsLeft= this.hoodPPID.calculate(this.hoodSimLeft.getPositionMeters(), angleToMeters);
    // double pidVoltsRight = this.hoodPPID.calculate(this.hoodSimRight.getPositionMeters(), angleToMeters);

    double pidVoltsLeft = this.hoodPID.calculate(this.hoodSimLeft.getPositionMeters(), angleToMeters);
    double pidVoltsRight = this.hoodPID.calculate(this.hoodSimRight.getPositionMeters(), angleToMeters);

    // double ffVolts = hoodFeedForward.calculate(
    //   previousSetpoint.velocity, // Old setpoint and old velocity
    //   this.hoodPPID.getSetpoint().velocity // New setpoint and new velocity
    // );

    double ffVolts = hoodFeedForward.calculate(0.0, 0.0);

    this.hoodMotorLeftAppliedVoltage = MathUtil.clamp(pidVoltsLeft + ffVolts, -12.0, 12.0);
    this.hoodMotorRightAppliedVoltage = MathUtil.clamp(pidVoltsRight + ffVolts, -12.0, 12.0);
  }

  @Override
  public void hoodToAngleLeft(double angle) {
    this.hoodLeftSetpointRotations = this.angleToMeters(angle);
    
    TrapezoidProfile.State previousSetpoint = this.hoodPPID.getSetpoint();
    double pidVoltsLeft= this.hoodPPID.calculate(this.hoodSimLeft.getPositionMeters(), this.hoodLeftSetpointRotations );

    double ffVolts = hoodFeedForward.calculate(
      previousSetpoint.velocity, // Old setpoint and old velocity
      this.hoodPPID.getSetpoint().velocity // New setpoint and new velocity
    );

    this.hoodMotorLeftAppliedVoltage = MathUtil.clamp(pidVoltsLeft + ffVolts, -12.0, 12.0);
  }

  @Override
  public void hoodToAngleRight(double angle) {
    this.hoodRightSetpointRotations = this.angleToMeters(angle);
    
    TrapezoidProfile.State previousSetpoint = this.hoodPPID.getSetpoint();
    double pidVoltsRight = this.hoodPPID.calculate(this.hoodSimRight.getPositionMeters(), this.hoodRightSetpointRotations);

    double ffVolts = hoodFeedForward.calculate(
      previousSetpoint.velocity, // Old setpoint and old velocity
      this.hoodPPID.getSetpoint().velocity // New setpoint and new velocity
    );

    this.hoodMotorRightAppliedVoltage = MathUtil.clamp(pidVoltsRight+ ffVolts, -12.0, 12.0);
  }

  @Override
  public double getHoodPositionErrorRight() {
    return this.hoodRightSetpointRotations - this.metersToRotations(this.hoodSimRight.getPositionMeters());
  }

  @Override
  public double getHoodPositionErrorLeft() {
    return this.hoodLeftSetpointRotations - this.metersToRotations(this.hoodSimLeft.getPositionMeters());
  }

  @Override
  public boolean hoodsAtPositionSetpoint() {
    return Math.abs(this.getHoodPositionErrorRight()) < this.setpointThreshold && Math.abs(this.getHoodPositionErrorLeft()) < this.setpointThreshold;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    this.hoodSimLeft.setInput(this.hoodMotorLeftAppliedVoltage);
    this.hoodSimLeft.update(0.020);
    this.hoodSimRight.setInput(this.hoodMotorRightAppliedVoltage);
    this.hoodSimRight.update(0.020);
    
    inputs.connected = true;
    inputs.hoodCurrentRight = this.hoodSimRight.getCurrentDrawAmps();
    inputs.hoodPositionRight = this.metersToRotations(this.hoodSimRight.getPositionMeters());
    inputs.hoodPositionErrorRight = this.getHoodPositionErrorRight();
    inputs.hoodPositionSetpointRight = this.hoodRightSetpointRotations;
    inputs.hoodRPSRight = this.metersToRotations(this.hoodSimRight.getVelocityMetersPerSecond());
    inputs.hoodSupplyCurrentRight = this.hoodSimRight.getCurrentDrawAmps();
    inputs.hoodVoltsRight = this.hoodMotorRightAppliedVoltage;
    inputs.hoodAtSetpointRight = this.hoodsAtPositionSetpoint();

    inputs.hoodCurrentLeft = this.hoodSimLeft.getCurrentDrawAmps();
    inputs.hoodPositionLeft = this.metersToRotations(this.hoodSimLeft.getPositionMeters());
    inputs.hoodPositionErrorLeft = this.getHoodPositionErrorLeft();
    inputs.hoodPositionSetpointLeft = this.hoodLeftSetpointRotations;
    inputs.hoodRPSLeft = this.metersToRotations(this.hoodSimLeft.getVelocityMetersPerSecond());
    inputs.hoodSupplyCurrentLeft = this.hoodSimLeft.getCurrentDrawAmps();
    inputs.hoodVoltsLeft = this.hoodMotorLeftAppliedVoltage;
    inputs.hoodAtSetpointLeft = this.hoodsAtPositionSetpoint();
  }
}
