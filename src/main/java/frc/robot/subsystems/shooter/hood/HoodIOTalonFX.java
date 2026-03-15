package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;

public class HoodIOTalonFX implements HoodIO {

  TalonFX hoodMotorRight;
  TalonFX hoodMotorLeft;

  // ---- Simulation objects (only used in sim) ----
  ElevatorSim hoodRightSim;
  ElevatorSim hoodLeftSim;
  TalonFXSimState hoodMotorRightSimState;
  TalonFXSimState hoodMotorLeftSimState;

  /**
   * Used to control motor output by specifying postiton setpoint
   * (number of rotations from start position)
   */
  PositionVoltage positionVoltageRequestRight;
  PositionVoltage positionVoltageRequestLeft;

  StatusSignal<Voltage> hoodVoltsRight;
  StatusSignal<Angle> hoodPositionRight;
  StatusSignal<AngularVelocity> hoodRPSRight;
  StatusSignal<Current> hoodCurrentRight;
  StatusSignal<Current> hoodSupplyCurrentRight;
  StatusSignal<Double> hoodErrorFromSetpointRight;

  StatusSignal<Voltage> hoodVoltsLeft;
  StatusSignal<Angle> hoodPositionLeft;
  StatusSignal<AngularVelocity> hoodRPSLeft;
  StatusSignal<Current> hoodCurrentLeft;
  StatusSignal<Current> hoodSupplyCurrentLeft;
  StatusSignal<Double> hoodErrorFromSetpointLeft;

  // Speficy min and max position for adjusting
  // other elements of the hood subsystem such as tolerance amount
  static final double hoodMinPosition = 0;
  static final double hoodMaxPosition = 5.6;

  // Setpoint tracking variables
  /**
   * How far the motor position can be off and still be concidered at setpoint.
   * Valid values from 0.0 (no tolerance!) to 100.0 (no accuracy!)
   */
  double setPointTolerancePercent = 1;
  double setPointTolerance;
  double positionSetPointLow;
  double positionSetPointHigh;

  public HoodIOTalonFX() {

    // |==============================|
    // | Common Hood Configuration    |
    // |==============================|

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    cfg.Slot0.kP = 1;
    cfg.Slot0.kI = 1;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5.6;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    // |==============================|
    // | Right Hand Shooter Assy      |
    // |==============================|

    // Instantiating Right Hood motor and its variables for monitoring
    hoodMotorRight = new TalonFX(ShooterConstants.hoodRightMotorId);
    hoodVoltsRight = hoodMotorRight.getMotorVoltage();
    hoodPositionRight = hoodMotorRight.getPosition();
    hoodRPSRight = hoodMotorRight.getVelocity();
    hoodCurrentRight = hoodMotorRight.getTorqueCurrent();
    hoodSupplyCurrentRight = hoodMotorRight.getSupplyCurrent();
    hoodErrorFromSetpointRight = hoodMotorRight.getClosedLoopError();

    // Apply configuration
    this.hoodMotorRight.getConfigurator().apply(cfg);

    // Set inital encoder value to 0
    // NOTE: Make sure hood is completely retracted with starting robot!
    hoodMotorRight.setPosition(0);

    // |==============================|
    // | Left Hand Shooter Assy      |
    // |==============================|

    // Instantiating Left Hood motor and its variables for monitoring
    hoodMotorLeft = new TalonFX(ShooterConstants.hoodLeftMotorId);
    hoodVoltsLeft = hoodMotorLeft.getMotorVoltage();
    hoodPositionLeft = hoodMotorLeft.getPosition();
    hoodRPSLeft = hoodMotorLeft.getVelocity();
    hoodCurrentLeft = hoodMotorLeft.getTorqueCurrent();
    hoodSupplyCurrentLeft = hoodMotorLeft.getSupplyCurrent();
    hoodErrorFromSetpointLeft = hoodMotorLeft.getClosedLoopError();

    // Apply configuration
    this.hoodMotorLeft.getConfigurator().apply(cfg);

    // Set inital encoder value to 0
    // NOTE: Make sure hood is completely retracted with starting robot!
    hoodMotorLeft.setPosition(0);

    // |==============================|
    // | Status Signal Updates        |
    // |==============================|

    // Set update period for device metrics to be 50 Hz (20 milliseconds)
    BaseStatusSignal.setUpdateFrequencyForAll(
      50,
      hoodVoltsRight,
      hoodPositionRight,
      hoodRPSRight,
      hoodCurrentRight,
      hoodSupplyCurrentRight,
      hoodErrorFromSetpointRight,
      hoodVoltsLeft,
      hoodPositionLeft,
      hoodRPSLeft,
      hoodCurrentLeft,
      hoodSupplyCurrentLeft,
      hoodErrorFromSetpointLeft
    );

    hoodMotorRight.optimizeBusUtilization(0.0, 1.0);
    hoodMotorLeft.optimizeBusUtilization(0.0, 1.0);

    // Instantiating position voltage object for setting the output position
    positionVoltageRequestRight = new PositionVoltage(0);
    hoodMotorRight.setControl(positionVoltageRequestRight.withSlot(0));

    positionVoltageRequestLeft = new PositionVoltage(0);
    hoodMotorLeft.setControl(positionVoltageRequestLeft.withSlot(0));

    // Compute setpoint tolerance from max position and tolerance percent
    this.setPointTolerance = hoodMaxPosition * this.setPointTolerancePercent / 100;

    if (Robot.isSimulation()) {
      // Instantiate the elevator sim object that best simulates the hood system
      this.hoodRightSim = new ElevatorSim(
        DCMotor.getKrakenX60(1), 
        1, 
        0.05, 
        0.01, 
        0, 
        0.127, // 5 inches to meters
        false, 
        0, 
        0.004
      );

      this.hoodLeftSim = new ElevatorSim(
        DCMotor.getKrakenX60(1), 
        1, 
        0.05, 
        0.01, 
        0, 
        0.127, // inch to meters
        false, 
        0, 
        0.004
      );

      // Instantiate the motor simulated states for reporting to simulators
      this.hoodMotorRightSimState = this.hoodMotorRight.getSimState();
      this.hoodMotorLeftSimState = this.hoodMotorLeft.getSimState();
    }
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public Rotation2d getRotationsRight() {
    return new Rotation2d(Units.rotationsToRadians(hoodMotorRight.getPosition().getValueAsDouble()));
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public Rotation2d getRotationsLeft() {
    return new Rotation2d(Units.rotationsToRadians(hoodMotorLeft.getPosition().getValueAsDouble()));
  }

  /**
   * Sets the hood to the specified angle by converting it
   * to number of rotations for the motor
   * 
   * @param angle
   *          Angle in degrees. From 0 to 45 max
   */
  @Override
  public void hoodsToAngle(double angle) {
    // We are trying to map the degrees 0 -> 45 to the motor rotations 0 -> 5.6
    // We map angle to an input value between 0 and 45 to prevent larger or smaller
    // values
    // Then divide the angle by the ratio between the max angle (45) and the max
    // postion in rotations (5.6)
    double angletoRotations = (MathUtil.clamp(angle, 15.0, 45.0) - 15.0) / (30.0 / hoodMaxPosition);
    this.hoodMotorLeft.setControl(positionVoltageRequestLeft.withPosition(angletoRotations));
    this.hoodMotorRight.setControl(positionVoltageRequestRight.withPosition(angletoRotations));
  }

  @Override
  public void hoodToAngleLeft(double angle) {
    // We are trying to map the degrees 0 -> 45 to the motor rotations 0 -> 5.6
    // We map angle to an input value between 0 and 45 to prevent larger or smaller
    // values
    // Then divide the angle by the ratio between the max angle (45) and the max
    // postion in rotations (5.6)
    double angletoRotations = (MathUtil.clamp(angle, 15.0, 45.0) - 15.0) / (30.0 / hoodMaxPosition);
    this.hoodMotorLeft.setControl(positionVoltageRequestLeft.withPosition(angletoRotations));
  }

  @Override
    public void hoodToAngleRight(double angle) {
      // We are trying to map the degrees 0 -> 45 to the motor rotations 0 -> 5.6
      // We map angle to an input value between 0 and 45 to prevent larger or smaller
      // values
      // Then divide the angle by the ratio between the max angle (45) and the max
      // postion in rotations (5.6)
      double angletoRotations = (MathUtil.clamp(angle, 15.0, 45.0) - 15.0) / (30.0 / hoodMaxPosition);
      this.hoodMotorRight.setControl(positionVoltageRequestRight.withPosition(angletoRotations));
    }

  @Override
  public double getHoodPositionErrorRight() {
    return this.hoodMotorRight.getClosedLoopError().getValueAsDouble();
  }

  @Override
  public double getHoodPositionErrorLeft() {
    return this.hoodMotorLeft.getClosedLoopError().getValueAsDouble();
  }

  @Override
  public boolean hoodsAtPositionSetpoint() {
    // Get absolute value of the error and see if it is less
    // than the setpoint tolerance for both shooter assemblies
    return (Math.abs(this.getHoodPositionErrorRight()) < this.setPointTolerance) && (Math.abs(this.getHoodPositionErrorLeft()) < this.setPointTolerance);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (Robot.isSimulation()) {
      // 1. Feed current battery voltage to sim states
      this.hoodMotorRightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      this.hoodMotorLeftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

      // 2. Read the voltage the hood motors are actually applying
      double hoodMotorRightVoltage = this.hoodMotorRightSimState.getMotorVoltage();
      double hoodMotorLeftVoltage = this.hoodMotorLeftSimState.getMotorVoltage();

      // 3. Update the physics model
      this.hoodRightSim.setInput(hoodMotorRightVoltage);
      this.hoodRightSim.update(0.020);
      this.hoodLeftSim.setInput(hoodMotorLeftVoltage);
      this.hoodLeftSim.update(0.020);

      // 4. Write simulated velocity/position back into the TalonFX sim states
      double hoodMotorRightRPS = hoodRightSim.getVelocityMetersPerSecond();
      double hoodMotorLeftRPS = hoodLeftSim.getVelocityMetersPerSecond();

      this.hoodMotorRightSimState.setRotorVelocity(hoodMotorRightRPS);
      this.hoodMotorLeftSimState.setRotorVelocity(hoodMotorLeftRPS);

      this.hoodMotorRightSimState.addRotorPosition(hoodMotorRightRPS * 0.020);
      this.hoodMotorLeftSimState.addRotorPosition(hoodMotorLeftRPS * 0.020);
    }
    
    inputs.connected = BaseStatusSignal.refreshAll(
      hoodVoltsRight,
      hoodPositionRight,
      hoodRPSRight,
      hoodCurrentRight,
      hoodSupplyCurrentRight,
      hoodErrorFromSetpointRight,
      hoodVoltsLeft,
      hoodPositionLeft,
      hoodRPSLeft,
      hoodCurrentLeft,
      hoodSupplyCurrentLeft,
      hoodErrorFromSetpointLeft).isOK();

    // Update Hardware fields
    inputs.hoodVoltsRight = this.hoodVoltsRight.getValueAsDouble();
    inputs.hoodPositionRight = this.hoodPositionRight.getValueAsDouble();
    inputs.hoodRPSRight = this.hoodRPSRight.getValueAsDouble();
    inputs.hoodCurrentRight = this.hoodCurrentRight.getValueAsDouble();
    inputs.hoodSupplyCurrentRight = this.hoodSupplyCurrentRight.getValueAsDouble();

    inputs.hoodVoltsLeft = this.hoodVoltsLeft.getValueAsDouble();
    inputs.hoodPositionLeft = this.hoodPositionLeft.getValueAsDouble();
    inputs.hoodRPSLeft = this.hoodRPSLeft.getValueAsDouble();
    inputs.hoodCurrentLeft = this.hoodCurrentLeft.getValueAsDouble();
    inputs.hoodSupplyCurrentLeft = this.hoodSupplyCurrentLeft.getValueAsDouble();

    // Upodate Setpoint related fields
    inputs.hoodPositionSetpointRight = this.positionVoltageRequestRight.Position;
    inputs.hoodPositionErrorRight = this.hoodErrorFromSetpointRight.getValueAsDouble();
    inputs.hoodAtSetpointRight = this.hoodsAtPositionSetpoint();

    inputs.hoodPositionSetpointLeft = this.positionVoltageRequestLeft.Position;
    inputs.hoodPositionErrorLeft = this.hoodErrorFromSetpointLeft.getValueAsDouble();
    inputs.hoodAtSetpointLeft = this.hoodsAtPositionSetpoint();
  }

}
