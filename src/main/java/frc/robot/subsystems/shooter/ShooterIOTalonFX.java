package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOTalonFX implements ShooterIO {
  protected TalonFX rightLeadShooterMotor;
  protected TalonFX rightFollowerShooterMotor;

  protected TalonFX leftLeadShooterMotor;
  protected TalonFX leftFollowerShooterMotor;

  // Used to control motor output by specifying rotaion speed
  VelocityVoltage velocityVoltageRequest;
  // Used to make followShooterMotor mimic the leadShooterMotor
  Follower rightFollower;
  Follower leftFollower;

  // ---- Simulation objects (only used in sim) ----
  FlywheelSim rightFlywheelSim;
  FlywheelSim leftFlywheelSim;
  TalonFXSimState rightLeadShooterMotorSimState;
  TalonFXSimState rightFollowerShooterMotorSimState;
  TalonFXSimState leftLeadShooterMotorSimState;
  TalonFXSimState leftFollowerShooterMotorSimState;

  StatusSignal<Voltage> rightLeadShooterVolts;
  StatusSignal<Angle> rightLeadShooterPosition;
  StatusSignal<AngularVelocity> rightLeadShooterRps;
  StatusSignal<Current> rightLeadShooterCurrent;
  StatusSignal<Current> rightLeadShooterSupplyCurrent;

  StatusSignal<Voltage> rightFollowerShooterVolts;
  StatusSignal<Angle> rightFollowerShooterPosition;
  StatusSignal<AngularVelocity> rightFollowerShooterRps;
  StatusSignal<Current> rightFollowerShooterCurrent;
  StatusSignal<Current> rightFollowerShooterSupplyCurrent;
  StatusSignal<Double> rightShooterVelocityError;

  // setting values for the second shooter...
  StatusSignal<Voltage> leftLeadShooterVolts;
  StatusSignal<Angle> leftLeadShooterPosition;
  StatusSignal<AngularVelocity> leftLeadShooterRps;
  StatusSignal<Current> leftLeadShooterCurrent;
  StatusSignal<Current> leftLeadShooterSupplyCurrent;

  StatusSignal<Voltage> leftFollowerShooterVolts;
  StatusSignal<Angle> leftFollowerShooterPosition;
  StatusSignal<AngularVelocity> leftFollowerShooterRps;
  StatusSignal<Current> leftFollowerShooterCurrent;
  StatusSignal<Current> leftFollowerShooterSupplyCurrent;
  StatusSignal<Double> leftShooterVelocityError;


  // Speficy min and max velocity for adjusting
  // other elements of the shooter subsystem such as tolerance amount
  public static final double shooterMinVelocityRPS = 0;
  public static final double shooterMaxVelocityRPS = 100;

  // Setpoint tracking variables
  /**
   * How far the shooter's velocity can be off and still be concidered at
   * setpoint.
   * Valid values from 0.0 (no tolerance!) to 100.0 (no accuracy!)
   */
  double setPointTolerancePercent = 1;
  double setPointTolerance;
  double velocitySetPointLow;
  double velocitySetPointHigh;

  public ShooterIOTalonFX() {
    // Instantiating Lead Shooter motor and its variables for monitoring
    rightLeadShooterMotor = new TalonFX(Constants.ShooterConstants.rightLeadShooterMotorId);
    rightLeadShooterVolts = rightLeadShooterMotor.getMotorVoltage();
    rightLeadShooterPosition = rightLeadShooterMotor.getPosition();
    rightLeadShooterRps = rightLeadShooterMotor.getVelocity();
    rightLeadShooterCurrent = rightLeadShooterMotor.getTorqueCurrent();
    rightLeadShooterSupplyCurrent = rightLeadShooterMotor.getSupplyCurrent();
    rightShooterVelocityError = rightLeadShooterMotor.getClosedLoopError();

    leftLeadShooterMotor = new TalonFX(Constants.ShooterConstants.leftLeadShooterMotorId);
    leftLeadShooterVolts = leftLeadShooterMotor.getMotorVoltage();
    leftLeadShooterPosition = leftLeadShooterMotor.getPosition();
    leftLeadShooterRps = leftLeadShooterMotor.getVelocity();
    leftLeadShooterCurrent = leftLeadShooterMotor.getTorqueCurrent();
    leftLeadShooterSupplyCurrent = leftLeadShooterMotor.getSupplyCurrent();
    leftShooterVelocityError = leftLeadShooterMotor.getClosedLoopError();

    // Instantiating configuration for Lead Shooter motor
    TalonFXConfiguration leadcfg = new TalonFXConfiguration();
    leadcfg.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    leadcfg.Slot0.kP = 0.05;
    leadcfg.Slot0.kV = 0.1166;
    leadcfg.Slot0.kD = 0;
    this.rightLeadShooterMotor.getConfigurator().apply(leadcfg);
    this.leftLeadShooterMotor.getConfigurator().apply(leadcfg);

    // Instantiating velocity voltage object for setting the output velocity
    this.velocityVoltageRequest = new VelocityVoltage(0);
    this.rightLeadShooterMotor.setControl(velocityVoltageRequest.withSlot(0));
    this.leftLeadShooterMotor.setControl(velocityVoltageRequest.withSlot(0));

    // Setting the StatusSignal variables to be mapped to actual
    // aspect of the ShooterIO's hardware
    rightFollowerShooterMotor = new TalonFX(Constants.ShooterConstants.rightFollowerShooterMotorId
    );
    rightFollowerShooterVolts = rightFollowerShooterMotor.getMotorVoltage();
    rightFollowerShooterPosition = rightFollowerShooterMotor.getPosition();
    rightFollowerShooterRps = rightFollowerShooterMotor.getVelocity();
    rightFollowerShooterCurrent = rightFollowerShooterMotor.getTorqueCurrent();
    rightFollowerShooterSupplyCurrent = rightFollowerShooterMotor.getSupplyCurrent();
    rightFollower = new Follower(Constants.ShooterConstants.rightLeadShooterMotorId, MotorAlignmentValue.Opposed);
    this.rightFollowerShooterMotor.setControl(rightFollower);

    leftFollowerShooterMotor = new TalonFX(Constants.ShooterConstants.leftFollowerShooterMotorId
    );
    leftFollowerShooterVolts = leftFollowerShooterMotor.getMotorVoltage();
    leftFollowerShooterPosition = leftFollowerShooterMotor.getPosition();
    leftFollowerShooterRps = leftFollowerShooterMotor.getVelocity();
    leftFollowerShooterCurrent = leftFollowerShooterMotor.getTorqueCurrent();
    leftFollowerShooterSupplyCurrent = leftFollowerShooterMotor.getSupplyCurrent();
    leftFollower = new Follower(Constants.ShooterConstants.leftLeadShooterMotorId, MotorAlignmentValue.Opposed);
    this.leftFollowerShooterMotor.setControl(leftFollower);    

    // Set update period for device metrics to be 50 Hz (20 milliseconds)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        rightLeadShooterVolts,
        rightLeadShooterPosition,
        rightLeadShooterRps,
        rightLeadShooterCurrent,
        rightLeadShooterSupplyCurrent,
        rightFollowerShooterVolts,
        rightFollowerShooterPosition,
        rightFollowerShooterRps,
        rightFollowerShooterCurrent,
        rightFollowerShooterSupplyCurrent,
        rightShooterVelocityError,
        leftLeadShooterVolts,
        leftLeadShooterPosition,
        leftLeadShooterRps,
        leftLeadShooterCurrent,
        leftLeadShooterSupplyCurrent,
        leftFollowerShooterVolts,
        leftFollowerShooterPosition,
        leftFollowerShooterRps,
        leftFollowerShooterCurrent,
        leftFollowerShooterSupplyCurrent,
        leftShooterVelocityError);

    // Forcing optimal use of the CAN Bus for this subsystems
    // hardware
    rightLeadShooterMotor.optimizeBusUtilization(0.0, 1.0);
    rightFollowerShooterMotor.optimizeBusUtilization(0.0, 1.0);

    leftLeadShooterMotor.optimizeBusUtilization(0.0, 1.0);
    leftFollowerShooterMotor.optimizeBusUtilization(0.0, 1.0);

    if (Robot.isSimulation()) {
      // Instantiate the flywheel object that simulates the shooter wheel
      this.rightFlywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(2), 
          0.001, 
          1.0
          ), 
        DCMotor.getKrakenX60(2), 
        0.004
      );

      this.leftFlywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(2), 
          0.001, 
          1.0
          ), 
        DCMotor.getKrakenX60(2), 
        0.004
      );
      
      // Instantiate the motor simulated states for reporting to simulators
      this.rightLeadShooterMotorSimState = this.rightLeadShooterMotor.getSimState();
      this.rightFollowerShooterMotorSimState = this.rightFollowerShooterMotor.getSimState();
      this.leftLeadShooterMotorSimState = this.leftLeadShooterMotor.getSimState();
      this.leftFollowerShooterMotorSimState = this.leftFollowerShooterMotor.getSimState();
    }
    
    this.setPointTolerance = shooterMaxVelocityRPS * this.setPointTolerancePercent;
  }

  @Override
  public void setShooterVelocityFeetPerSecond(double feetPerSecond) {
    double rotationsPerSecond = feetPerSecond / ((4.0 / 12.0) * Math.PI);
    double rpsToUse = MathUtil.clamp(rotationsPerSecond, shooterMinVelocityRPS, shooterMaxVelocityRPS);

    this.rightLeadShooterMotor.setControl(velocityVoltageRequest.withVelocity(rpsToUse));
    this.rightFollowerShooterMotor.setControl(this.rightFollower);
    this.leftLeadShooterMotor.setControl(velocityVoltageRequest.withVelocity(rpsToUse));
    this.leftFollowerShooterMotor.setControl(this.leftFollower);
  }

  @Override
  public double getRightShooterVelocityError() {
    return this.rightLeadShooterMotor.getClosedLoopError().getValueAsDouble();
  }


  @Override
  public double getLeftShooterVelocityError() {
    return this.leftLeadShooterMotor.getClosedLoopError().getValueAsDouble();
  }

  @Override
  public boolean rightShooterAtVelocitySetPoint() {
    // Get absolute value of the error and see if it is less
    // than the setpoint tolerance
    return Math.abs(this.getRightShooterVelocityError()) < this.setPointTolerance;
  }
  
  @Override
  public boolean leftShooterAtVelocitySetPoint() {
    // Get absolute value of the error and see if it is less
    // than the setpoint tolerance
    return Math.abs(this.getLeftShooterVelocityError()) < this.setPointTolerance;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (Robot.isSimulation()) {
      // 1. Feed current battery voltage to sim states
      this.rightLeadShooterMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      this.rightFollowerShooterMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      this.leftLeadShooterMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      this.leftFollowerShooterMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

      // 2. Read the voltage the leader motor is actually applying
      double rightLeadShooterMotorVoltage = this.rightLeadShooterMotorSimState.getMotorVoltage();
      double leftLeadShooterMotorVoltage = this.leftLeadShooterMotorSimState.getMotorVoltage();

      // 3. Update the physics model
      this.rightFlywheelSim.setInput(rightLeadShooterMotorVoltage);
      this.rightFlywheelSim.update(0.020);
      this.leftFlywheelSim.setInput(leftLeadShooterMotorVoltage);
      this.leftFlywheelSim.update(0.020);

      // 4. Write simulated velocity/position back into the TalonFX sim states
      double rightFlywheelRPS = rightFlywheelSim.getAngularVelocityRPM() / 60;
      double leftFlywheelRPS = leftFlywheelSim.getAngularVelocityRPM() / 60;

      this.rightLeadShooterMotorSimState.setRotorVelocity(rightFlywheelRPS);
      this.rightLeadShooterMotorSimState.setRotorVelocity(-rightFlywheelRPS);
      this.leftLeadShooterMotorSimState.setRotorVelocity(leftFlywheelRPS);
      this.leftFollowerShooterMotorSimState.setRotorVelocity(-leftFlywheelRPS);

      this.rightLeadShooterMotorSimState.addRotorPosition(rightFlywheelRPS * 0.020);
      this.rightFollowerShooterMotorSimState.addRotorPosition(-rightFlywheelRPS * 0.020);
      this.leftLeadShooterMotorSimState.addRotorPosition(leftFlywheelRPS * 0.020);
      this.leftFollowerShooterMotorSimState.addRotorPosition(-leftFlywheelRPS * 0.020);
    }
    
    inputs.connected = BaseStatusSignal.refreshAll(
        rightLeadShooterVolts,
        rightLeadShooterPosition,
        rightLeadShooterRps,
        rightLeadShooterCurrent,
        rightLeadShooterSupplyCurrent,
        rightFollowerShooterVolts,
        rightFollowerShooterPosition,
        rightFollowerShooterRps,
        rightFollowerShooterCurrent,
        rightFollowerShooterSupplyCurrent,
        rightShooterVelocityError,
        leftLeadShooterVolts,
        leftLeadShooterPosition,
        leftLeadShooterRps,
        leftLeadShooterCurrent,
        leftLeadShooterSupplyCurrent,
        leftFollowerShooterVolts,
        leftFollowerShooterPosition,
        leftFollowerShooterRps,
        leftFollowerShooterCurrent,
        leftFollowerShooterSupplyCurrent,
        leftShooterVelocityError).isOK();

    // Update Hardware fields
    inputs.rightLeadShooterVolts = this.rightLeadShooterVolts.getValueAsDouble();
    inputs.rightLeadShooterRps = this.rightLeadShooterRps.getValueAsDouble();
    inputs.rightLeadShooterCurrent = this.rightLeadShooterCurrent.getValueAsDouble();
    inputs.rightLeadShooterSupplyCurrent = this.rightLeadShooterSupplyCurrent.getValueAsDouble();

    inputs.rightFollowShooterVolts = this.rightFollowerShooterVolts.getValueAsDouble();
    inputs.rightFollowShooterRps = this.rightFollowerShooterRps.getValueAsDouble();
    inputs.rightFollowShooterCurrent = this.rightFollowerShooterCurrent.getValueAsDouble();
    inputs.rightFollowShooterSupplyCurrent = this.rightFollowerShooterSupplyCurrent.getValueAsDouble();

    // Update Setpoint related fields
    inputs.rightShooterVelocitySetpoint = this.velocityVoltageRequest.Velocity;
    inputs.rightShooterVelocityError = this.rightShooterVelocityError.getValueAsDouble();
    inputs.rightShooterAtVelocitySetpoint = this.rightShooterAtVelocitySetPoint();

    // Update Hardware fields
    inputs.leftLeadShooterVolts = this.leftLeadShooterVolts.getValueAsDouble();
    inputs.leftLeadShooterRps = this.leftLeadShooterRps.getValueAsDouble();
    inputs.leftLeadShooterCurrent = this.leftLeadShooterCurrent.getValueAsDouble();
    inputs.leftLeadShooterSupplyCurrent = this.leftLeadShooterSupplyCurrent.getValueAsDouble();

    inputs.leftFollowShooterVolts = this.leftFollowerShooterVolts.getValueAsDouble();
    inputs.leftFollowShooterRps = this.leftFollowerShooterRps.getValueAsDouble();
    inputs.leftFollowShooterCurrent = this.leftFollowerShooterCurrent.getValueAsDouble();
    inputs.leftFollowShooterSupplyCurrent = this.leftFollowerShooterSupplyCurrent.getValueAsDouble();

    // Update Setpoint related fields
    inputs.leftShooterVelocitySetpoint = this.velocityVoltageRequest.Velocity;
    inputs.leftShooterVelocityError = this.leftShooterVelocityError.getValueAsDouble();
    inputs.leftShooterAtVelocitySetpoint = this.leftShooterAtVelocitySetPoint();
  }

}
