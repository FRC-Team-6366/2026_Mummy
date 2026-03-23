package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX intakeRollersMotor;
  TalonFXConfiguration iMRcfg;
  StatusSignal<Voltage> intakeRollersVolts;
  StatusSignal<Angle> intakeRollersPosition;
  StatusSignal<AngularVelocity> intakeRollersRps;
  StatusSignal<Current> intakeRollersCurrent;
  StatusSignal<Current> intakeRollersSupplyCurrent;

  private TalonFX intakePivotMotor;
  TalonFXConfiguration iMPcfg;
  StatusSignal<Voltage> intakePivotVolts;
  StatusSignal<Angle> intakePivotPosition;
  StatusSignal<AngularVelocity> intakePivotRps;
  StatusSignal<Current> intakePivotCurrent;
  StatusSignal<Current> intakePivotSupplyCurrent;
  StatusSignal<Double> intakePivotErrorFromSetpoint;

  private CANcoder intakePivotCANcoder;
  CANcoderConfiguration iPCANcfg;

  PositionVoltage positionVoltageRequest;
  VoltageOut voltageRequest;

  double intakePivotMaxPosition = 0.5;
  double setPointTolerancePercent = 5;
  double setPointTolerance;
  double positionSetPointLow;
  double positionSetPointHigh;

  public IntakeIOTalonFX() {
    intakeRollersMotor = new TalonFX(Constants.IntakeConstants.intakeRollersMotorId); // 19
    iMRcfg = new TalonFXConfiguration();
    iMRcfg.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    intakeRollersMotor.getConfigurator().apply(iMRcfg);
    // Setting the StatusSignal variables to be mapped
    // to actual aspect of the IntakeIO's hardware
    intakeRollersVolts = intakeRollersMotor.getMotorVoltage();
    intakeRollersPosition = intakeRollersMotor.getPosition();
    intakeRollersRps = intakeRollersMotor.getVelocity();
    intakeRollersCurrent = intakeRollersMotor.getTorqueCurrent();
    intakeRollersSupplyCurrent = intakeRollersMotor.getSupplyCurrent();

    intakePivotMotor = new TalonFX(Constants.IntakeConstants.intakePivotMotorId); // 20
    iMPcfg = new TalonFXConfiguration();

    iMPcfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    iMPcfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.51;
    iMPcfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    iMPcfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.12;

    iMPcfg.Slot0.kP = 14;
    iMPcfg.Slot0.kI = 0.5;
    iMPcfg.Slot0.kG = 0.75;
    iMPcfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    iMPcfg.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    iMPcfg.Feedback.FeedbackRemoteSensorID = Constants.IntakeConstants.intakePivotCANcoderId;
    iMPcfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    iMPcfg.Feedback.RotorToSensorRatio = 8.6;
    iMPcfg.Feedback.SensorToMechanismRatio = 1;

    intakePivotMotor.getConfigurator().apply(iMPcfg);
    // Setting the StatusSignal variables to be mapped
    // to actual aspect of the IntakeIO's hardware
    intakePivotVolts = intakePivotMotor.getMotorVoltage();
    intakePivotPosition = intakePivotMotor.getPosition();
    intakePivotRps = intakePivotMotor.getVelocity();
    intakePivotCurrent = intakePivotMotor.getTorqueCurrent();
    intakePivotSupplyCurrent = intakePivotMotor.getSupplyCurrent();
    intakePivotErrorFromSetpoint = intakePivotMotor.getClosedLoopError();

    intakePivotCANcoder = new CANcoder(Constants.IntakeConstants.intakePivotCANcoderId);
    // intakePivotCANcoder.setPosition(0.12);

    iPCANcfg = new CANcoderConfiguration();
    iPCANcfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    iPCANcfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.75;
    intakePivotCANcoder.getConfigurator().apply(iPCANcfg);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        intakeRollersVolts,
        intakeRollersPosition,
        intakeRollersRps,
        intakeRollersCurrent,
        intakeRollersSupplyCurrent,
        intakePivotVolts,
        intakePivotPosition,
        intakePivotRps,
        intakePivotCurrent,
        intakePivotSupplyCurrent,
        intakePivotErrorFromSetpoint);

    // Forcing optimal use of the CAN Bus for this subsystems
    // hardware
    intakeRollersMotor.optimizeBusUtilization(0, 1);

    // Forcing optimal use of the CAN Bus for this subsystems
    // hardware
    intakePivotMotor.optimizeBusUtilization(0, 1);

    voltageRequest = new VoltageOut(0);
    positionVoltageRequest = new PositionVoltage(0.12);

    this.setPointTolerance = intakePivotMaxPosition * this.setPointTolerancePercent;
  }

  // Resets cancoder to 0.12 rotations, must be used when intake is at
  // upper hard limit
  @Override
  public void intakeResetCanCoder() {
    intakePivotCANcoder.setPosition(0.12);
  }

  // Turns pivot neutral mode to 'brake' on true, 'coast' on false
  @Override
  public void setBrakeMode(boolean brakeMode) {
    intakePivotMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void rollersRunVolts(double power) {
    double voltage = power * 12;
    VoltageOut volts = new VoltageOut(voltage);
    intakeRollersMotor.setControl(volts);
  }

    @Override
  public void pivotRunVolts(double power) {
    double voltage = power * 12;
    VoltageOut volts = new VoltageOut(voltage);
    intakePivotMotor.setControl(volts);
  }

  @Override
  public void rollersStop() {
    intakeRollersMotor.stopMotor();
  }

  @Override
  public Rotation2d getRotations() {
    return new Rotation2d(Units.rotationsToRadians(intakePivotMotor.getPosition().getValueAsDouble()));
  }

  @Override
  public void intakePivotToAngle(double angleDegrees) {
    double angletoRotations = (MathUtil.clamp(angleDegrees, 0.0, 138.6)) / 360; // Changes angle degree to rotations
    angletoRotations += 0.12;
    this.intakePivotMotor.setControl(positionVoltageRequest.withPosition(angletoRotations));
  }

  @Override
  public double getIntakePivotPositionError() {
    return this.intakePivotMotor.getClosedLoopError().getValueAsDouble();
  }

  @Override
  public boolean intakeAtPositionSetpoint() {
    return Math.abs(this.getIntakePivotPositionError()) < this.setPointTolerance;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Check to make sure that all StatusSignal variables
    // are returning values
    inputs.connected = BaseStatusSignal.refreshAll(
        intakeRollersVolts,
        intakeRollersPosition,
        intakeRollersRps,
        intakeRollersCurrent,
        intakeRollersSupplyCurrent,
        intakePivotVolts,
        intakePivotPosition,
        intakePivotRps,
        intakePivotCurrent,
        intakePivotSupplyCurrent,
        intakePivotErrorFromSetpoint
        

    ).isOK();

    inputs.intakeRollersVolts = this.intakeRollersVolts.getValueAsDouble();
    inputs.intakeRollersPosition = this.intakeRollersPosition.getValueAsDouble();
    inputs.intakeRollersRps = this.intakeRollersRps.getValueAsDouble();
    inputs.intakeRollersCurrent = this.intakeRollersCurrent.getValueAsDouble();
    inputs.intakeRollersSupplyCurrent = this.intakeRollersSupplyCurrent.getValueAsDouble();

    // Update the inputs object with the current status if the
    // Kicker's object's current statuses
    inputs.intakePivotVolts = this.intakePivotVolts.getValueAsDouble();
    inputs.intakePivotPosition = this.intakePivotPosition.getValueAsDouble();
    inputs.intakePivotRps = this.intakePivotRps.getValueAsDouble();
    inputs.intakePivotCurrent = this.intakePivotCurrent.getValueAsDouble();
    inputs.intakePivotSupplyCurrent = this.intakePivotSupplyCurrent.getValueAsDouble();
    inputs.intakePivotErrorFromSetpoint = this.intakePivotErrorFromSetpoint.getValueAsDouble();
    inputs.intakePivotSetpoint = this.positionVoltageRequest.Position;
    inputs.intakePivotAtSetpoint = this.intakeAtPositionSetpoint();

  }

}
