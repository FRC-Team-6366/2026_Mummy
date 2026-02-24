package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;

public class HoodIOTalonFX implements HoodIO {
    TalonFX hoodMotor;

    /**
     * Used to control motor output by specifying postiton setpoint
     * (number of rotations from start position)
     */
    PositionVoltage positionVoltageRequest;

    StatusSignal<Voltage> hoodVolts;
    StatusSignal<Angle> hoodPosition;
    StatusSignal<AngularVelocity> hoodRps;
    StatusSignal<Current> hoodCurrent;
    StatusSignal<Current> hoodSupplyCurrent;
    StatusSignal<Double> hoodErrorFromSetpoint;

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
        // Instantiating Hood motor and its variables for monitoring
        hoodMotor = new TalonFX(ShooterConstants.hoodMotorId);
        hoodVolts = hoodMotor.getMotorVoltage();
        hoodPosition = hoodMotor.getPosition();
        hoodRps = hoodMotor.getVelocity();
        hoodCurrent = hoodMotor.getTorqueCurrent();
        hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
        hoodErrorFromSetpoint = hoodMotor.getClosedLoopError();

        // Instantiating and configuring configuration for Hood motor
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        cfg.Slot0.kP = 1;
        cfg.Slot0.kI = 1;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5.6;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        this.hoodMotor.getConfigurator().apply(cfg);

        // Set inital encoder value to 0
        // NOTE: Make sure hood is completely retracted with starting robot!
        hoodMotor.setPosition(0);

        // Set update period for device metrics to be 50 Hz (20 milliseconds)
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                hoodVolts,
                hoodPosition,
                hoodRps,
                hoodCurrent,
                hoodSupplyCurrent,
                hoodErrorFromSetpoint);

        hoodMotor.optimizeBusUtilization(0.0, 1.0);

        // Instantiating position voltage object for setting the output position
        positionVoltageRequest = new PositionVoltage(0);
        hoodMotor.setControl(positionVoltageRequest.withSlot(0));

        // Compute setpoint tolerance from max position and tolerance percent
        this.setPointTolerance = hoodMaxPosition * this.setPointTolerancePercent / 100;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Rotation2d getRotations() {
        return new Rotation2d(Units.rotationsToRadians(hoodMotor.getPosition().getValueAsDouble()));
    }

    
    /**
     * Sets the hood to the specified angle by converting it
     * to number of rotations for the motor
     * @param angle Angle in degrees. From 0 to 45 max
     */
    @Override
     public void hoodToAngle(double angle) {
        // We are trying to map the degrees 0 -> 45 to the motor rotations 0 -> 5.6
        // We map angle to an input value between 0 and 45 to prevent larger or smaller values
        // Then divide the angle by the ratio between the max angle (45) and the max postion in rotations (5.6)
        double angletoRotations = MathUtil.clamp(angle, 0, 45) / (45 / hoodMaxPosition);
        this.hoodMotor.setControl(positionVoltageRequest.withPosition(angletoRotations));
    }

    /**
     * Sets the hood to move to a certian positon, otherwise known as
     * number of rotations from starting position
     * 
     * @param position Number of rotations. Min 0, max 5.6
     */
    @Override
    public void hoodToPosition(double position) {
        this.hoodMotor.setControl(positionVoltageRequest.withPosition(position));
    }

    @Override
    public double getHoodPositionError() {
        return this.hoodMotor.getClosedLoopError().getValueAsDouble();
    }

    @Override
    public boolean hoodAtPositionSetpoint() {
        // Get absolute value of the error and see if it is less
        // than the setpoint tolerance
        return Math.abs(this.getHoodPositionError()) < this.setPointTolerance;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                hoodVolts,
                hoodPosition,
                hoodRps,
                hoodCurrent,
                hoodSupplyCurrent,
                hoodErrorFromSetpoint
        ).isOK();

        // Update Hardware fields
        inputs.hoodVolts = this.hoodVolts.getValueAsDouble();
        inputs.hoodPosition = this.hoodPosition.getValueAsDouble();
        inputs.hoodRps = this.hoodRps.getValueAsDouble();
        inputs.hoodCurrent = this.hoodCurrent.getValueAsDouble();
        inputs.hoodSupplyCurrent = this.hoodSupplyCurrent.getValueAsDouble();
        
        // Upodate Setpoint related fields
        inputs.hoodPositionSetpoint = this.positionVoltageRequest.Position;
        inputs.hoodPositionError = this.hoodErrorFromSetpoint.getValueAsDouble();
        inputs.hoodAtSetpoint = this.hoodAtPositionSetpoint();
    }

}
