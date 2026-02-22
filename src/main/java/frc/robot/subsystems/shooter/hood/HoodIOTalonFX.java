package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;

public class HoodIOTalonFX implements HoodIO {
    TalonFX hoodMotor;

    // Used to control motor output by specifying postiton setpoint
    // (number of rotations)
    PositionVoltage positionVoltageRequest;

    StatusSignal<Voltage> hoodVolts;
    StatusSignal<Angle> hoodPosition;
    StatusSignal<AngularVelocity> hoodRps;
    StatusSignal<Current> hoodCurrent;
    StatusSignal<Current> hoodSupplyCurrent;

    public HoodIOTalonFX() {
        // Instantiating Hood motor and its variables for monitoring
        hoodMotor = new TalonFX(ShooterConstants.hoodMotorId);
        hoodVolts = hoodMotor.getMotorVoltage();
        hoodPosition = hoodMotor.getPosition();
        hoodRps = hoodMotor.getVelocity();
        hoodCurrent = hoodMotor.getTorqueCurrent();
        hoodSupplyCurrent = hoodMotor.getSupplyCurrent();

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
                hoodSupplyCurrent);

        hoodMotor.optimizeBusUtilization(0.0, 1.0);

        // Instantiating position voltage object for setting the output position
        positionVoltageRequest = new PositionVoltage(0);
        hoodMotor.setControl(positionVoltageRequest.withSlot(0));
    }

    @Override
    public Rotation2d getRotations() {
        return new Rotation2d(Units.rotationsToRadians(hoodMotor.getPosition().getValueAsDouble()));
    }

    @Override
    public void hoodToAngle(double angle) {
        // We are trying to map the degrees 0 -> 45 to the rotations0 -> 5.6 
        

        // This code only converts angle (0 -> 360) to rotations of the motor.
        // 90 degrees would only make the motor shaft spin 1/4 turn. Not what 
        // we want
        positionVoltageRequest.Position = Units.degreesToRotations(angle);
    }

    /**
     * Sets the hood to move to a certian positon, otherwise known as
     * number of rotations from starting position
     * @param position Number of rotations. Min 0, max 5.6
     */
    @Override
    public void hoodToPosition(double position) {
        this.hoodMotor.setControl(positionVoltageRequest.withPosition(position));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                hoodVolts,
                hoodPosition,
                hoodRps,
                hoodCurrent,
                hoodSupplyCurrent).isOK();

        inputs.hoodVolts = this.hoodVolts.getValueAsDouble();
        inputs.hoodPosition = this.hoodPosition.getValueAsDouble();
        inputs.hoodRps = this.hoodRps.getValueAsDouble();
        inputs.hoodCurrent = this.hoodCurrent.getValueAsDouble();
        inputs.hoodSupplyCurrent = this.hoodSupplyCurrent.getValueAsDouble();
    }

}
