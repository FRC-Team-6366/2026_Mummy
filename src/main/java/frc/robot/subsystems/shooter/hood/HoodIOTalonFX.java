package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

    
   MotionMagicVoltage positionVoltageRequest;
    VoltageOut volts;

    StatusSignal<Voltage> hoodVolts;
    StatusSignal<Angle> hoodPosition;
    StatusSignal<AngularVelocity> hoodRps;
    StatusSignal<Current> hoodCurrent;
    StatusSignal<Current> hoodSupplyCurrent;
    StatusSignal<Angle> hoodCANPositionRotations;
    StatusSignal<AngularVelocity> hoodCANVelocityRps;

    public HoodIOTalonFX() {
        hoodMotor = new TalonFX(ShooterConstants.hoodMotorId);

        hoodVolts = hoodMotor.getMotorVoltage();
        hoodPosition = hoodMotor.getPosition();
        hoodRps = hoodMotor.getVelocity();
        hoodCurrent = hoodMotor.getTorqueCurrent();

        

                TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
            cfg.Slot0.kP = 0;
    cfg.Slot0.kI =  0;
    cfg.Slot0.kD = 0;
    cfg.Slot0.kG = 0;
    cfg.Slot0.kS = 0;
    cfg.Slot0.kV = 0;
    cfg.Slot0.kA = 0;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(0);
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(0);
        cfg.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(0));
    cfg.MotionMagic.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(0));
    cfg.MotionMagic.withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0));    
    this.hoodMotor.getConfigurator().apply(cfg);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                hoodVolts,
                hoodPosition,
                hoodRps,
                hoodCurrent,
                hoodSupplyCurrent,
                hoodCANPositionRotations,
                hoodCANVelocityRps);


        positionVoltageRequest = new MotionMagicVoltage(0.0);
        volts = new VoltageOut(0.0);
        }
    

    @Override
    public Rotation2d getRotations() {
        return new Rotation2d(Units.rotationsToRadians(hoodMotor.getPosition().getValueAsDouble()));
    }

    @Override
    public void hoodToAngle(double angle) {
    hoodMotor.setControl(positionVoltageRequest.withPosition(Units.degreesToRotations(angle)).withSlot(0));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                hoodVolts,
                hoodPosition,
                hoodRps,
                hoodCurrent,
                hoodSupplyCurrent,
                hoodCANPositionRotations,
                hoodCANVelocityRps).isOK();

        inputs.hoodVolts = this.hoodVolts.getValueAsDouble();
        inputs.hoodPosition = this.hoodPosition.getValueAsDouble();
        inputs.hoodRps = this.hoodRps.getValueAsDouble();
        inputs.hoodCurrent = this.hoodCurrent.getValueAsDouble();
        inputs.hoodSupplyCurrent = this.hoodSupplyCurrent.getValueAsDouble();
        inputs.hoodCANPositionRotations = this.hoodCANPositionRotations.getValueAsDouble();
        inputs.hoodCANVelocityRps = this.hoodCANVelocityRps.getValueAsDouble();
    }

}
