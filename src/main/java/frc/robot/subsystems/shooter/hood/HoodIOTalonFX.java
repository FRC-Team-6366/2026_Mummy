package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

    PositionVoltage positionVoltageRequest;
    VoltageOut volts;

    StatusSignal<Voltage> hoodVolts;
    StatusSignal<Angle> hoodPosition;
    StatusSignal<AngularVelocity> hoodRps;
    StatusSignal<Current> hoodCurrent;
    StatusSignal<Current> hoodSupplyCurrent;

    public HoodIOTalonFX() {
        hoodMotor = new TalonFX(ShooterConstants.hoodMotorId);

        hoodVolts = hoodMotor.getMotorVoltage();
        hoodPosition = hoodMotor.getPosition();
        hoodRps = hoodMotor.getVelocity();
        hoodCurrent = hoodMotor.getTorqueCurrent();
        hoodSupplyCurrent = hoodMotor.getSupplyCurrent();



    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    cfg.Slot0.kP = 1;
    cfg.Slot0.kI =  1;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5.6;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    this.hoodMotor.getConfigurator().apply(cfg);

             hoodMotor.setPosition(0);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                hoodVolts,
                hoodPosition,
                hoodRps,
                hoodCurrent,
                hoodSupplyCurrent);

            hoodMotor.optimizeBusUtilization(0.0, 1.0);


        positionVoltageRequest = new PositionVoltage(0);
        positionVoltageRequest.Velocity = 5;
        hoodMotor.setControl(positionVoltageRequest.withSlot(0));
        }
    

    @Override
    public Rotation2d getRotations() {
        return new Rotation2d(Units.rotationsToRadians(hoodMotor.getPosition().getValueAsDouble()));
    }

    @Override
    public void hoodToAngle(double angle) {
        positionVoltageRequest.Position = Units.degreesToRotations(angle);
        
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


    @Override
    public void hoodToPosition(double position) {
       this.hoodMotor.setControl(positionVoltageRequest.withPosition(position));
    }

}
