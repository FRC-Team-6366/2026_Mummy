package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants.IdConstants;

public class HoodIOTalonFX implements HoodIO {
    TalonFX hoodMotor;
    VoltageOut volts;

    StatusSignal<Voltage> hoodVolts;
    StatusSignal<Angle> hoodPosition;
    StatusSignal<AngularVelocity> hoodRps;
    StatusSignal<Current> hoodCurrent;
    StatusSignal<Current> hoodSupplyCurrent;
    StatusSignal<Angle> hoodCANPositionRotations;
    StatusSignal<AngularVelocity> hoodCANVelocityRps;

    public HoodIOTalonFX() {
        hoodMotor = new TalonFX(IdConstants.hoodMotorID);

        hoodVolts = hoodMotor.getMotorVoltage();
        hoodPosition = hoodMotor.getPosition();
        hoodRps = hoodMotor.getVelocity();
        hoodCurrent = hoodMotor.getTorqueCurrent();
        hoodSupplyCurrent = hoodMotor.getSupplyCurrent();

        TalonFXConfiguration hoodcfg = new TalonFXConfiguration();
        hoodcfg.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        
        this.hoodMotor.getConfigurator().apply(hoodcfg);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                hoodVolts,
                hoodPosition,
                hoodRps,
                hoodCurrent,
                hoodSupplyCurrent,
                hoodCANPositionRotations,
                hoodCANVelocityRps);

        hoodMotor.optimizeBusUtilization(0.0, 1.0);

        volts = new VoltageOut(0.0);
    }
    

    @Override
    public Rotation2d getRotations() {
    return new Rotation2d(Units.rotationsToRadians(hoodMotor.getPosition().getValueAsDouble()));
    }

    @Override
    public void hoodToAngle(double angle) {
    hoodMotor.setControl(volts.withOutput(0));
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
