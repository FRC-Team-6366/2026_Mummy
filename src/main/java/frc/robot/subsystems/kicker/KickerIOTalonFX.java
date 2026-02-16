package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class KickerIOTalonFX implements KickerIO {
    private final TalonFX kickMotor;
    StatusSignal<Voltage> kickVolts;
    StatusSignal<Angle> kickPosition;
    StatusSignal<AngularVelocity> kickRps;
    StatusSignal<Current> kickCurrent;
    StatusSignal<Current> kickSupplyCurrent;

    public KickerIOTalonFX() {
        kickMotor = new TalonFX(Constants.KickerConstants.kickerMotorId);
        TalonFXConfiguration kickConfiguration = new TalonFXConfiguration();
        kickConfiguration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        kickMotor.getConfigurator().apply(kickConfiguration);

        // Setting the StatusSignal variables to be mapped
        // to actual aspect of the KickerIO's hardware
        kickVolts = kickMotor.getMotorVoltage();
        kickPosition = kickMotor.getPosition();
        kickRps = kickMotor.getVelocity();
        kickCurrent = kickMotor.getTorqueCurrent();
        kickSupplyCurrent = kickMotor.getSupplyCurrent();

        // This sets the update frequency for all the StatusSignals
        // for this IO Class
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            kickVolts,
            kickPosition,
            kickRps,
            kickCurrent,
            kickSupplyCurrent
        );

        // Forcing optimal use of the CAN Bus for this subsystems
        // hardware
        kickMotor.optimizeBusUtilization(0, 1);
    }

    /**
     * Sets the power of the kicker. Will used the supplied value and
     * convert it to a value between -12.0 volts and 12.0 volts
     * 
     * @param power Power value between -1.0 and 1.0
     */
    @Override
    public void setKickPower(double power) {
        double voltage = power * 12;
        VoltageOut volts = new VoltageOut(voltage);
        kickMotor.setControl(volts);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        // Check to make sure that all StatusSignal variables
        // are returning values
        inputs.connected = BaseStatusSignal.refreshAll(
                kickVolts,
                kickPosition,
                kickRps,
                kickCurrent,
                kickSupplyCurrent
        ).isOK();

        // Update the inputs object with the current status if the
        // Kicker's object's current statuses
        inputs.kickVolts = this.kickVolts.getValueAsDouble();
        inputs.kickPosition = this.kickVolts.getValueAsDouble();
        inputs.kickRps = this.kickVolts.getValueAsDouble();
        inputs.kickCurrent = this.kickVolts.getValueAsDouble();
        inputs.kickSupplyCurrent = this.kickVolts.getValueAsDouble();
    }
}
