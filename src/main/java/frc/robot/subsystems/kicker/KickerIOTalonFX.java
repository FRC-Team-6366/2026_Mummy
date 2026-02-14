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

public class KickerIOTalonFX implements KickerIO{
    private final TalonFX kickMotor;


    StatusSignal<Voltage> kickVolts;
    StatusSignal<Angle> kickPosition;
    StatusSignal<AngularVelocity> kickRps;
    StatusSignal<Current> kickCurrent;
    StatusSignal<Current> kickSupplyCurrent;
    
    public KickerIOTalonFX(){
        kickMotor = new TalonFX(24);//assuming the motor id is the same as prior testing
        TalonFXConfiguration kickConfiguration = new TalonFXConfiguration();
        kickConfiguration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        kickMotor.getConfigurator().apply(kickConfiguration);

        kickVolts = kickMotor.getMotorVoltage();
        kickPosition = kickMotor.getPosition();
        kickRps = kickMotor.getVelocity();
        kickCurrent = kickMotor.getTorqueCurrent();
        kickSupplyCurrent = kickMotor.getSupplyCurrent();
        //typical set of values to log for a motor^^



        BaseStatusSignal.setUpdateFrequencyForAll(
            
            50,
            kickVolts,
            kickPosition,
            kickRps,
            kickCurrent,
            kickSupplyCurrent
            );

        // don't recall what this is for^

        kickMotor.optimizeBusUtilization(0,1);
        //same here^
    }

    @Override
    public void setKickPower(double power){
        double voltage = power *12;
        VoltageOut volts = new VoltageOut(voltage);
        kickMotor.setControl(volts);
    }


    
	@Override
	public void updateInputs(KickerIOInputs inputs) {
		inputs.connected = BaseStatusSignal.refreshAll (
            kickVolts,
            kickPosition,
            kickRps,
            kickCurrent,
            kickSupplyCurrent

        ).isOK();

        inputs.kickVolts = this.kickVolts.getValueAsDouble();
        inputs.kickPosition = this.kickVolts.getValueAsDouble();
        inputs.kickRps = this.kickVolts.getValueAsDouble();
        inputs.kickCurrent = this.kickVolts.getValueAsDouble();
        inputs.kickSupplyCurrent = this.kickVolts.getValueAsDouble();
	}

    
}
