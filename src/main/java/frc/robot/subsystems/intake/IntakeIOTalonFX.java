package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.kicker.KickerIOInputs;

public class IntakeIOTalonFX implements IntakeIO{
    private TalonFX intakeMotorPivot;
    TalonFXConfiguration iMPcfg;
    StatusSignal<Voltage> intakePivotVolts;
    StatusSignal<Angle> intakePivotPosition;
    StatusSignal<AngularVelocity> intakePivotRps;
    StatusSignal<Current> intakePivotCurrent;
    StatusSignal<Current> intakePivotSupplyCurrent;

    private TalonFX intakeMotorRollers;
    TalonFXConfiguration iMRcfg;
    StatusSignal<Voltage> intakeRollersVolts;
    StatusSignal<Angle> intakeRollersPosition;
    StatusSignal<AngularVelocity> intakeRollersRps;
    StatusSignal<Current> intakeRollersCurrent;
    StatusSignal<Current> intakeRollersSupplyCurrent;

    MotionMagicVoltage positionVoltageRequest;
    VoltageOut voltageRequest;

    

public IntakeIOTalonFX(){
    intakeMotorPivot = new TalonFX(20);
    iMPcfg = new TalonFXConfiguration();
    iMPcfg.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    intakeMotorRollers.getConfigurator().apply(iMPcfg);
        // Setting the StatusSignal variables to be mapped
        // to actual aspect of the IntakeIO's hardware
        intakePivotVolts = intakeMotorPivot.getMotorVoltage();
        intakePivotPosition = intakeMotorPivot.getPosition();
        intakePivotRps = intakeMotorPivot.getVelocity();
        intakePivotCurrent = intakeMotorPivot.getTorqueCurrent();
        intakePivotSupplyCurrent = intakeMotorPivot.getSupplyCurrent();

    intakeMotorRollers = new TalonFX(19);
    iMRcfg = new TalonFXConfiguration();
    iMRcfg.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    intakeMotorRollers.getConfigurator().apply(iMRcfg);
        // Setting the StatusSignal variables to be mapped
        // to actual aspect of the IntakeIO's hardware
        intakeRollersVolts = intakeMotorRollers.getMotorVoltage();
        intakeRollersPosition = intakeMotorRollers.getPosition();
        intakeRollersRps = intakeMotorRollers.getVelocity();
        intakeRollersCurrent = intakeMotorRollers.getTorqueCurrent();
        intakeRollersSupplyCurrent = intakeMotorRollers.getSupplyCurrent();

                BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            intakePivotVolts,
            intakePivotPosition,
            intakePivotRps,
            intakePivotCurrent,
            intakePivotSupplyCurrent,
            intakeRollersVolts,
            intakeRollersPosition,
            intakeRollersRps,
            intakeRollersCurrent,
            intakeRollersSupplyCurrent
        );

        // Forcing optimal use of the CAN Bus for this subsystems
        // hardware
        intakeMotorPivot.optimizeBusUtilization(0, 1);
        // Forcing optimal use of the CAN Bus for this subsystems
        // hardware
        intakeMotorRollers.optimizeBusUtilization(0, 1);

        voltageRequest = new VoltageOut(0);
        positionVoltageRequest = new MotionMagicVoltage(null);
}



@Override
public void rollersRunVolts(double power) {
        double voltage = power * 12;
        VoltageOut volts = new VoltageOut(voltage);
        intakeMotorRollers.setControl(volts);
}

@Override
public void rollersStop() {
intakeMotorRollers.stopMotor();
}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Check to make sure that all StatusSignal variables
        // are returning values
        inputs.connected = BaseStatusSignal.refreshAll(
            intakePivotVolts,
            intakePivotPosition,
            intakePivotRps,
            intakePivotCurrent,
            intakePivotSupplyCurrent,
            intakeRollersVolts,
            intakeRollersPosition,
            intakeRollersRps,
            intakeRollersCurrent,
            intakeRollersSupplyCurrent
        ).isOK();

        // Update the inputs object with the current status if the
        // Kicker's object's current statuses
        inputs.intakePivotVolts = this.intakePivotVolts.getValueAsDouble();
        inputs.intakePivotPosition = this.intakePivotPosition.getValueAsDouble();
        inputs.intakePivotRps = this.intakePivotRps.getValueAsDouble();
        inputs.intakePivotCurrent = this.intakePivotCurrent.getValueAsDouble();
        inputs.intakePivotSupplyCurrent = this.intakePivotSupplyCurrent.getValueAsDouble();

        inputs.intakeRollersVolts = this.intakeRollersVolts.getValueAsDouble();
        inputs.intakeRollersPosition = this.intakeRollersPosition.getValueAsDouble();
        inputs.intakeRollersRps = this.intakeRollersRps.getValueAsDouble();
        inputs.intakeRollersCurrent = this.intakeRollersCurrent.getValueAsDouble();
        inputs.intakeRollersSupplyCurrent = this.intakeRollersSupplyCurrent.getValueAsDouble();
    }

}
