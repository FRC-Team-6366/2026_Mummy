package frc.robot.subsystems.intake;

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
    private VoltageOut voltageRequest;

    

IntakeIOTalonFX(){
    intakeMotorPivot = new TalonFX(20);

    intakeMotorRollers = new TalonFX(19);
    iMRcfg = new TalonFXConfiguration();
    iMRcfg.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    intakeMotorRollers.getConfigurator().apply(iMRcfg);


}

}
