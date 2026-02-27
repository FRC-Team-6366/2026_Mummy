package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IntakeIOInputs {
    boolean connected = false;

    double intakePivotVolts = 0;
    double intakePivotPosition = 0;
    double intakePivotRps = 0;
    double intakePivotCurrent = 0;
    double intakePivotSupplyCurrent = 0;

    double intakeRollersVolts = 0;
    double intakeRollersPosition = 0;
    double intakeRollersRps = 0;
    double intakeRollersCurrent = 0;
    double intakeRollersSupplyCurrent = 0;
}
