package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class KickerIOInputs {
    boolean connected = false;
    double kickVolts = 0;
    double kickPosition = 0;
    double kickRps = 0;
    double kickCurrent = 0;
    double kickSupplyCurrent = 0;
}
