
package frc.robot.subsystems.Pivotmain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivotmain extends SubsystemBase {
    private Pivotmain io;
   // private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private double setpointAngleDegrees;

    public Pivot(Pivotmain io) {
    this.io = io;
  }
    public void pivotToAngle(double angleDegrees) {
    io.pivotToAngle(angleDegrees);
    this.setpointAngleDegrees = angleDegrees;
    Logger.recordOutput("Pivot/setpoint", angleDegrees);
  }

  public void stop() {
    io.stop();
  }

  public Rotation2d getRotation() {
    return io.getRotation();
  }

  public boolean atSetpoint() {
    // Debouncer setpointDebouncer = new Debouncer(0.5);
    double closedLoopErrorDegrees = io.getRotation().getDegrees() - setpointAngleDegrees;
    // boolean atSetpoint = setpointDebouncer.calculate(Math.abs(closedLoopError) < 1);
    boolean atSetpoint = Math.abs(closedLoopErrorDegrees) < 1;
    Logger.recordOutput("Pivot/atSetPoint", atSetpoint);
    Logger.recordOutput("Pivot/closedLoopError", closedLoopErrorDegrees);
    return atSetpoint;
  }
    
}
