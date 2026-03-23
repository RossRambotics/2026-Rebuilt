package frc.robot.subsystems;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Base class for all subsystems that automatically records loop timing telemetry to
 * "Timing/<ClassName>" in milliseconds. Subclasses implement periodicImpl() instead of periodic().
 */
public abstract class TimedSubsystem extends SubsystemBase {
  @Override
  public final void periodic() {
    long t = HALUtil.getFPGATime(); // microseconds
    periodicImpl();
    Logger.recordOutput("Timing/" + getName(), (HALUtil.getFPGATime() - t) / 1e3); // ms
  }

  protected abstract void periodicImpl();
}
