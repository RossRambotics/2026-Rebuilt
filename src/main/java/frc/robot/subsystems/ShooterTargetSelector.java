// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootingTarget;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks the currently selected {@link ShootingTarget} for the shooter.
 *
 * <p>The default command ({@link frc.robot.commands.AutoTargetSelector}) automatically selects the
 * appropriate target based on the robot's field zone. Operator buttons call {@link
 * #cmdSelectManual(ShootingTarget)} to override automatic selection; auto-selection resumes only
 * when {@link #cmdEnableAuto()} is triggered.
 */
public class ShooterTargetSelector extends TimedSubsystem {

  private final Shooter m_shooter;
  private ShootingTarget m_selectedTarget = ShootingTarget.HUB;
  private boolean m_isAutoEnabled = true;

  public ShooterTargetSelector(Shooter shooter) {
    m_shooter = shooter;
  }

  @Override
  protected void periodicImpl() {
    Logger.recordOutput("ShooterTargetSelector/SelectedTarget", m_selectedTarget.displayName);
    Logger.recordOutput("ShooterTargetSelector/IsAutoEnabled", m_isAutoEnabled);
  }

  // ── Selection API ──────────────────────────────────────────────────────────

  /**
   * Called by {@link frc.robot.commands.AutoTargetSelector} each loop to update the target based on
   * robot position. Does nothing when a manual selection is active ({@link #isAutoEnabled()} is
   * {@code false}) or the target is already set to {@code target}.
   */
  public void selectAuto(ShootingTarget target) {
    if (!m_isAutoEnabled) return;
    if (target == m_selectedTarget) return;
    m_selectedTarget = target;
    m_shooter.setTargetPose(target.displayName, target.getFieldPose3d());
  }

  /**
   * Selects a target manually and disables automatic zone-based selection. Auto-selection resumes
   * only when {@link #enableAuto()} is called.
   */
  public void selectManual(ShootingTarget target) {
    m_isAutoEnabled = false;
    if (target == m_selectedTarget) return;
    m_selectedTarget = target;
    m_shooter.setTargetPose(target.displayName, target.getFieldPose3d());
  }

  /**
   * Re-enables automatic zone-based target selection. {@link frc.robot.commands.AutoTargetSelector}
   * will resume updating the target on its next loop iteration.
   */
  public void enableAuto() {
    m_isAutoEnabled = true;
  }

  // ── Accessors ──────────────────────────────────────────────────────────────

  /** Returns the currently selected shooting target. */
  public ShootingTarget getSelected() {
    return m_selectedTarget;
  }

  /**
   * Returns {@code true} while {@link frc.robot.commands.AutoTargetSelector} is allowed to update
   * the target. Returns {@code false} after a manual selection until {@link #enableAuto()} is
   * called.
   */
  public boolean isAutoEnabled() {
    return m_isAutoEnabled;
  }

  // ── Command factories (for button box bindings) ────────────────────────────

  /**
   * Returns a command that manually selects the given target and disables automatic zone-based
   * selection. Runs while disabled so operators can pre-select a target before enable.
   */
  public Command cmdSelectManual(ShootingTarget target) {
    Command c = Commands.runOnce(() -> selectManual(target)).ignoringDisable(true);
    c.setName("Target/" + target.displayName);
    return c;
  }

  /**
   * Returns a command that re-enables automatic zone-based target selection. Runs while disabled so
   * operators can restore auto-targeting before enable.
   */
  public Command cmdEnableAuto() {
    Command c = Commands.runOnce(this::enableAuto).ignoringDisable(true);
    c.setName("Target/EnableAuto");
    return c;
  }
}
