// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.FieldSpot;
import frc.robot.commands.FieldSpot.Zone;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks the currently selected target spot for {@link frc.robot.commands.DriveToSpot}.
 *
 * <p>Button box bindings call {@link #cmdSelectSpot} to update the selection. The Y button then
 * triggers {@code DriveToSpot}, which reads the selection from here.
 *
 * <p>A custom arbitrary pose can be set via {@link #cmdSelectCustomPose} for fine-tuning without
 * adding a new enum entry.
 */
public class SpotSelector extends TimedSubsystem {

  private FieldSpot m_selectedSpot = null;
  private Pose2d m_customPosePose2d = null;
  private String m_customName = null;

  /** True while {@link frc.robot.commands.DriveToSpot} is actively pathfinding. */
  private boolean m_isAutoDriving = false;

  public SpotSelector() {}

  @Override
  protected void periodicImpl() {
    Logger.recordOutput("SpotSelector/SelectedName", getSelectedName());
    Logger.recordOutput("SpotSelector/HasTarget", getSelectedPose().isPresent());
    Logger.recordOutput("SpotSelector/IsAutoDriving", m_isAutoDriving);
    Logger.recordOutput(
        "SpotSelector/SelectedPose", FieldSpot.flipIfRed(getSelectedPose().orElse(new Pose2d())));
  }

  // ── Auto-drive state (set by DriveToSpot) ─────────────────────────────────

  /** Called by {@link frc.robot.commands.DriveToSpot} when pathfinding starts or stops. */
  public void setAutoDriving(boolean isAutoDriving) {
    m_isAutoDriving = isAutoDriving;
  }

  /**
   * Returns {@code true} while {@link frc.robot.commands.DriveToSpot} is actively pathfinding.
   * Zone-based teleop triggers should be gated on {@code !isAutoDriving()} to avoid interfering
   * with PathPlanner.
   */
  public boolean isAutoDriving() {
    return m_isAutoDriving;
  }

  // ── Selection API ──────────────────────────────────────────────────────────

  /** Sets the selected spot to one of the predefined {@link FieldSpot} entries. */
  public void selectSpot(FieldSpot spot) {
    m_selectedSpot = spot;
    m_customPosePose2d = null;
    m_customName = null;
  }

  /**
   * Sets a custom target pose not in the {@link FieldSpot} enum. Useful for fine-tuning without
   * adding a permanent entry.
   */
  public void selectCustomPose(Pose2d pose, String name) {
    m_selectedSpot = null;
    m_customPosePose2d = pose;
    m_customName = name;
  }

  /** Clears the current selection so that {@link frc.robot.commands.DriveToSpot} will no-op. */
  public void clearSelection() {
    m_selectedSpot = null;
    m_customPosePose2d = null;
    m_customName = null;
  }

  // ── Accessors ──────────────────────────────────────────────────────────────

  /**
   * Returns the selected {@link FieldSpot}, or empty if a custom pose or nothing is selected.
   * Useful for callers that need zone or display information beyond the raw pose.
   */
  public Optional<FieldSpot> getSelectedSpot() {
    return Optional.ofNullable(m_selectedSpot);
  }

  /**
   * Returns the selected target pose, or {@link Optional#empty()} if nothing is selected. {@link
   * frc.robot.commands.DriveToSpot} exits immediately when empty.
   */
  public Optional<Pose2d> getSelectedPose() {
    if (m_selectedSpot != null) {
      return Optional.of(m_selectedSpot.pose);
    }
    if (m_customPosePose2d != null) {
      return Optional.of(m_customPosePose2d);
    }
    return Optional.empty();
  }

  /** Returns the human-readable display name of the current selection, or {@code "None"}. */
  public String getSelectedName() {
    if (m_selectedSpot != null) {
      return m_selectedSpot.displayName;
    }
    if (m_customName != null) {
      return m_customName;
    }
    return "None";
  }

  // ── Auto-selection ─────────────────────────────────────────────────────────

  /**
   * Selects a target spot automatically based on the robot's current field position.
   *
   * <p>Poses are always in Blue-alliance coordinates, so "own alliance zone" maps to {@link
   * Zone#ALLIANCE} for blue and {@link Zone#OPPOSITION} for red.
   *
   * <ul>
   *   <li>Own alliance zone → {@link FieldSpot#HUB_NEUTRAL}
   *   <li>Neutral or opponent zone, left side (Y ≥ {@link FieldSpot#Y_SPLIT_M}) → {@link
   *       FieldSpot#ALLIANCE_SHOOTING_LEFT}
   *   <li>Neutral or opponent zone, right side (Y < {@link FieldSpot#Y_SPLIT_M}) → {@link
   *       FieldSpot#ALLIANCE_SHOOTING_RIGHT}
   * </ul>
   *
   * @param robotPose current robot pose in Blue-alliance coordinates
   */
  public void autoSelectForRobot(Pose2d robotPose) {
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    Zone zone = FieldSpot.getZone(robotPose.getX());
    // In blue-frame: blue's home = ALLIANCE zone, red's home = OPPOSITION zone.
    boolean inOwnAllianceZone = isRed ? (zone == Zone.OPPOSITION) : (zone == Zone.ALLIANCE);
    if (inOwnAllianceZone) {
      selectSpot(FieldSpot.HUB_NEUTRAL);
    } else {
      // ALLIANCE_SHOOTING_LEFT is at high Y in blue-frame; PathPlanner flips it to low Y for red.
      // So for red, a robot at high Y should pick RIGHT (which flips to high Y), not LEFT.
      boolean highY = robotPose.getY() >= FieldSpot.Y_SPLIT_M;
      boolean pickLeft = isRed ? !highY : highY;
      selectSpot(pickLeft ? FieldSpot.ALLIANCE_SHOOTING_LEFT : FieldSpot.ALLIANCE_SHOOTING_RIGHT);
    }
  }

  // ── Command factories (for button box bindings) ────────────────────────────

  /**
   * Returns a command that selects the given spot when triggered. Runs while disabled so drivers
   * can pre-select a target before enable.
   */
  public Command cmdSelectSpot(FieldSpot spot) {
    return new InstantCommand(() -> selectSpot(spot), this).ignoringDisable(true);
  }

  /**
   * Returns a command that selects a custom pose when triggered. Runs while disabled so drivers can
   * pre-select a target before enable.
   *
   * @param pose target pose (Blue-alliance frame)
   * @param name display name shown in telemetry
   */
  public Command cmdSelectCustomPose(Pose2d pose, String name) {
    return new InstantCommand(() -> selectCustomPose(pose, name), this).ignoringDisable(true);
  }

  /**
   * Returns a command that clears the current selection when triggered. Runs while disabled so
   * drivers can clear a target before enable.
   */
  public Command cmdClearSelection() {
    return new InstantCommand(this::clearSelection, this).ignoringDisable(true);
  }
}
