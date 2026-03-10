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
   * <p>Poses are always in Blue-alliance coordinates (as returned by {@code drive.getPose()}).
   * Dispatches to the alliance-specific implementation.
   *
   * <ul>
   *   <li>Blue in {@link Zone#BLUE_HOME} / Red in {@link Zone#RED_HOME} → {@link
   *       FieldSpot#HUB_NEUTRAL}
   *   <li>Top half of field (Y &ge; {@link FieldSpot#Y_SPLIT_M}) → {@link FieldSpot#SHOOTING_TOP}
   *   <li>Bottom half of field (Y &lt; {@link FieldSpot#Y_SPLIT_M}) → {@link
   *       FieldSpot#SHOOTING_BOTTOM}
   * </ul>
   *
   * @param robotPose current robot pose in Blue-alliance coordinates
   */
  public void autoSelectForRobot(Pose2d robotPose) {
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    if (isRed) {
      autoSelectForRobotRed(robotPose);
    } else {
      autoSelectForRobotBlue(robotPose);
    }
  }

  /**
   * Auto-selection logic for the <strong>Blue alliance</strong>.
   *
   * <p>Blue's home zone is {@link Zone#BLUE_HOME} (X &lt; {@link FieldSpot#BLUE_TRENCH_X_M}). When
   * in the home zone, selects {@link FieldSpot#HUB_NEUTRAL} to score. Otherwise, selects the
   * shooting position on the same half of the field as the robot:
   *
   * <ul>
   *   <li>Top half (Y &ge; {@link FieldSpot#Y_SPLIT_M}) → {@link FieldSpot#SHOOTING_TOP}
   *   <li>Bottom half → {@link FieldSpot#SHOOTING_BOTTOM}
   * </ul>
   */
  private void autoSelectForRobotBlue(Pose2d robotPose) {
    if (FieldSpot.getZone(robotPose.getX()) == Zone.BLUE_HOME) {
      selectSpot(FieldSpot.HUB_NEUTRAL);
      return;
    }
    // High Y (blue-frame) = TOP half of field — select the TOP shooting position.
    boolean topHalf = robotPose.getY() >= FieldSpot.Y_SPLIT_M;
    selectSpot(topHalf ? FieldSpot.SHOOTING_TOP : FieldSpot.SHOOTING_BOTTOM);
  }

  /**
   * Auto-selection logic for the <strong>Red alliance</strong>.
   *
   * <p>Red's home zone is {@link Zone#RED_HOME} (X &gt; {@link FieldSpot#RED_TRENCH_X_M} in
   * Blue-alliance frame). When in the home zone, selects {@link FieldSpot#HUB_NEUTRAL} to score.
   * Otherwise, selects the shooting position on the same half of the field as the robot.
   *
   * <p>TOP/BOTTOM is determined by the robot's Y in blue-frame — the same check as for Blue —
   * because TOP and BOTTOM refer to absolute field positions (high Y / low Y in blue-frame). After
   * PathPlanner's alliance flip, the correct physical corridor is selected automatically without
   * any inversion of the Y comparison.
   */
  private void autoSelectForRobotRed(Pose2d robotPose) {
    if (FieldSpot.getZone(robotPose.getX()) == Zone.RED_HOME) {
      selectSpot(FieldSpot.HUB_NEUTRAL);
      return;
    }
    // High Y in blue-frame = TOP half of field — same physical meaning for both
    // alliances.
    // PathPlanner flips SHOOTING_TOP to the correct side for Red automatically.
    boolean topHalf = robotPose.getY() < FieldSpot.Y_SPLIT_M;
    selectSpot(topHalf ? FieldSpot.SHOOTING_TOP : FieldSpot.SHOOTING_BOTTOM);
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
