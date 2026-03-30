package frc.robot.util;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.InputStream;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;

/**
 * Small utility to play short WAV files. Tries classpath resource first, then looks under
 * deploy/sounds/ for a file name match. If the file isn't found or audio playback isn't available,
 * this class fails silently (logs to stderr).
 *
 * <p>Usage: place your file at src/main/resources/sounds/instant_win.wav or
 * deploy/sounds/instant_win.wav on the roboRIO/host, then call AudioPlayer.playInstantWin().
 */
public final class AudioPlayer {
  private static final String INSTANT_WIN_NAME = "instant_win.wav";
  private static final String INSTANT_WIN_CLASSPATH = "/sounds/" + INSTANT_WIN_NAME;

  private AudioPlayer() {}

  public static void playInstantWin() {
    playSound(INSTANT_WIN_CLASSPATH, INSTANT_WIN_NAME);
  }

  private static void playSound(String classpathResource, String fallbackFileName) {
    try {
      // Try classpath resource first
      InputStream res = AudioPlayer.class.getResourceAsStream(classpathResource);
      if (res != null) {
        try (AudioInputStream ais = AudioSystem.getAudioInputStream(new BufferedInputStream(res))) {
          Clip clip = AudioSystem.getClip();
          clip.open(ais);
          clip.start();
          return;
        }
      }

      // Fallback: look in deploy/sounds/<name>
      File f = new File("deploy/sounds/" + fallbackFileName);
      if (f.exists()) {
        try (AudioInputStream ais = AudioSystem.getAudioInputStream(f)) {
          Clip clip = AudioSystem.getClip();
          clip.open(ais);
          clip.start();
          return;
        }
      }

      System.err.println(
          "AudioPlayer: sound not found: "
              + classpathResource
              + " or deploy/sounds/"
              + fallbackFileName);
    } catch (Exception e) {
      // Don't throw from robot code; log to stderr for diagnostics.
      System.err.println("AudioPlayer: failed to play sound: " + e);
      e.printStackTrace();
    }
  }
}
