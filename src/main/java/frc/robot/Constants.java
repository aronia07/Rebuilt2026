// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.util.PolynomialRegression;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static Optional<Alliance> alliance = DriverStation.getAlliance();
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
  public static class LightsConstants {
    public static int port = 0;
    public static int length = 12;

    public static enum LightsType {
      ENDGAME,
      CLIMB,
      SHOOTING,
      INTAKE,
      IDLE,
      DISABLED
    }

    public static class Colors {
      public static int[] RED = new int[] { 255, 0, 0 };
      public static int[] GREEN = new int[] { 0, 255, 0 };
      public static int[] BLUE = new int[] { 0, 0, 255 };
      public static int[] GOLD = new int[] { 175, 184, 6 };
      public static int[] MAGENTA = new int[] { 255, 0, 255 };
      public static int[] BRIGHT = new int[] { 234, 255, 48 };
      public static int[] OFF = new int[] {0, 0, 0};
    }
  }
  public static class ShooterConstants {
    public static double activeWaitingSpeed;
    public static double inactiveWaitingSpeed;
    public static PolynomialRegression hoodAngleInterpolation;
    public static PolynomialRegression shooterSpeedInterpolation;
    public static double distanceToHub;
    public static double passDistance;

    public static int shooterMotionMagicExpoK_V;
    public static int shooterMotionMagicExpoK_A;
    public static int shooterMotionMagicAccel;
    public static int shooterMotionMagicJerk;

    public static int SupplyCurrentLimit;
    public static int StatorCurrentLimit;

    public static int hoodMotorID;
    public static int shooterMotor1ID;
    public static int shooterMotor2ID;

    public static double[] shooterPID = {0, 0, 0};
    public static double[] shooterSVA = {0, 0, 0};
    public enum ShooterWantedState {
      IDLE,
      WAIT,
      PASS_SHOOT,
      HUB_SHOOT

    }
    public enum SystemState {
      IDLING,
      ACTIVE_WAITING,
      INACTIVE_WAITING,
      PASS_SHOOTING,
      HUB_SHOOTING
    }
  }

  public static class IntakeConstants {
    public static int intakeMotionMagicExpoK_V;
    public static int intakeMotionMagicExpoK_A;
    public static int intakeMotionMagicAccel;
    public static int intakeMotionMagicJerk;
    public static int SupplyCurrentLimit = 80;
    public static int StatorCurrentLimit = 80;
    public static int intakeMotorID;
    public static double intakingPosition;
    public static double intakingSpeed;
    public static double shootingPosition;
    public static double retractingPos;
    public static double[] intakePID = {0, 0, 0};
    public static double[] intakeSVA = {0, 0, 0};
    public enum IntakeWantedState {
      IDLE,
      INTAKE,
      SHOOT, 
      RETRACT,
    }
    public enum SystemState {
      IDLING,
      INTAKING,
      SHOOTING,
      RETRACTING
    }
  }  
  public static class TurretConstants {
    public static int turretMotionMagicExpoK_V;
    public static int turretMotionMagicExpoK_A;
    public static int turretMotionMagicAccel;
    public static int turretMotionMagicJerk;
    public static int SupplyCurrentLimit;
    public static int StatorCurrentLimit;
    public static int turretMotorID;
    public static int passAimPosition;
    public static int hubAimPosition;
    public static int trenchPresetPosition;
    public static double[] turretPID = {0, 0, 0};
    public static double[] turretSVA = {0, 0, 0};
    public enum TurretWantedState {
      IDLE,
      AIM,
      TRENCH_PRESET,
      CLOSE_PRESET
      
    }
    public enum SystemState {
      IDLING,
      PASS_AIMING,
      HUB_AIMING,
      TRENCH_PRESETTING,
      CLOSE_PRESETTING
    }
  }

  public static class FeederConstants {
    public static double feederIntakeSpeed;
    public static double feederShootSpeed;
    public static int feederMotionMagicExpoK_V;
    public static int feederMotionMagicExpoK_A;
    public static int feederMotionMagicAccel;
    public static int feederMotionMagicJerk;
    public static int SupplyCurrentLimit;
    public static int StatorCurrentLimit;
    public static int shootFeederMotorID;
    public static int intakeFeederMotorID;
    public static double[] feederPID = {0, 0, 0};
    public static double[] feederSVA = {0, 0, 0};
    public enum FeederWantedState {
      IDLE,
      INTAKE,
      SHOOT
    }
    public enum SystemState {
      IDLING,
      INTAKING,
      SHOOTING
    }
  }
  public static class LightsConstants {
    public static Distance spacing = Meters.of(1 / 60);   // (1 / 60) - 60 leds per 1m strip [Spacing: 1m/#ofLEDs]
    // Main LED Strip (sides)
    public static int main_port = 0;
    public static int main_length = 48;   // 48, 24 a side
    public static int main_brightness = 50;
    // Signal LED Sector (on shooter)
    //public static int signal_port = None;
    //public static int signal_length = 10;   // 10, 5 a side

    // public static enum LightsType {
    //     ENDGAME,
    //     CLIMB,
    //     SHOOTING,
    //     INTAKE,
    //     IDLE,
    //     DISABLED
    //   }

    public static class Colors {
        public static int[] RED = new int[] { 255, 0, 0 };
        public static int[] GREEN = new int[] { 0, 255, 0 };
        public static int[] BLUE = new int[] { 0, 0, 255 };
        public static int[] GOLD = new int[] { 175, 184, 6 };
        public static int[] MAGENTA = new int[] { 255, 0, 255 };
        public static int[] BRIGHT = new int[] { 234, 255, 48 };
      }

    // RGB Color Map
    // Not needed: use Color.k[colorname]
    public static Map<String, Color> RGBColors = Map.of( // Color Map
            "black", new Color(0, 0, 0),
            "white", new Color(255, 255, 255),
            "red", new Color(255, 0, 0),
            "green", new Color(0, 255, 0),
            "blue", new Color(0, 0, 255),
            "gold", new Color(175, 184, 6),
            "team_Gold", new Color(179, 134, 27),
            "yellow", new Color(255, 255, 0),
            "orange", new Color(255, 165, 0),
            "pink", new Color(255, 20, 147),
            "magenta", new Color(255, 0, 255),
            "bright", new Color(234, 255, 48));

    // GRB Color Map (Old LEDs)
    public static Map<String, Color> GRBColors = Map.of(
            "black", new Color(0, 0, 0),
            "white", new Color(255, 255, 255),
            "red", new Color(0, 255, 0),
            "green", new Color(100, 0, 0),
            "blue", new Color(0, 0, 100),
            "team_Gold", new Color(126, 235, 44),
            "yellow", new Color(255, 255, 0),
            "orange", new Color(165, 255, 0),
            "pink", new Color(20, 255, 147),
            "magenta", new Color(0, 100, 100));

    // GBR Color Map
    public static Map<String, Color> GBRColors = Map.of(
            "black", new Color(0, 0, 0),
            "white", new Color(255, 255, 255),
            "red", new Color(0, 0, 255),
            "green", new Color(255, 0, 0),
            "blue", new Color(0, 255, 0),
            "team_Gold", new Color(134, 27, 179),
            "yellow", new Color(255, 0, 255),
            "orange", new Color(165, 0, 255),
            "pink", new Color(20, 147, 255),
            "magenta", new Color(0, 255, 255));
    
    // BRG Color Map (New LED strip)
    public static Map<String, Color> BRGColors = Map.of(
            "black", new Color(0, 0, 0),                 // (0,0,0) → (0,0,0)
            "white", new Color(255, 255, 255),           // (255,255,255) → (255,255,255)
            "red", new Color(0, 255, 0),                 // RGB(255,0,0) → BRG(0,255,0)
            "green", new Color(0, 0, 255),               // RGB(0,255,0) → BRG(0,0,255)
            "blue", new Color(255, 0, 0),                // RGB(0,0,255) → BRG(255,0,0)
            "gold", new Color(6, 175, 184),              // RGB(175,184,6) → BRG(6,175,184)
            "team_Gold", new Color(27, 179, 134),        // RGB(179,134,27) → BRG(27,179,134)
            "yellow", new Color(0, 255, 255),            // RGB(255,255,0) → BRG(0,255,255)
            "orange", new Color(0, 255, 165),            // RGB(255,165,0) → BRG(0,255,165)
            "pink", new Color(147, 255, 20),             // RGB(255,20,147) → BRG(147,255,20)
            "magenta", new Color(255, 255, 0),           // RGB(255,0,255) → BRG(255,255,0)
            "bright", new Color(48, 234, 255)            // RGB(234,255,48) → BRG(48,234,255)
    );
  }
}
