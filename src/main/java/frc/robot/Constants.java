package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public final class Constants {
  public static class Wirings {
    public static final int swerveModule1DriveMotor = 1;
    public static final int swerveModule1TurningMotor = 2;
    public static final int swerveModule2DriveMotor = 3;
    public static final int swerveModule2TurningMotor = 4;
    public static final int swerveModule3DriveMotor = 5;
    public static final int swerveModule3TurningMotor = 6;
    public static final int swerveModule4DriveMotor = 7;
    public static final int swerveModule4TurningMotor = 8;

    public static final int elevatorMotor1 = 9;
    public static final int elevatorMotor2 = 10;

    public static final int algaeClawMotor = 11;
    public static final int algaeArmMotor = 12;
    public static final int coralManipulatorMotor = 13;
    public static final int climberMotor = 14;
  }

  public static class Swerve {
    public static final int driveAmps = 35;
    public static final int turnAmps = 30;

    public static final double module1Offset = 0.4547673;
    public static final double module2Offset = 0.3288317;
    public static final double module3Offset = 0.2699317;
    public static final double module4Offset = 0.5163250;

    public static final String module1Name = "Front right";
    public static final String module2Name = "Front left";
    public static final String module3Name = "Back left";
    public static final String module4Name = "Back right";
    public static final Distance sideLength = Inches.of(29.75);

    public static class Movement {
      // bot speeds
      public static final LinearVelocity maxBotSpeed = MetersPerSecond.of(3.5);
      public static final LinearAcceleration maxBotAccel = MetersPerSecondPerSecond.of(15);

      // wheel speeds
      public static final LinearVelocity maxWheelSpeed = MetersPerSecond.of(15);
      public static final LinearAcceleration maxWheelAccel = MetersPerSecondPerSecond.of(30);

      // bot turn
      public static final AngularVelocity maxRotationSpeed = RotationsPerSecond.of(1);
      public static final AngularAcceleration maxRotationAccel = RotationsPerSecondPerSecond.of(1.5);

      // public static final double driveGearRatio = 1 / (10 * Math.PI * 15 / 50);
      // public static final double driveGearRatio = 1 / 6.75 * (Math.PI * 4 * 0.254);
      public static final double driveGearRatio = 1.8 / 2150 / 1.8 * 2;

      public static class Drive {
        public static final double P = 0.5; // 0.5
        public static final double I = 0;
        public static final double D = 0;
        public static final double S = -0.15;
        public static final double V = 2.18;
        public static final double A = 4; // 2.5
        public static final double[] PIDF = { P, I, D, S, V, A };
      }

      public static class Turn {
        public static final double P = 5; // 5
        public static final double I = 0;
        public static final double D = 0;
        public static final double[] PID = { P, I, D };
      }

      public static class BotTurn {
        public static final double P = 0; // 4 // TODO: re-add turning
        public static final double I = 0;
        public static final double D = 0.025; // 0.1 // 0.025
        // public static final double S = 0;
        // public static final double V = 0;
        // public static final double A = 0;
        // public static final double[] PIDF = { P, I, D, S, V, A };
      }

      public static class Holonomic {
        public static final double xP = 0.2;
        public static final double xI = 0;
        public static final double xD = xP / 50 / 2;

        public static final double yP = 0.2;
        public static final double yI = 0;
        public static final double yD = yP / 50 / 2;
      }

      public static class MovePID {
        public static final double P = 5;
        public static final double I = 0;
        public static final double D = 0.5;
      }
    }
  }

  public static class Elevator {
    public static final int amps = 40;
    public static final double positionMultiplier = 0.013335;
    public static final Distance positionBoundsMin = Inches.of(16); // TODO: If coral stuck 35 inches
    public static final Distance positionBoundsMax = Inches.of(78); // 79

    public static final double P = 10;
    public static final double I = 0;
    public static final double D = 0;
    public static final double S = 0;
    public static final double V = 9;
    public static final double G = 0.4; // .3
    public static final double A = 0.019;
    public static final double[] PIDF = { P, I, D, S, V, G, A };

    public static final LinearVelocity maxVelocity = MetersPerSecond.of(4.75);
    public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(1.4); // 2.5
  }

  public static class AlgaeClaw {
    public static final int currentInLimit = 15;
    // public static final int currentOutLimit = 30;
  }

  public static class AlgaeArm {
    public static final int amps = 35;
    // 2350 for 90 degrees to radians
    public static final double positionMultiplier = Math.PI / 2 / 23.5;
    public static final Angle positionBoundsMin = Degrees.of(-80);
    public static final Angle positionBoundsMax = Degrees.of(83);
    public static final Angle bootupAngle = Degrees.of(87);

    public static final double P = 5;
    public static final double I = 0;
    public static final double D = 01;
    public static final double S = 0.15;
    public static final double V = 1.8;
    public static final double G = 0.25;
    public static final double A = 0.001 / positionMultiplier;
    public static final double[] PIDF = { P, I, D, S, V, G, A };

    public static final AngularVelocity maxVelocity = RadiansPerSecond.of(20); // just acceleration limits
    public static final AngularAcceleration maxAcceleration = RadiansPerSecondPerSecond.of(5);
  }

  public static class CoralManipulator {
    public static final int currentLimit = 20;
  }

  public static class Climber {
    public static final int currentLimit = 40;
  }

  public static final boolean oldControls = false;
}
