// Weird bug: Error in this class that dissapears upon viewing
package frc.robot.Commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.AlgaeArm;
import frc.robot.Subsystems.AlgaeClaw;
import frc.robot.Subsystems.CoralManipulator;
import frc.robot.Subsystems.Elevator;
import frc.robot.SyncedLibraries.SystemBases.PhotonVisionBase;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;

public class Autos {
  // OLD
  public static Command coralPlace(SwerveDriveBase drivetrain, Elevator elevator,
      AlgaeArm algaeArm, CoralManipulator coralManipulator) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> algaeArm.moveToPosition(40)),
        new WaitCommand(0.5),
        new InstantCommand(elevator::positionL4),
        new InstantCommand(algaeArm::retract),
        new WaitCommand(0.5),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(
            MetersPerSecond.of(-1), MetersPerSecond.of(0), RadiansPerSecond.of(0)))
            .withTimeout(2))
        .alongWith(coralManipulator.comIntake())
        .andThen(_coralPlace(coralManipulator, elevator),
            new RunCommand(() -> drivetrain.inputDrivingX_Y(MetersPerSecond.of(0.75),
                MetersPerSecond.of(0), RadiansPerSecond.of(0))).withTimeout(2))
        .andThen(stopDrive(drivetrain))
        .andThen(new InstantCommand(algaeArm::retract),
            new InstantCommand(elevator::retract));
  }

  public static Command driveOut(SwerveDriveBase drivetrain, Elevator elevator,
      AlgaeArm algaeArm) {
    return new SequentialCommandGroup(
        new InstantCommand(algaeArm::retract),
        new InstantCommand(elevator::retract),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(
            MetersPerSecond.of(-1.25), MetersPerSecond.of(0), RadiansPerSecond.of(0))).withTimeout(2),
        stopDrive(drivetrain));
  }

  // OLD
  public static Command centerCoralAndAlgae(SwerveDriveBase drivetrain, Elevator elevator,
      AlgaeArm algaeArm, CoralManipulator coralManipulator) {
    return coralPlace(drivetrain, elevator, algaeArm, coralManipulator)
        .andThen(new InstantCommand());
  }

  // OLD
  private static Command _coralPlace(CoralManipulator coralManipulator, Elevator elevator) {
    return new WaitCommand(1)
        .andThen(coralManipulator.comOuttake())
        .andThen(new WaitCommand(0.5))
        .andThen(new InstantCommand(elevator::positionBarge))
        .andThen(new WaitCommand(0.5))
        .andThen(new InstantCommand(elevator::positionL4));
  }

  public static Command aprilAuto(SwerveDriveBase drivetrain, PhotonVisionBase photon, TeleDrive teleDrive,
      Elevator elevator, AlgaeArm algaeArm, CoralManipulator coralManipulator, AlgaeClaw algaeClaw,
      Runnable algaeRunnable) {
    final LinearVelocity maxSpeed = MetersPerSecond.of(1);
    final LinearAcceleration maxAccel = MetersPerSecondPerSecond.of(3);
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new PrintCommand("Starting April auto"),
            coralManipulator.comIntake(),
            // new InstantCommand(teleDrive::disable),
            new InstantCommand(algaeArm::retract),
            new InstantCommand(elevator::retract),
            new InstantCommand(() -> drivetrain.setSlowMode(false)),
            centerOnAprilTag(drivetrain, photon)),
        new InstantCommand(elevator::positionL4),
        new PrintCommand("Move Elevator"),
        new WaitCommand(1),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(-0.3, 0, 0)).withTimeout(0.85),
        new ParallelCommandGroup(
            stopDrive(drivetrain),
            coralManipulator.comOuttake()),
        new InstantCommand(elevator::positionBarge),
        new WaitCommand(0.35),
        new InstantCommand(elevator::positionL4),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(0.35, 0, 0)).withTimeout(0.9),
        // new DistanceMoveCommand(drivetrain, Feet.of(1), Feet.of(0),
        // new PIDConfig().set(MovePID.P, MovePID.I, MovePID.D, 0.0, 0.0, 0.0, maxSpeed,
        // maxAccel),
        // new PIDConfig().set(MovePID.P, MovePID.I, MovePID.D, 0.0, 0.0, 0.0, maxSpeed,
        // maxAccel),
        // new PIDConfig().set(4, 0, 0.05)),
        new InstantCommand(algaeRunnable),
        stopDrive(drivetrain),
        new PrintCommand("Finished April auto"),
        new InstantCommand(algaeArm::positionOut),
        new DumbAprilMove(drivetrain, photon, Feet.of(3.1), Inches.of(-8), Radians.zero())
            .withTimeout(3),
        new InstantCommand(algaeClaw::intake),
        // new RunCommand(() -> drivetrain.inputDrivingX_Y(-0.5, 0,
        // 0)).withTimeout(0.5),
        // stopDrive(drivetrain).withTimeout(0.5),
        // new RunCommand(() -> drivetrain.inputDrivingX_Y(0.5, 0, 0)).withTimeout(0.5),
        // new InstantCommand(elevator::retract),
        // new InstantCommand(algaeArm::retract),
        // stopDrive(drivetrain),
        // new InstantCommand(() ->
        // algaeArm.moveToPosition(Constants.AlgaeArm.positionBoundsMax.minus(Degrees.of(1)))),
        // new RunCommand(() -> drivetrain.inputDrivingX_Y(-0.4, 0, 0)).withTimeout(1),
        new InstantCommand(() -> elevator.adjustBy(Inches.of(7))),
        stopDrive(drivetrain))
        .finallyDo(() -> {
          // teleDrive.enable();
        });
  }

  public static Command aprilAutoFiltered(SwerveDriveBase drivetrain, PhotonVisionBase photon, TeleDrive teleDrive,
      Elevator elevator, AlgaeArm algaeArm, CoralManipulator coralManipulator, AlgaeClaw algaeClaw,
      Runnable algaeRunnable, int... ids) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new PrintCommand("Starting April auto"),
            coralManipulator.comIntake(),
            new InstantCommand(algaeArm::retract),
            new InstantCommand(elevator::retract),
            new InstantCommand(() -> drivetrain.setSlowMode(false)),
            centerOnAprilTagFiltered(drivetrain, photon, ids)),
        new InstantCommand(elevator::positionL4),
        new PrintCommand("Move Elevator"),
        new WaitCommand(1),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(-0.3, 0, 0)).withTimeout(0.85),
        new ParallelCommandGroup(
            stopDrive(drivetrain),
            coralManipulator.comOuttake()),
        new InstantCommand(elevator::positionBarge),
        new WaitCommand(0.35),
        new InstantCommand(elevator::positionL4),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(0.35, 0, 0)).withTimeout(0.9),
        new InstantCommand(algaeRunnable),
        stopDrive(drivetrain),
        new PrintCommand("Finished April auto"),
        new InstantCommand(algaeArm::positionOut),
        new DumbAprilMoveFiltered(drivetrain, photon, Feet.of(3.1), Inches.of(-8), Radians.zero(), ids)
            .withTimeout(3),
        new InstantCommand(algaeClaw::intake),
        new InstantCommand(() -> elevator.adjustBy(Inches.of(7))),
        stopDrive(drivetrain));
  }

  /** Goes up to apriltag, hits the wall to become parallel, then backs up */
  private static Command centerOnAprilTag(SwerveDriveBase drivetrain, PhotonVisionBase photon) {
    return new SequentialCommandGroup(
        new DumbAprilMove(drivetrain, photon, Feet.of(2.5), Inches.of(-3), Radians.zero()),
        stopDrive(drivetrain).withTimeout(0.125),
        new PrintCommand("Moved infront"),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(-0.5, 0, 0)).withTimeout(1),
        new PrintCommand("Wall slammed"),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(0.3, 0, 0)).withTimeout(0.7),
        stopDrive(drivetrain).withTimeout(0.5),
        new PrintCommand("Stopped"),
        new DumbAprilMove(drivetrain, photon, Feet.of(2.3), Inches.of(-3), Radians.zero())
            .withTimeout(3),
        new PrintCommand("Re-centered"))
        .finallyDo(() -> System.out.println("Finished centering?"));
  }

  /** Goes up to apriltag, hits the wall to become parallel, then backs up */
  private static Command centerOnAprilTagFiltered(SwerveDriveBase drivetrain, PhotonVisionBase photon, int... ids) {
    return new SequentialCommandGroup(
        new DumbAprilMoveFiltered(drivetrain, photon, Feet.of(2.5), Inches.of(-4), Radians.zero(), ids),
        stopDrive(drivetrain).withTimeout(0.125),
        new PrintCommand("Moved infront"),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(-0.5, 0, 0)).withTimeout(1),
        new PrintCommand("Wall slammed"),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(0.3, 0, 0)).withTimeout(0.7),
        stopDrive(drivetrain).withTimeout(0.5),
        new PrintCommand("Stopped"),
        new DumbAprilMoveFiltered(drivetrain, photon, Feet.of(2.3), Inches.of(-4), Radians.zero(), ids)
            .withTimeout(3),
        new PrintCommand("Re-centered"))
        .finallyDo(() -> System.out.println("Finished centering?"));
  }

  public static ParallelRaceGroup stopDrive(SwerveDriveBase drivetrain) {
    return new RunCommand(drivetrain::stop).withTimeout(1);
  }
}
