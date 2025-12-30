package frc.robot.Commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.List;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.SyncedLibraries.SystemBases.PhotonVisionBase;
import frc.robot.SyncedLibraries.SystemBases.PathPlanning.TrajectoryMoveCommand;
import frc.robot.SyncedLibraries.SystemBases.Utils.BackgroundTrajectoryGenerator;

public class MoveToDistanceApriltag extends Command {
  /** Creates a new MoveToDistanceApriltag. */
  Drivetrain driveBase;
  PhotonVisionBase photon;
  double desiredX;
  double desiredY;
  double desiredTheta;
  boolean fieldRelative;
  TrajectoryMoveCommand moveCommand;
  HolonomicDriveController holoDrive;

  /**
   * Constructs a new MoveToDistanceApriltag command.
   * This command uses a holonomic drive controller and photon vision to move the
   * robot to a specified position and orientation relative to an AprilTag. Make
   * sure that the distance is negative as the robot likely can't move through the
   * tag.
   * 
   *
   * @param drivetrain     The drivetrain subsystem used to control the robot's
   *                       movement.
   * @param holoController The holonomic drive controller used for precise
   *                       movement control.
   * @param photonVision   The photon vision system used for detecting and
   *                       tracking AprilTags.
   * @param distanceX      The desired distance in the X direction
   *                       (forward/backward).
   * @param distanceY      The desired distance in the Y direction (left/right).
   * @param angle          The desired orientation angle compared to the tag
   *                       (typically 0).
   */
  public MoveToDistanceApriltag(Drivetrain drivetrain, HolonomicDriveController holoController,
      PhotonVisionBase photonVision, Distance distanceX,
      Distance distanceY, Angle angle) {
    // addRequirements(drivetrain);

    driveBase = drivetrain;
    photon = photonVision;
    desiredX = distanceX.in(Meters);
    desiredY = distanceY.in(Meters);
    desiredTheta = angle.in(Radians);
    holoDrive = holoController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (photon.timeSinceLastSeen.get() > 1) {
      cancel();
      System.out.println("No target found");
      return;
    }
    Transform3d apriltag = photon.mainTarget.bestCameraToTarget.inverse();
    Translation2d targetTranslation = new Translation2d(
        apriltag.getX() + desiredX, apriltag.getY() - desiredY);
    BackgroundTrajectoryGenerator trajectory = new BackgroundTrajectoryGenerator(
        new Pose2d(),
        new Pose2d(targetTranslation,
            apriltag.getRotation().toRotation2d()
                .plus(Rotation2d.fromRadians(desiredTheta + (Math.PI / 2 * 0)))),
        List.of(targetTranslation.times(0.5)),
        MetersPerSecond.of(1), MetersPerSecondPerSecond.of(5));
    moveCommand = new TrajectoryMoveCommand(trajectory, holoDrive, driveBase, true)
        .setStopTimeMultiplier(0.8);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    moveCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    moveCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (moveCommand == null) {
      return false;
    }
    return moveCommand.isFinished();
  }
}
