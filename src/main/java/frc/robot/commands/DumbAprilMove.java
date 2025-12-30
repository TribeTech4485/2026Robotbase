package frc.robot.Commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve.Movement.MovePID;
import frc.robot.SyncedLibraries.SystemBases.PhotonVisionBase;
import frc.robot.SyncedLibraries.SystemBases.PathPlanning.DistanceMoveCommand;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig;

public class DumbAprilMove extends Command {
  PhotonVisionBase photon;
  SwerveDriveBase driveBase;
  Distance distanceX;
  Distance distanceY;
  Angle angle;
  Command moveCommand;
  PIDConfig turnConfig = new PIDConfig().set(4, 0, 0.05); // p = 4
  PIDController turnController;
  final LinearVelocity maxSpeed = MetersPerSecond.of(1);
  final LinearAcceleration maxAccel = MetersPerSecondPerSecond.of(3);

  public DumbAprilMove(SwerveDriveBase drivetrain, PhotonVisionBase photonVision,
      Distance distanceX, Distance distanceY, Angle angle) {
    // addRequirements(drivetrain);
    driveBase = drivetrain;
    photon = photonVision;
    this.distanceX = distanceX;
    this.distanceY = distanceY;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Transform3d target = photon.mainTarget.bestCameraToTarget;
    photon.mainTarget.getSkew();
    turnController = new PIDController(turnConfig.P, turnConfig.I, turnConfig.D);
    turnController
        .setSetpoint(Units.degreesToRadians(-photon.mainTarget.yaw) - angle.in(Radians));
    System.out.println("Scanned apriltag at " + target.getMeasureX() + ", " + target.getMeasureY());
    moveCommand = new DistanceMoveCommand(driveBase,
        target.getMeasureX().minus(distanceX),
        target.getMeasureY().minus(distanceY).unaryMinus(),
        new PIDConfig().set(MovePID.P, MovePID.I, MovePID.D, maxSpeed, maxAccel),
        new PIDConfig().set(MovePID.P, MovePID.I, MovePID.D, maxSpeed, maxAccel),
        turnConfig);
    moveCommand.initialize();
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
    System.out.println("DumbAprilMove ended with interrupted = " + interrupted);
    // this.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return moveCommand.isFinished();
  }
}