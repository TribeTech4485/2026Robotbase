package frc.robot.Commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.Swerve.Movement.MovePID;
import frc.robot.SyncedLibraries.SystemBases.PathPlanning.DistanceMoveCommand;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig;

public class MoveToDistanceCommand extends DistanceMoveCommand {

  public MoveToDistanceCommand(SwerveDriveBase drivetrain, Distance xDistance, Distance yDistance, Angle angle,
      LinearVelocity maxSpeed, LinearAcceleration maxAccel) {
    super(drivetrain, xDistance, yDistance,
        new PIDConfig().set(MovePID.P, MovePID.I, MovePID.D, maxSpeed, maxAccel),
        new PIDConfig().set(MovePID.P, MovePID.I, MovePID.D, maxSpeed, maxAccel),
        new PIDConfig().set(4, 0, 0.05));
  }
}
