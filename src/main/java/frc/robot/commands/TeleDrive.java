package frc.robot.Commands;

import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.AlgaeArm;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.AlgaeClaw;
import frc.robot.Subsystems.CoralManipulator;
import frc.robot.SyncedLibraries.Controllers;
import frc.robot.SyncedLibraries.SystemBases.TeleDriveCommandBase;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;

public class TeleDrive extends TeleDriveCommandBase {
  final Elevator elevator;
  final AlgaeArm algaeArm;
  final AlgaeClaw algaeClaw;
  final CoralManipulator coralManipulator;

  final Distance elevatorPositionMiddle;
  final Distance elevatorPositionRadius;

  final Angle armPositionMiddle;
  final Angle armPositionRadius;

  // Percentage from center to edge, once outside:
  // the power goes to reverse in opposite the direction
  final double powerControlMaxSafeMoveArm = 0.75;
  final double powerControlMaxSafeMoveEle = 0.75;
  final double powerControlReversePowerArm = 0.5;
  final double powerControlReversePowerEle = 0.5;

  public TeleDrive(SwerveDriveBase driveTrain, Controllers controllers, Elevator elevator, AlgaeArm algaeArm,
      AlgaeClaw algaeClaw, CoralManipulator coralManipulator) {
    super(driveTrain, controllers.Zero, controllers.One);
    this.elevator = elevator;
    this.algaeArm = algaeArm;
    this.algaeClaw = algaeClaw;
    this.coralManipulator = coralManipulator;

    elevatorPositionMiddle = (Constants.Elevator.positionBoundsMax.plus(Constants.Elevator.positionBoundsMin)).div(2);
    elevatorPositionRadius = elevatorPositionMiddle.minus(Constants.Elevator.positionBoundsMin);
    armPositionMiddle = (Constants.AlgaeArm.positionBoundsMax.plus(Constants.AlgaeArm.positionBoundsMin)).div(2);
    armPositionRadius = armPositionMiddle.minus(Constants.AlgaeArm.positionBoundsMin);

    swerveTrain.setFieldRelative(false);
    driveMode = DriveModes.ROTATION_SPEED;

    usePOV = false;

    this.controllers[1].LeftTrigger.and(this.controllers[1].LeftBumper)
        .onFalse(new InstantCommand(() -> elevator.moveToPosition(elevator.getPosition())))
        .onFalse(new InstantCommand(() -> algaeArm.moveToPosition(algaeArm.getAngle())));
  }

  @Override
  public void execute() {
    super.execute();

    // Right trigger = move to position
    if (controllers[1].LeftTrigger.and(controllers[1].LeftBumper).getAsBoolean()) {
      if (controllers[1].RightJoyMoved.getAsBoolean()) {
        elevator.moveToPosition(
            elevatorPositionMiddle.plus(elevatorPositionRadius.times(-controllers[1].getRightY())));
      } else {
        elevator.moveToPosition(elevator.getPosition());
      }

      if (controllers[1].LeftJoyMoved.getAsBoolean()) {
        Angle rotation = Radians.of(-Math.atan2(controllers[1].getLeftY(), Math.abs(controllers[1].getLeftX())));
        algaeArm.moveToPosition(rotation);
      } else {
        algaeArm.moveToPosition(algaeArm.getAngle());
      }
    }
  }
}
