package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.SyncedLibraries.SystemBases.AngleManipulatorBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig.FeedForwardType;

public class AlgaeArm extends AngleManipulatorBase {
  final Elevator elevator;

  public AlgaeArm(Elevator elevator) {
    super(
        new PIDConfig().set(Constants.AlgaeArm.P, Constants.AlgaeArm.I, Constants.AlgaeArm.D, Constants.AlgaeArm.S,
            Constants.AlgaeArm.V, Constants.AlgaeArm.A, Constants.AlgaeArm.G, Constants.AlgaeArm.maxVelocity,
            Constants.AlgaeArm.maxAcceleration, FeedForwardType.Arm));
    breakerMaxAmps = 40;
    addMotors(new SparkMax(Constants.Wirings.algaeArmMotor, SparkMax.MotorType.kBrushless));
    resetMotors();
    setBrakeMode(true);
    setBreakerMaxAmps(40);
    setCurrentLimit(Constants.AlgaeArm.amps);
    setEncoderMultiplier(Constants.AlgaeArm.positionMultiplier);
    invertSpecificMotors(false, 0);
    // 0 is straight out, positive is up
    setAngleBounds(Constants.AlgaeArm.positionBoundsMin, Constants.AlgaeArm.positionBoundsMax);
    // home().schedule();
    getMoveCommand().setEndOnTarget(false); // should already be done?

    // if not homed, set angle to top position
    if (Math.abs(getAngle().in(Radians)) <= 0.0001) {
      _setAngle(Constants.AlgaeArm.bootupAngle);
    }
    customSensor = getMotor(0).getForwardLimitSwitch()::isPressed;
    this.elevator = elevator;
    persistMotorConfig();
    Robot.onInits.add(
        new ConditionalCommand(new InstantCommand(this::holdPosition),
            new InstantCommand(),
            () -> !getMoveCommand().isScheduled()));
  }

  @Override
  public Command test() {
    return new SequentialCommandGroup(
        new InstantCommand(this::retract),
        new WaitUntilCommand(this::isAtPosition),
        new InstantCommand(() -> moveToPosition(0)),
        new WaitUntilCommand(this::isAtPosition),
        new InstantCommand(this::retract));
  }

  @Override
  public Command home() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> setCurrentLimit(1)),
        new InstantCommand(() -> setPower(0.1)),
        new WaitUntilCommand(customSensor),
        new InstantCommand(() -> _setAngle(Degrees.of(110))),
        new InstantCommand(this::fullStop),
        new InstantCommand(() -> setCurrentLimit(Constants.AlgaeArm.amps)),
        new InstantCommand(this::retract),
        new InstantCommand(() -> getMoveCommand().setEndOnTarget(false)));
  }

  @Override
  public void ESTOP() {
    fullStop();
    setBrakeMode(true);
  }

  public void retract() {
    moveToPosition(70);
  }

  public void positionGroundIntake() {
    moveToPosition(-35);
  }

  public void positionOut() {
    moveToPosition(0);
  }

  public void positionBarge() {
    moveToPosition(30);
  }

  public void holdPosition() {
    moveToPosition(getAngle().plus(getCurrentSpeed().times(Seconds.of(0.1))));
  }

  @Override
  public void moveToPosition(Angle position) {
    moveToPosition(position, false);
  }

  public void moveToPosition(double degrees) {
    moveToPosition(Degrees.of(degrees));
  }

  public void elevatorCheck() {
    Time lookAheadTime = Seconds.of(0.5);
    Distance elevatorPosition = elevator.getPosition().plus(elevator.getVelocity().times(lookAheadTime));
    if (elevatorPosition.compareTo(Feet.of(2)) < 0
        && getAngle().compareTo(Degrees.of(-50)) < 0) {
      retract();
    }

    if (getAngle().compareTo(maxPosition) > 0) {
      retract();
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    elevatorCheck();
  }
}
