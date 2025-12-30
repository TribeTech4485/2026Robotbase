package frc.robot.Subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class CoralManipulator extends ManipulatorBase {
  ColorSensorV3 colorSensor;
  boolean sensed = false;
  int counter = 0;

  public CoralManipulator() {
    breakerMaxAmps = 30;
    addMotors(new SparkMax(Constants.Wirings.coralManipulatorMotor, SparkMax.MotorType.kBrushless));
    setBrakeMode(true);
    setCurrentLimit(Constants.CoralManipulator.currentLimit);
    colorSensor = new ColorSensorV3(Port.kMXP);
    customSensor = () -> sensed;
    persistMotorConfig();

    // on init, if there is a coral, re-seat it
    // Robot.onInits.add(new ConditionalCommand(comIntake(),
    // new InstantCommand(), customSensor));
  }

  @Override
  public Command test() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> setPower(1)),
        new WaitCommand(1),
        new InstantCommand(() -> setPower(-1)),
        new WaitCommand(1),
        new InstantCommand(() -> setPower(0)));
  }

  public void intake() {
    setPower(1);

  }

  public void outtake() {
    setPower(-1);
  }

  public Command comIntake() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> setPower(-0.25)),
        new PrintCommand("Started"),
        new WaitUntilCommand(customSensor),
        new PrintCommand("Seen"),
        new InstantCommand(() -> setPower(-0.125)),
        new WaitUntilCommand(() -> !customSensor.getAsBoolean()),
        new PrintCommand("Unseen"),
        new InstantCommand(() -> setPower(0.075)),
        new WaitUntilCommand(customSensor),
        new InstantCommand(() -> stop()))
        .withTimeout(10).andThen(
            new InstantCommand(() -> stop()));
  }

  public Command comOuttake() {
    return new SequentialCommandGroup(new InstantCommand(() -> outtake()),
        new ParallelRaceGroup( // either 2 seconds, or 0.5 seconds after coral leaving
            new WaitCommand(2),
            new SequentialCommandGroup(
                new WaitUntilCommand(customSensor),
                new WaitUntilCommand(() -> !customSensor.getAsBoolean()),
                new WaitCommand(0.75))),
        new InstantCommand(() -> stop()));
  }

  @Override
  public void ESTOP() {
    setBrakeMode(false);
    fullStop();
  }

  @Override
  public void periodic() {
    super.periodic();
    if (counter++ == 20) {
      counter = 0;
      sensed = colorSensor.getProximity() > 250;
    }

    SmartDashboard.putNumber("Coral sensor distance", colorSensor.getProximity());
    SmartDashboard.putBoolean("Coral sened", customSensor.getAsBoolean());
  }

  @Override
  public void stopCommand() {
    stop();
  }
}
