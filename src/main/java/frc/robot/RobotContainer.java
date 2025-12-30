// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Autos;
import frc.robot.Commands.TeleDrive;
import frc.robot.Subsystems.CoralManipulator;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.LEDS;
import frc.robot.Subsystems.PhotonVision;
import frc.robot.Subsystems.AlgaeArm;
import frc.robot.Subsystems.AlgaeClaw;
import frc.robot.Subsystems.Controllers2025;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.SyncedLibraries.SystemBases.ControllerBase;
import frc.robot.SyncedLibraries.SystemBases.Estopable;

// 58 inches forward

public class RobotContainer {
  Controllers2025 controllers = new Controllers2025();
  ControllerBase drivCont = controllers.Zero;
  ControllerBase opCont = controllers.One;

  Drivetrain drivetrain = new Drivetrain();
  PhotonVision photon = new PhotonVision();
  PowerDistribution pdh = new PowerDistribution(20, ModuleType.kRev);
  AlgaeClaw algaeClaw = new AlgaeClaw();
  public Elevator elevator = new Elevator();
  AlgaeArm algaeArm = new AlgaeArm(elevator);
  CoralManipulator coralManipulator = new CoralManipulator();
  LEDS leds = new LEDS(elevator);

  TeleDrive teleDrive = new TeleDrive(drivetrain, controllers, elevator, algaeArm, algaeClaw, coralManipulator);
  HolonomicDriveController holoDrive = new HolonomicDriveController(
      new PIDController(Constants.Swerve.Movement.Holonomic.xP,
          Constants.Swerve.Movement.Holonomic.xI, Constants.Swerve.Movement.Holonomic.xD),
      new PIDController(Constants.Swerve.Movement.Holonomic.yP,
          Constants.Swerve.Movement.Holonomic.yI, Constants.Swerve.Movement.Holonomic.yD),
      drivetrain.getTurnController());

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();

    System.out.print("Creating autos... ");
    autoChooser.addOption("Drive out", Autos.driveOut(drivetrain, elevator, algaeArm));
    LinearVelocity speed = MetersPerSecond.of(1.5);
    autoChooser.addOption("Left auto",
        new SequentialCommandGroup(
            new RunCommand(() -> drivetrain.inputDrivingX_Y(
                speed.div(-2), speed.div(-2 / Math.sqrt(3)), RadiansPerSecond.zero())).withTimeout(0.75),
            Autos.stopDrive(drivetrain).withTimeout(0.5),
            Autos.aprilAutoFiltered(drivetrain, photon, teleDrive, elevator, algaeArm, coralManipulator, algaeClaw,
                elevator::positionAlgaeHigh, 11, 20)));
    autoChooser.addOption("Right (proccessor) auto",
        new SequentialCommandGroup(
            new RunCommand(() -> drivetrain.inputDrivingX_Y(
                speed.div(-2), speed.div(2 / Math.sqrt(3)), RadiansPerSecond.zero())).withTimeout(0.75),
            Autos.stopDrive(drivetrain).withTimeout(0.5),
            Autos.aprilAutoFiltered(drivetrain, photon, teleDrive, elevator, algaeArm, coralManipulator, algaeClaw,
                elevator::positionAlgaeHigh, 9, 22)));

    autoChooser.setDefaultOption("Center filtered",
        Autos.aprilAutoFiltered(drivetrain, photon, teleDrive, elevator, algaeArm, coralManipulator, algaeClaw,
            elevator::positionAlgaeLow, 10, 21, 7));

    autoChooser.addOption("Center auto",
        Autos.aprilAuto(drivetrain, photon, teleDrive, elevator, algaeArm, coralManipulator, algaeClaw,
            elevator::positionAlgaeLow));

    SmartDashboard.putData("Auto chooser", autoChooser);
    System.out.println("Done");
  }

  private void configureBindings() {
    teleDrive.setNormalTriggerBinds(); // MOST DRIVER CONTROLLS ARE HERE

    // drivCont.ESTOPCondition.onTrue(new InstantCommand(Estopable::KILLIT));
    /** Shift == coral mode */
    Trigger Shift = opCont.LeftTrigger;
    Trigger notShift = Shift.negate();
    opCont.RightTrigger.and(notShift)
        .onTrue(new InstantCommand(algaeClaw::outtake))
        .onFalse(new InstantCommand(algaeClaw::stop));
    opCont.RightTrigger.and(Shift)
        .onTrue(new InstantCommand(algaeClaw::plop))
        .onFalse(new InstantCommand(algaeClaw::stop));
    opCont.RightBumper
        .onTrue(new InstantCommand(algaeClaw::intake));

    // coral mode, with trigger
    opCont.PovUp.and(Shift)
        .onTrue(new InstantCommand(elevator::positionL4))
        .onTrue(new InstantCommand(algaeArm::retract));
    opCont.PovRight.and(Shift)
        .onTrue(new InstantCommand(elevator::positionL3))
        .onTrue(new InstantCommand(algaeArm::retract));
    opCont.PovLeft.and(Shift)
        .onTrue(new InstantCommand(elevator::positionL2))
        .onTrue(new InstantCommand(algaeArm::retract));
    opCont.PovDown.and(Shift)
        .onTrue(new InstantCommand(elevator::positionL1))
        .onTrue(new InstantCommand(algaeArm::retract));

    // retract all
    opCont.LeftStickPress
        .onTrue(new SequentialCommandGroup(
            new WaitCommand(0.1),
            new InstantCommand(elevator::retract)))
        .onTrue(new InstantCommand(algaeArm::retract));

    opCont.Y.onTrue(new InstantCommand(algaeArm::retract));
    opCont.RightStickPress
        .onTrue(new InstantCommand(elevator::holdPosition))
        .onTrue(new InstantCommand(algaeArm::holdPosition));

    // algae mode, no trigger
    opCont.PovUp.and(notShift)
        .onTrue(new InstantCommand(elevator::positionBarge))
        .onTrue(new InstantCommand(algaeArm::positionBarge));
    opCont.PovRight.and(notShift)
        .onTrue(new InstantCommand(elevator::positionAlgaeHigh))
        .onTrue(new InstantCommand(algaeArm::positionOut));
    opCont.PovLeft.and(notShift)
        .onTrue(new InstantCommand(elevator::positionAlgaeLow))
        .onTrue(new InstantCommand(algaeArm::positionOut));
    opCont.PovDown.and(notShift)
        .onTrue(new InstantCommand(elevator::positionAlgaeGround))
        .onTrue(new InstantCommand(algaeArm::positionGroundIntake));

    opCont.LeftBumper.and(notShift)
        .onTrue(new InstantCommand(elevator::positionProccessor))
        .onTrue(new InstantCommand(algaeArm::positionOut));

    // opCont.A
    // .onTrue(coralManipulator.comIntake());
    opCont.A.onTrue(coralManipulator.comIntake());
    opCont.B.onTrue(coralManipulator.comOuttake());

    opCont.Options // THE ONE LABELED OPTIONS
        .onTrue(new InstantCommand(() -> algaeArm._setAngle(Radian.of(0))));
    opCont.LeftBumper.and(opCont.Options) // THE ONE LABELED SHARE
        .and(Shift)
        .onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> elevator.setPower(-0.1, true)),
            new WaitUntilCommand(elevator.getCustomSensor()),
            new InstantCommand(elevator::stop),
            new InstantCommand(elevator::holdPosition)));

    opCont.Options.onTrue(new InstantCommand(() -> algaeArm.getEncoder(0).setPosition(0)));

    opCont.LeftBumper.and(Shift); // Manual smart control

    if (drivCont.isJoystick) {
      drivCont.buttons[12]
          .onTrue(new InstantCommand(elevator::retract))
          .onTrue(new InstantCommand(algaeArm::retract));

      drivCont.buttons[10] // Right one
          .onTrue(new InstantCommand(() -> elevator
              ._setPosition(Constants.Elevator.positionBoundsMin)));

    } else {
      // driver xbox
      drivCont.A
          .onTrue(new InstantCommand(elevator::retract))
          .onTrue(new InstantCommand(algaeArm::retract));

      // falling button
      final double emergencyMult = 4;
      drivCont.Y
          .onTrue(new InstantCommand(elevator::retract))
          .onTrue(new InstantCommand(algaeArm::retract))
          .onTrue(new InstantCommand(() -> elevator.setCurrentLimit(-1)))
          .onTrue(new InstantCommand(
              () -> elevator.getPIDConfig().maxLinearAcceleration = elevator
                  .getPIDConfig().maxLinearAcceleration
                  .times(emergencyMult)))
          .onTrue(new InstantCommand(
              () -> elevator.getPIDConfig().maxLinearVelocity = elevator.getPIDConfig().maxLinearVelocity
                  .times(emergencyMult)))
          .onFalse(new InstantCommand(
              () -> elevator.getPIDConfig().maxLinearAcceleration = Constants.Elevator.maxAcceleration))
          .onFalse(new InstantCommand(
              () -> elevator.getPIDConfig().maxLinearVelocity = Constants.Elevator.maxVelocity))
          .onFalse(new InstantCommand(() -> elevator.setCurrentLimit(Constants.Elevator.amps)));
      drivCont.B.onTrue(new InstantCommand(algaeArm::retract));
      drivCont.Start.onTrue(new InstantCommand(() -> Estopable.KILLIT(false)));
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public TeleDrive getTeleDrive() {
    return teleDrive;
  }
}