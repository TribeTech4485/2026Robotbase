// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Wirings;
import frc.robot.Constants.Swerve.Movement;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig;

public class Drivetrain extends SwerveDriveBase {
	public static final TrapezoidProfile.Constraints driveConstraints = new TrapezoidProfile.Constraints(
			Movement.maxBotSpeed.in(MetersPerSecond), Movement.maxBotAccel.in(MetersPerSecondPerSecond));

	public Drivetrain() {
		super((Swerve.sideLength),
				(Swerve.sideLength),
				new SwerveModule[] {
						new SwerveModule(
								new SparkMax(Wirings.swerveModule2DriveMotor, MotorType.kBrushless),
								new SparkMax(Wirings.swerveModule2TurningMotor, MotorType.kBrushless),
								Swerve.module2Offset, Swerve.module2Name), // front left, 3, 4

						new SwerveModule(
								new SparkMax(Wirings.swerveModule1DriveMotor, MotorType.kBrushless),
								new SparkMax(Wirings.swerveModule1TurningMotor, MotorType.kBrushless),
								Swerve.module1Offset, Swerve.module1Name), // front right 1, 2

						new SwerveModule(
								new SparkMax(Wirings.swerveModule3DriveMotor, MotorType.kBrushless),
								new SparkMax(Wirings.swerveModule3TurningMotor, MotorType.kBrushless),
								Swerve.module3Offset, Swerve.module3Name), // back left 5, 6

						new SwerveModule(
								new SparkMax(Wirings.swerveModule4DriveMotor, MotorType.kBrushless),
								new SparkMax(Wirings.swerveModule4TurningMotor, MotorType.kBrushless),
								Swerve.module4Offset, Swerve.module4Name) // back right 7, 8
				// drive motors are odd, turning motors are even
				},
				// Swerve.Movement.BotTurn.PIDF,
				new PIDConfig().set(Movement.BotTurn.P, Movement.BotTurn.I, Movement.BotTurn.D,
						Movement.maxRotationSpeed, Movement.maxRotationAccel),
				Movement.maxBotSpeed,
				Movement.maxBotAccel);
		m_gyro.setAngleAdjustment(0);
	}

	@Override
	public void periodic() {
		super.periodic();
		try {
			SmartDashboard.putString("Current command", getCurrentCommand().toString());
		} catch (Exception e) {
			SmartDashboard.putString("Current command", "null");
		}
	}
}