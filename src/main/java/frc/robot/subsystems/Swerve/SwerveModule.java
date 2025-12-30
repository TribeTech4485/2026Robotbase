// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Movement;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveModuleBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig.FeedForwardType;

public class SwerveModule extends SwerveModuleBase {

  private static final PIDConfig drivePIDF = new PIDConfig().set(Movement.Drive.P, Movement.Drive.I, Movement.Drive.D,
      Movement.Drive.S, Movement.Drive.V, Movement.Drive.A, Movement.maxWheelSpeed,
      Movement.maxWheelAccel, FeedForwardType.SimpleMotor);
  private static final PIDConfig turnPID = new PIDConfig().set(Movement.Turn.P, Movement.Turn.I, Movement.Turn.D);
  private static final SparkBaseConfig driveConfig = new SparkMaxConfig()
      .smartCurrentLimit(Constants.Swerve.driveAmps).inverted(true)
      .apply(new EncoderConfig().velocityConversionFactor(Movement.driveGearRatio)
          .positionConversionFactor(Movement.driveGearRatio * 50));
  private static final SparkBaseConfig turningConfig = new SparkMaxConfig()
      .smartCurrentLimit(Constants.Swerve.turnAmps)
      .idleMode(IdleMode.kBrake)
      .apply(new AbsoluteEncoderConfig().positionConversionFactor(Math.PI * 2)
          .inverted(true).velocityConversionFactor(Math.PI * 2));

  public SwerveModule(SparkMax driveMotor, SparkMax turningMotor, double turningOffset, String name) {
    super(driveMotor, turningMotor, turningOffset, name,
        driveConfig, turningConfig, drivePIDF, turnPID);
    breakerMaxAmps = 40;
  }
}