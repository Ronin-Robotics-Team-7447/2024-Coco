// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.io.File;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;
  // DOUBLE CHECK MAX speed
  public double maximumSpeed = Units.feetToMeters(14.5);

  public SwerveSubsystem(File directory) {
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(
      13.71, 42);
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
      0.1016, 7.13, 42);
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(
      //    maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); 
    // ^^Heading correction should only be used while controlling the robot via angle.
    setupPathPlanner();
  }
  public void setupPathPlanner(){
    //AUTOBUILDER STUFF HERE
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
