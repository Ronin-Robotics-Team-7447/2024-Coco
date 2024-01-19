// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
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
    public SwerveSubsystem(
      SwerveDriveConfiguration driveCfg, 
      SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);
  }
  
  public void setupPathPlanner(){
    AutoBuilder.configureHolonomic(
      null, 
      null, 
      null, 
      null, 
      null, 
      null, 
      null);
      // ^^ unfinished
  }
  public Command getAutonomousCommand(String pathName, boolean setOdomToStart){
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

// SOMETHING MISSING HERE

    return AutoBuilder.followPath(path);
  }
// THREE types of drive command on YAGSL Example this is the FIRST one
//    * Command to drive the robot using translative values and heading as a setpoint.
  public Command driveCommand(
    DoubleSupplier translationX, 
    DoubleSupplier translationY, 
    DoubleSupplier headingX,
    DoubleSupplier headingY)
    {    
      return run(() -> {
        double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
        double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
        // Make the robot move
        driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                        headingX.getAsDouble(),
                                                                        headingY.getAsDouble(),
                                                                        swerveDrive.getYaw().getRadians(),
                                                                        swerveDrive.getMaximumVelocity()));
      });
      }

      public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation)
      {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
        return run(() -> {
          // Make the robot move
          driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                                                                          translationY.getAsDouble(),
                                                                          rotation.getAsDouble() * Math.PI,
                                                                          swerveDrive.getYaw().getRadians(),
                                                                          swerveDrive.getMaximumVelocity()));
        });
      }
  
  public void drive(Translation2d translation, double rotation, boolean fieldRelative){
    swerveDrive.drive(translation,rotation,fieldRelative,false);
  }

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }
  
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic(){
  }

  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading()
  {
    return swerveDrive.getYaw();
  }

  // this takes chassis speed inputs from two joystick CHECK WITH AMY
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(
      xInput,yInput,headingX,headingY,getHeading().getRadians(),maximumSpeed);
  }

  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock()
  {
    swerveDrive.lockPose();
  }

  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  // FAKE vision reading? 

}
