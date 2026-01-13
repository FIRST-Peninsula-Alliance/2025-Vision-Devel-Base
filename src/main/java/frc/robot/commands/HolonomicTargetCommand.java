package frc.robot.commands;

import org.photonvision.PhotonCamera;

import com.fasterxml.jackson.databind.util.RootNameLookup;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionTestSubsystem;
import java.lang.Math;
import java.util.ArrayDeque;

public class HolonomicTargetCommand extends Command {

  private final SwerveSubsystem drivetrainSubsystem;
  private final VisionTestSubsystem limelightCamera;
  public Rotation2d rotationObj;

  private final PIDController pidControllerZ = new PIDController(1.0, 0, 0);
  private final PIDController pidControllerX = new PIDController(1.0, 0, 0);
  private final PIDController pidControllerOmega = new PIDController(0.3, 0, 0.05);

  public HolonomicTargetCommand(SwerveSubsystem drivetrainSubsystem, VisionTestSubsystem limelightCamera) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.limelightCamera = limelightCamera;

    addRequirements(drivetrainSubsystem);
  }

  double targetdist = 50;

  
  double[] targetData;

  class Target {
    double x; /// panning left to right
    double y; /// up to down
    double z; /// forward and backward

    double pitch;
    double yaw;
    double roll;

    // We are using radians here. Do not forget.
    double getHorizantalTargetAngle() {
      return Math.atan2(this.x, this.z);
    }
    double getVerticalTargetAngle() {
      return Math.atan2(this.y, this.z);
    }
  };
  class Robot {
    double x;
    double y;

    // We are using radians here. Do not forget.
    double rotation = 0;
    double startingRot = 0;
    double setpoint = 0;
    PIDController rotPID = new PIDController(1, 0, 0);

    void setupRotPID(double setpointRot) {
      // - pi because camera on backside
      startingRot = drivetrainSubsystem.getYaw2d().getRadians()-Math.PI;
      this.setpoint = setpointRot;
      this.rotPID.setSetpoint(this.setpoint);
      this.rotPID.setTolerance(0.1);
    }
    double rotPIDIterate() {
      // - pi because camera is on backside
      double robotAngle = drivetrainSubsystem.getYaw2d().getRadians()-Math.PI;
      return rotPID.calculate(robotAngle-setpoint);
    }
  };

  Robot controller = new Robot();
  Target target = new Target();

  /*@Override
  public void initialize() {
    super.initialize();
    pidControllerX.reset();
    pidControllerZ.reset();
    pidControllerOmega.reset();
    pidControllerZ.setSetpoint(Units.inchesToMeters(targetdist)); // Move forward/backwork to keep distance from the
                                                                  // target
    pidControllerZ.setTolerance(Units.inchesToMeters(5));

    pidControllerX.setSetpoint(0); // Move side to side to keep target centered
    pidControllerX.setTolerance(Units.inchesToMeters(2));

    pidControllerOmega.setSetpoint(Units.degreesToRadians(0)); // Rotate the keep perpendicular with the target
    pidControllerOmega.setTolerance(Units.degreesToRadians(1));

  }

  @Override
  // TODO: We should never communicate through smartdashboard imo.
  public void execute() {
    double[] results = limelightCamera.getVisionData();
    // requires full 3d targeting to be enabled in limelight dashboard
    // [5] get most recent pipeline result
    if (results.length == 6) {

      // X - left and right of camera center (in meters)
      // Y - above below camera center (in meters)
      // Z - distance from camera (in meters)

      // pitch -positive is upwards camera relative (in degrees)
      //

      // roll -positive is clockwise in cameraview (in degrees)
      //

      // yaw - Z-axis - positive indicates counterclockwise tag when viewed from above
      // (in degrees)

      // Handle distance to target g
      var distanceFromTarget = limelightCamera.getCamTrigDist();
      var zSpeed = pidControllerZ.calculate(distanceFromTarget);
      SmartDashboard.putNumber("forwardspeed", zSpeed);
      zSpeed *= 1; // reduce travel speed
      if (pidControllerZ.atSetpoint()) {
        zSpeed = 0;
      }
      System.out.println(distanceFromTarget);

      // Handle alignment side-to-side
      var targetZ = results[1];
      var targetX = results[0];
      var xSpeed = pidControllerX.calculate(targetX);
      xSpeed *= 1; // reduce strafing speed
      if (pidControllerX.atSetpoint()) {
        xSpeed = 0;
      }

      var targetYaw = results[5];
      Rotation2d rotation = drivetrainSubsystem.getYaw2d();
      double currentRotation = rotation.getDegrees();
      double adjustedRotation = Math.atan2(targetX, targetZ);
      // Handle rotation using target Yaw/Z rotation
      // TODO: fix rotation

      var unfactoredZ = Units.inchesToMeters(-targetdist) * Math.cos(Math.toRadians(adjustedRotation));
      var unfactoredX = Units.inchesToMeters(-targetdist) * Math.sin(Math.toRadians(adjustedRotation));
      // TODO: If buggy, remove this
      xSpeed = pidControllerX.calculate(unfactoredX) + pidControllerX.calculate(targetX);
      zSpeed = pidControllerZ.calculate(unfactoredZ) - pidControllerZ.calculate(distanceFromTarget);
      // end todo

      double targetRot = Math.toDegrees(Math.atan2(targetX, targetZ));
      double currentRot = rotation.getDegrees();
      if (currentRot != targetRot) {
        var omegaSpeed = pidControllerOmega.calculate(currentRot-targetRot);
        omegaSpeed *= 1; // reduce rotation speed
        if (pidControllerOmega.atSetpoint()) {
          omegaSpeed = 0;
        }

        Translation2d translation = new Translation2d(0 * zSpeed, 0 * xSpeed);
        Translation2d origin = new Translation2d(0, 0);
        translation = translation.rotateAround(origin, rotation);

        drivetrainSubsystem.drive(translation, omegaSpeed, true);
      }

    } else {
      drivetrainSubsystem.stop();
    }
  }
  */

  /*@Override
  public void execute() {

    double rotSpeed = 0;

    if (shouldSearch) {
      targetData = limelightCamera.getVisionData();

      if (targetData.length == 6) {
        shouldSearch = false;
        target.x = targetData[0];
        target.z = targetData[1];
        target.yaw = targetData[5];
        controller.setupRotPID(target.getHorizantalTargetAngle()); // target.getHorizantalTargetAngle()
      }
    } else {
      targetData = limelightCamera.getVisionData();

      if (targetData.length == 6) {
        shouldSearch = false;
        target.x = targetData[0];
        target.z = targetData[1];
        target.yaw = targetData[5];
        controller.setupRotPID(target.getHorizantalTargetAngle()); // target.getHorizantalTargetAngle()
      }
      if (!(controller.rotPID.atSetpoint())) {
        rotSpeed = controller.rotPIDIterate();
      } else {
        shouldSearch = true;
      }
      
    }

    Translation2d go = new Translation2d();
    drivetrainSubsystem.drive(go, rotSpeed, true);
  }*/

  double[] previousTargetData = {};
  double[] currentTargetData = {};
  PIDController angleSolver = new PIDController(0.2, 0, 0.05);
  double error = 0;
  double lock = 0;
  double progress = 0;
  double difference = 0;
  boolean shouldSearch = true;
  

  @Override
  public void execute()
  {
    
    if (shouldSearch) {
      currentTargetData = limelightCamera.getVisionData();
      if (currentTargetData.length == 6) {
        shouldSearch = false;
        error = Math.atan2(currentTargetData[0], currentTargetData[1]);
        lock = drivetrainSubsystem.getYaw2d().getRadians();
      }
    } else {
      progress = drivetrainSubsystem.getYaw2d().getRadians() - lock;
    }
    angleSolver.setSetpoint(0);
    angleSolver.setTolerance(0.1);
    difference = progress-error;
    double rotate = angleSolver.calculate(difference);

    previousTargetData = currentTargetData;

    if (angleSolver.atSetpoint()) {
      shouldSearch = true;
      return;
    }
    Translation2d go = new Translation2d();
    drivetrainSubsystem.drive(go, rotate, true);
  }

  @Override
  public boolean isFinished() {
    return super.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

}
