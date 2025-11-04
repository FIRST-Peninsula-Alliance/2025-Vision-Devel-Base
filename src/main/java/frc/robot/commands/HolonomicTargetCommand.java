package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionTestSubsystem;

public class HolonomicTargetCommand extends Command {
  
  private final SwerveSubsystem drivetrainSubsystem;
  private final VisionTestSubsystem limelightCamera;

  private final PIDController pidControllerZ = new PIDController(1.3, 0, 0);
  private final PIDController pidControllerX = new PIDController(1.5, 0, 0);
  private final PIDController pidControllerOmega = new PIDController(.5, 0, 0);

  public HolonomicTargetCommand(SwerveSubsystem drivetrainSubsystem, VisionTestSubsystem limelightCamera) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.limelightCamera = limelightCamera;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    pidControllerX.reset();
    pidControllerZ.reset();
    pidControllerOmega.reset();

    pidControllerZ.setSetpoint(Units.inchesToMeters(40)); // Move forward/backwork to keep 36 inches from the target
    pidControllerZ.setTolerance(Units.inchesToMeters(2.5));

    pidControllerX.setSetpoint(0); // Move side to side to keep target centered
    pidControllerX.setTolerance(Units.inchesToMeters(2));

    pidControllerOmega.setSetpoint(Units.degreesToRadians(0)); // Rotate the keep perpendicular with the target
    pidControllerOmega.setTolerance(Units.degreesToRadians(1));

  }

  @Override
  // TODO: We should never communicate through smartdashboard imo.
  public void execute() {
    double[] results = {SmartDashboard.getNumber("targetposeX", 0),     // [0]
                        SmartDashboard.getNumber("targetposeY", 0),     // [1]
                        SmartDashboard.getNumber("targetposeZ", 0),     // [2]
                        SmartDashboard.getNumber("targetposeRoll", 0),  // [3]
                        SmartDashboard.getNumber("targetposePitch", 0), // [4]
                        SmartDashboard.getNumber("targetposeYaw", 0)};  // [5] get most recent pipeline result
    if (results.length > 0) { 

      // X - left and right of camera center (in meters)
      // Y - above below camera center (in meters)
      // Z - distance from camera (in meters)

      // pitch  -positive is upwards camera relative (in degrees)
      //        

      // roll   -positive is clockwise in cameraview (in degrees)
      //        

      // yaw    - Z-axis - positive indicates counterclockwise tag when viewed from above (in degrees)


      
      
      // Handle distance to target
      var distanceFromTarget = results[2];
      var zSpeed = pidControllerZ.calculate(distanceFromTarget);
          zSpeed *= 1.1; //reduce travel speed
      if (pidControllerZ.atSetpoint()) {
        zSpeed = 0;
      }

      // Handle alignment side-to-side
      var targetX = results[0];
      var xSpeed = pidControllerX.calculate(targetX);
          xSpeed *= 1.1; //reduce strafing speed
      if (pidControllerX.atSetpoint()) {
        xSpeed = 0;
      }

      // Handle rotation using target Yaw/Z rotation
      var targetYaw = results[5];
      var omegaSpeed = pidControllerOmega.calculate(targetYaw);
          omegaSpeed *= 0.5; //reduce rotation speed
      if (pidControllerOmega.atSetpoint()) {
        omegaSpeed = 0;
      }
        Translation2d translation = new Translation2d(-zSpeed, xSpeed);
      drivetrainSubsystem.drive(translation, -omegaSpeed, false);
    } else {
      drivetrainSubsystem.stop();
    }
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
