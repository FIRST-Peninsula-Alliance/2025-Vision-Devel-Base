// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DeadReckoningPIDCommand extends Command {

  private final SwerveSubsystem  drivetrainSubsystem;

  private final PIDController pidControllerX = new PIDController(25., 0, 0);
   //Creates a new DeployIntakeCmd. 
  public DeadReckoningPIDCommand(SwerveSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  double distance = 50; 
  double currentPos = 0;
  @Override
  public void initialize() {
    pidControllerX.reset();
    pidControllerX.setSetpoint(Units.inchesToMeters(distance));
    pidControllerX.setTolerance(Units.inchesToMeters(2));
  }

  // Returns true when the command should end.
  @Override
  public void execute() {
    currentPos = drivetrainSubsystem.getCurrentPose();
    var xSpeed = pidControllerX.calculate(currentPos);
    SmartDashboard.putNumber("PID Out", xSpeed);
    SmartDashboard.putNumber("currentPos", currentPos);
    if (pidControllerX.atSetpoint() || currentPos > distance) {
      xSpeed = 0;
    }
    var zSpeed = 0;
    double omegaSpeed = 0;

    Translation2d translation = new Translation2d(0, xSpeed);

    drivetrainSubsystem.drive(translation, omegaSpeed, true);
  }
  
}
