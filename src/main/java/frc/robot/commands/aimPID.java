// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PID_IDConstants.aimHood;
import frc.robot.subsystems.ShooterSubsystem;

public class aimPID extends CommandBase {
  /** Creates a new shooterPID. */
  PIDController aimPID; 
  double aimSetpoint;
  ShooterSubsystem shooter;
  private double aimCalc;
  

  public aimPID(ShooterSubsystem shooter, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    aimSetpoint = setPoint;
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double kaP = aimHood.kP;
    double kaI = aimHood.kI;
    double kaD = aimHood.kD;

    aimPID = new PIDController(kaP, kaI, kaD);

    aimPID.setSetpoint(aimSetpoint);

    aimPID.setTolerance(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    aimCalc = aimPID.calculate(shooter.getAimDegrees(), aimSetpoint);

    shooter.driveHood(0, aimCalc);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.driveHood(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}