// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PID_IDConstants.velocityCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class VelocityCommand extends CommandBase {
  double velocity;
  PIDController velocityController;
  ShooterSubsystem shooterSubsystem;

  /** Creates a new VelocityCommand. */
  public VelocityCommand(ShooterSubsystem shooterSubsystem, double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.velocity = velocity;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double kVP = velocityCommand.kP;
    double kVI = velocityCommand.kI;
    double kVD = velocityCommand.kD;
       
    velocityController = new PIDController(kVP, kVI, kVD);
        
    velocityController.setTolerance(2);

    velocityController.setSetpoint(velocity);
    
    velocityController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocityCalc = velocityController.calculate(shooterSubsystem.getVelocity(), velocity);
    shooterSubsystem.setVelocity(velocityCalc);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
