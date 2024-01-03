// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {
  /** Creates a new arcadeDrive. */
  DriveSubsystem driveBase;
  private final Supplier<Double> speedFunction, turnFunction;  
  
  public ArcadeDriveCommand(DriveSubsystem driveBase, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speedFunction = speedFunction;
    this.turnFunction = turnFunction;
    this.driveBase = driveBase;
    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = speedFunction.get();
    double turn = turnFunction.get();

    driveBase.setMotors(speed, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.setMotors(0 ,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}