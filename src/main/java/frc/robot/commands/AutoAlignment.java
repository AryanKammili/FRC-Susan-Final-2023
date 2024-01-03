// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.AlignmentCalculator;
import frc.lib.Limelight;
import frc.robot.Constants.PID_IDConstants.velocityCommand;
import frc.robot.Constants.PID_IDConstants.aimHood;
import frc.robot.Constants.PID_IDConstants.turnCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAlignment extends CommandBase {
  /** Creates a new AutoAlignment. */
  VisionSubsystem vision;
  Limelight limelight;

  DriveSubsystem driveSubsystem;
  ShooterSubsystem shooterSubsystem;

  AlignmentCalculator aligner;
  double heightDiff;
  double desiredAngle;
  double distance;
  double angle;
  double velocity;
  double rotation;
  double targetX;
  double targetY;
  double botX;
  double botY;

  PIDController driveController;
  PIDController aimController;
  PIDController velocityController;
  

  public AutoAlignment() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(vision.getCenterLimelight().hasTarget()){
      exitCommand();
    }

    // Drive PID controller //
    double kPt = turnCommand.kP;
    double kIt = turnCommand.kI;
    double kDt = turnCommand.kD;
  
    driveController = new PIDController(kPt, kIt, kDt);

    driveController.enableContinuousInput(-Math.PI, Math.PI);

    driveController.setTolerance(2);

    // Aim PID controller // 
    double kaP = aimHood.kP;
    double kaI = aimHood.kI;
    double kaD = aimHood.kD;
   
    aimController = new PIDController(kaP, kaI, kaD);
    
    aimController.setTolerance(2);


    // Velocity PID controller // 
    double kVP = velocityCommand.kP;
    double kVI = velocityCommand.kI;
    double kVD = velocityCommand.kD;
       
    velocityController = new PIDController(kVP, kVI, kVD);
        
    velocityController.setTolerance(2);
    

    // TODO: change make sure in meters //
    desiredAngle = -30;
    heightDiff = 0;

    targetX = vision.getCenterLimelight().getTarget().getTranslation().getX();
    targetY = vision.getCenterLimelight().getTarget().getTranslation().getY();
    botX = vision.getCenterLimelight().getPose().getTranslation().getX();
    botY = vision.getCenterLimelight().getPose().getTranslation().getY();

    aligner = new AlignmentCalculator(heightDiff, desiredAngle);

    // Calculates every part neccessary to move Susan to correct Posistion // 
    distance = aligner.calculateDistance(targetX, targetY, botX, botY);

    angle = aligner.calculateAngle(distance);

    velocity = aligner.calculateVelocity(angle, distance);

    rotation = aligner.calculateRotation(targetX, targetY, botX, botY);

    // Set the setpoints and resets so they don't go crazy // 
    velocityController.setSetpoint(velocity);
    
    velocityController.reset();

    aimController.setSetpoint(angle);

    aimController.reset();

    driveController.setSetpoint(rotation);

    driveController.reset();

    // If angle is too high for the hood or the velocity is too high than terminate the program  // 
    if(!shooterSubsystem.safe2MoveAimHood(angle)){
        new InstantCommand(() -> System.out.println("Aim angle is too high for the robot!"));
        exitCommand();
    }

    if(!vision.getCenterLimelight().hasTarget()){
        new InstantCommand(() -> System.out.println("Limelight can't detect object!"));
        exitCommand();
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){

    double driveCalc = driveController.calculate(vision.getCenterLimelight().getPose().getRotation().getDegrees(), rotation);
    driveSubsystem.autonDriveCommand(0, driveCalc);

    double aimCalc = aimController.calculate(shooterSubsystem.getAimDegrees(), angle);
    shooterSubsystem.setAimHood(aimCalc);

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

  public void exitCommand(){}
}
