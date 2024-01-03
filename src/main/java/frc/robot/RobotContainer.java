// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.aimPID;
// import frc.robot.commands.PrimeShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private DriveSubsystem m_robotDrive;
  private ShooterSubsystem m_robotShooter;
  private PrimerSubsystem m_robotPrimer;
  private IntakeSubsystem m_robotIntake;
  private final CommandXboxController utilityController;
  private final CommandXboxController driveController;
  public static boolean runMotor;

  
  
  public RobotContainer() {
    runMotor = false;
    utilityController = new CommandXboxController(1);
    driveController = new CommandXboxController(0);
    
    m_robotDrive = new DriveSubsystem();
    m_robotShooter = new ShooterSubsystem();
    m_robotPrimer = new PrimerSubsystem();
    m_robotIntake = new IntakeSubsystem();

    m_robotDrive.setDefaultCommand(new ArcadeDriveCommand(m_robotDrive, 
    () -> applyDeadzone(-driveController.getLeftY()) , 
    () -> applyDeadzone(driveController.getRightX()) * -1
    ));

    configureBindings();
  }

  private void configureBindings() {
    utilityController.rightTrigger().onTrue(new InstantCommand(() -> m_robotIntake.grab()));
    utilityController.rightTrigger().onFalse(new InstantCommand(() -> m_robotIntake.stopIntake()));

    utilityController.leftTrigger().onTrue(new InstantCommand(() -> m_robotIntake.retract()));
    utilityController.leftTrigger().onFalse(new InstantCommand(() -> m_robotIntake.stopIntake()));

    utilityController.rightBumper().onTrue(new InstantCommand(() -> m_robotPrimer.ball2Shooter()));
    utilityController.rightBumper().onFalse(new InstantCommand(() -> m_robotPrimer.offBallHandler()));

    utilityController.leftBumper().onTrue(new InstantCommand(() -> m_robotPrimer.exitShooter()));
    utilityController.leftBumper().onFalse(new InstantCommand(() -> m_robotPrimer.offBallHandler()));

    utilityController.povDown().onTrue(new InstantCommand(() -> m_robotShooter.setAimHood(0.20)));
    utilityController.povDown().onFalse(new InstantCommand(() -> m_robotShooter.setAimHood(0)));

    utilityController.povUp().onTrue(new InstantCommand(() -> m_robotShooter.setAimHood(-0.20)));
    utilityController.povUp().onFalse(new InstantCommand(() -> m_robotShooter.setAimHood(0)));

    utilityController.povRight().onTrue(new InstantCommand(() -> m_robotShooter.setRotaterHood(0.20)));
    utilityController.povRight().onFalse(new InstantCommand(() -> m_robotShooter.setRotaterHood(0)));

    utilityController.povLeft().onTrue(new InstantCommand(() -> m_robotShooter.setRotaterHood(-0.20)));
    utilityController.povLeft().onFalse(new InstantCommand(() -> m_robotShooter.setRotaterHood(0)));

    utilityController.a().onTrue(new InstantCommand(() -> m_robotShooter.runShooter()));
    utilityController.a().onFalse(new InstantCommand(() -> m_robotShooter.offShooter()));

    utilityController.b().onTrue(new aimPID(m_robotShooter, -20))
    .onFalse(new InstantCommand(() -> m_robotShooter.setAimHood(0)));
  
  }


  public Command getAutonomousCommand() {
    return null;
  }

  public double applyDeadzone(double input){
    if(-0.1 < input && input < 0.1){
      return 0;}

    else{
      return input;}

  }
}