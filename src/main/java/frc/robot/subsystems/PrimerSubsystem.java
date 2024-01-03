// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallHandlerConstants;

public class PrimerSubsystem extends SubsystemBase {
  /** Creates a new BallHandler. */
  private WPI_TalonSRX hopper;
  private WPI_TalonSRX sucker;

  public PrimerSubsystem() {
    hopper = new WPI_TalonSRX(BallHandlerConstants.K_HOPPER);
    sucker = new WPI_TalonSRX(BallHandlerConstants.K_SUCKER);

    configMotor(hopper);
    configMotor(sucker);

    sucker.setInverted(true);
    hopper.setInverted(true);
  }

  public void ball2Shooter(){
    hopper.set(BallHandlerConstants.K_HOPPER_SPEED);
    sucker.set(BallHandlerConstants.K_SUCKER_SPEED);
  }

  public void exitShooter(){
    hopper.set(0);
    sucker.set(-BallHandlerConstants.K_SUCKER_SPEED);
  }

  public void offBallHandler(){
    hopper.set(0);
    sucker.set(0);
  }

  public void configMotor(WPI_TalonSRX motor){
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}