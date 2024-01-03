// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private WPI_TalonSRX aimHood;
  private WPI_TalonFX shooterRight;
  private WPI_TalonFX shooterLeft;
  private WPI_TalonSRX rotaterHood;

  private double kTicks2Degree;

  /** Creates a new FlyWheel. */
  public ShooterSubsystem() {

    aimHood = new WPI_TalonSRX(ShooterConstants.K_AIMHOOD);
    rotaterHood = new WPI_TalonSRX(ShooterConstants.K_ROTATERHOOD);

    shooterRight = new WPI_TalonFX(ShooterConstants.K_SHOOTERRIGHT);
    shooterLeft = new WPI_TalonFX(ShooterConstants.K_SHOOTERLEFT);

    kTicks2Degree = 1 / 4096.0;

    configMotor(aimHood);
    configMotor(shooterRight);
    configMotor(shooterLeft);
    configMotor(rotaterHood);

    shooterRight.setInverted(true);

    aimHood.setInverted(false);
    rotaterHood.setInverted(true);

    shooterRight.setInverted(true);
    shooterLeft.setInverted(false);

    shooterRight.follow(shooterLeft);

    rotaterHood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    aimHood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);

    aimHood.setSelectedSensorPosition(0);

    rotaterHood.configNeutralDeadband(0.1);
    aimHood.configNeutralDeadband(0.1);
  }

  // Hood Shooter Angle PID Methods //
  public void setAimHood(double speed) {
    aimHood.set(speed);
  }

  public void setRotaterHood(double speed) {
    rotaterHood.set(speed);
  }

  // XBOX Command Drive //

  public void aimStop() {
    aimHood.set(0);
  }

  public void rotateOff() {
    rotaterHood.set(0);
  }

  public void runShooter() {
    shooterRight.set(0.8);
    shooterLeft.set(0.8);
  }

  public double getRotationDegrees() {
    return (rotaterHood.getSelectedSensorPosition(0) * kTicks2Degree);
  }

  public double getAimDegrees() {
    return (aimHood.getSelectedSensorPosition(0) * kTicks2Degree);
  }

  public void offShooter() {
    shooterRight.set(0);
    shooterLeft.set(0);
  }

  public void driveHood(double rotation, double aim) {
    rotaterHood.set(rotation);
    aimHood.set(aim);
  }

  public void setVelocity(double velocity) {
    shooterLeft.set(ControlMode.Velocity, velocity);
  }

  public double getVelocity() {
    return shooterLeft.getSelectedSensorVelocity(0);
  }


  public boolean safe2MoveAimHood(double angle) {
    if (angle > Constants.ShooterConstants.K_AIM_UP_MAX || angle < 0) {
      return false;
    }

    else {
      return true;
    }
  }

  // public boolean safe2ApplyVelocity(double velocity){
  // if(velocity > ________ || velocity < 0 ){
  // return false;
  // }

  // else{
  // return true;
  // }
  // }

  public void configMotor(WPI_TalonSRX motor) {
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configPeakCurrentLimit(55);
  }

  public void configMotor(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Roatater Hood Degrees ", getRotationDegrees());
    SmartDashboard.putNumber("Aim Hood Degrees ", getAimDegrees());
    // SmartDashboard.putNumber("Velocity of Flywheel", getVelocity());
  }
}