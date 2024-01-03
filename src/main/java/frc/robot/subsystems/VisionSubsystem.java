// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Limelight;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  Limelight limelight;

  public VisionSubsystem() {
    limelight = new Limelight("limelight");
  }

  public Limelight getCenterLimelight(){
    return limelight;
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    limelight.periodic();

    SmartDashboard.putNumber("Robot Rotation: ", getCenterLimelight().getPose().getRotation().getDegrees());
  }
}