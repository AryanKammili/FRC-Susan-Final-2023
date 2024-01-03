// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private WPI_TalonSRX intakeMotor;
  private DoubleSolenoid pneumatic1;
  private DoubleSolenoid pneumatic2;
  private Compressor compressor;
  
  public IntakeSubsystem() {
    intakeMotor = new WPI_TalonSRX(IntakeConstants.K_INTAKE);

    configMotor(intakeMotor);

    compressor = new Compressor(IntakeConstants.K_COMPRESSOR, PneumaticsModuleType.REVPH);

    pneumatic1 = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 0 , 2);
    pneumatic2 = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 1 , 3);
  }

  public void grab(){
    pneumatic1.set(DoubleSolenoid.Value.kForward);
    pneumatic2.set(DoubleSolenoid.Value.kForward);
    intakeMotor.set(1);
  }

  public void retract(){
    pneumatic1.set(DoubleSolenoid.Value.kReverse);
    pneumatic2.set(DoubleSolenoid.Value.kReverse);
    intakeMotor.set(-1);
  }

  public void stopIntake(){
    intakeMotor.set(0);
  }

  public String getSolenoidState(DoubleSolenoid pneumatic){
    
    if(pneumatic.get().equals(DoubleSolenoid.Value.kOff)){
      return "off";
    }

    else if(pneumatic.get().equals(DoubleSolenoid.Value.kForward)){
      return "Forward";
    }

    else if(pneumatic.get().equals(DoubleSolenoid.Value.kReverse)){
      return "Reverse";
    }

    return ":)";
  }

  public void configMotor(WPI_TalonSRX motor){
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Pneumatic 1 status", getSolenoidState(pneumatic1));
    SmartDashboard.putString("Pneumatic 2 status", getSolenoidState(pneumatic2));
    SmartDashboard.putNumber("Compressor Reading: ", compressor.getPressure());
  }
}