// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  AddressableLED led;
  AddressableLEDBuffer buffer;
  Timer timer;
  
  public LED() {
    led = new AddressableLED(LEDConstants.K_PORT);
    buffer = new AddressableLEDBuffer(LEDConstants.K_LENGTH);
    timer = new Timer();

    led.setLength(LEDConstants.K_LENGTH);

    led.start();

    timer.start();
  }

  public void setStandardColor(Color color){
    for(int i = 0; i < buffer.getLength(); i++){
      buffer.setLED(i, color);
    }
    led.setData(buffer);
  }

  public void startFlash(Color color1, Color color2, boolean able2Align){

    while(able2Align){
      // switches between each one very second // 
      if(timer.get() % 2 == 0){
        setStandardColor(color1);
        led.setData(buffer);
      }

      else{
        setStandardColor(color2);
        led.setData(buffer);
      }
    }

    while(!able2Align){
      setStandardColor(color1);
      led.setData(buffer);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
