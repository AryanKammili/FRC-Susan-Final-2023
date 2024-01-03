// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;
import java.lang.Math;

// Ctrl + Click to access links // 

// All information to calcaulte this can be found here with a working diagram where we get to see how each variable works // 
// https://www.chiefdelphi.com/t/desmos-trajectory-calculator-for-a-shooter-with-an-adjustable-hood/400024 // 

// Further information and break down of this is found in our engineering notebook // 
// https://docs.google.com/document/d/1x7_GRnuuen645iZir8wHqi2jV7Wr1hxT9HZAEn5M7TU/edit?usp=sharing // 


/** Add your docs here. */
public class AlignmentCalculator {
    double d;
    double h;
    double s;

    public AlignmentCalculator(double heightDiff, double desiredAngle){
        h = heightDiff;
        s = desiredAngle;
    }

    public double calculateAngle(double d){
        
        double angle = (tan(s) * d * (-2 * h)) / -d;
        angle = atan(angle);

        return angle;
    }

    public double calculateVelocity(double angle, double d){

        double a = angle;

        double velocity = -9.8 * square(d) * ( 1 + square(tan(a)));

        velocity /= (2 * h) - ((2 * d) * (tan(a)));
        velocity = sqrt(velocity);

        return velocity;
    }

    public double calculateRotation(double targetX, double targetZ, double robotX, double robotZ){
        double x = (targetX - robotX);
        double z = (targetZ - robotZ);

        return tan(x / z);
    }

    public double calculateDistance(double targetX, double targetZ, double robotX, double robotZ){
        double distance;

        distance = sqrt(square(robotX - targetX) + square(robotZ - targetZ));

        return distance;
    }


    private double square(double number){
        return Math.pow(number, 2);
    }

    private double tan(double number){
        return Math.toDegrees(Math.tan(number));
    }

    private double atan(double number){
        return Math.toDegrees(Math.atan(number));
    }

    private double sqrt(double number){
        return Math.sqrt(number);
    }
}