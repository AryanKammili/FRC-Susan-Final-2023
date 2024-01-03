// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

    public static class DriveConstants{

        public static final int K_RIGHTLEAD = 10;
        public static final  int K_LEFTLEAD = 11;
        public static final int K_RIGHTBACK = 12;
        public static final int K_LEFTBACK = 13;

        // Please Fix this Later // 
        public static final double m_GEARRATIO = 1;

    }

    public static class IntakeConstants{
        public static final int K_INTAKE = 16;
        public static final int K_COMPRESSOR = 1;

    }

    public static class BallHandlerConstants{

        public static final int K_HOPPER = 22;
        public static final int K_SUCKER = 17;

        public static final double K_HOPPER_SPEED = 0.5;
        public static final double K_SUCKER_SPEED = 1;
    }

    public static class ShooterConstants{

        public final static int K_AIMHOOD = 18;
        public final static int K_SHOOTERRIGHT = 14;
        public final static int K_SHOOTERLEFT = 15;
        public final static int K_ROTATERHOOD = 3;


        public final static double K_ROTATERHOOD_CONSTANT_SPEED = 0.25;
        public final static double K_AIMHOOD_CONSTANT_SPEED = 0.25;

        public final static double K_ROTATERHOOD_OFFSET = 0;
        public final static double K_AIMHOOD_OFFSET = 1.7 - 0.13;

        public final static int K_AIM_UP_MAX = 27;
        public final static int K_AIM_DOWN_MAX = 0;

        public final static double K_ROTATER_GEARRATIO_ROTATION = 236/38;
        public final static double K_ROTATER_GEARRATIO_AIM = 1;

    }

    public static class LEDConstants{
        public final static int K_PORT = 0;
        public final static int K_LENGTH = 50;



    }

    public static class PID_IDConstants{

        public static class rotaterHood{
            public final static int kP = 0;
            public final static int kI = 0;
            public final static int kD = 0;
    
            public final static int maxVelocity = 0;
            public final static int maxAcceleration = 0;
        }

        public static class aimHood{
            public final static double kP = 0.09;
            public final static int kI = 0;
            public final static double kD = 0.0001;
    
            public final static int maxVelocity = 0;
            public final static int maxAcceleration = 0;
        }

        public static class turnCommand{
            public final static int kP = 0;
            public final static int kI = 0;
            public final static int kD = 0;
            
            public final static int maxVelocity = 0;
            public final static int maxAcceleration = 0;
        }

        public static class velocityCommand{
            public final static int kP = 0;
            public final static int kI = 0;
            public final static int kD = 0;
            
            public final static int maxVelocity = 0;
            public final static int maxAcceleration = 0;
        }

        // public static class driveCommand{
        //     public final static int kP = 0;
        //     public final static int kI = 0;
        //     public final static int kD = 0;
    
        //     public final static int maxVelocity = 0;
        //     public final static int maxAcceleration = 0;
        // }

    }
    
}