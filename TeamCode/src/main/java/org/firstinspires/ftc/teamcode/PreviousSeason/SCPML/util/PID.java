package org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util;

public class PID {
    public static double KPTurn;
    public static double KITurn;
    public static double KDTurn;

    public static double KPMove;
    public static double KIMove;
    public static double KDMove;

    public static double KPArm;
    public static double KIArm;
    public static double KDArm;

    public double KP;
    public double KI;
    public double KD;

    public double p;
    public double i;
    public double d;
    public double pidout = 0;

    NanoClock Timer = new NanoClock();
    public double updateTime = Timer.secondsLifeSpan();
    public double timeChange;
    public double lastUpdateTime = 0;
    public double lastError = 0;

    public static final int turn = 0;
    public static final int move = 1;




    public PID(Type type) {
        switch(type){
            case move:
            KP = KPMove;
            KI = KIMove;
            KD = KDMove;
            case turn:
            KP = KPTurn;
            KI = KITurn;
            KD = KDTurn;
            case arm:
            KP = KPArm;
            KI = KIArm;
            KD = KDArm;
        }
    }

    public static void setConstantsArm(double kp, double ki, double kd) {
        KPArm = kp;
        KIArm = ki;
        KDArm = kd;
    }

    public static void setConstantsTurn(double kp, double ki, double kd) {
        KPTurn = kp;
        KITurn = ki;
        KDTurn = kd;
    }

    public static void setConstantsMove(double kp, double ki, double kd) {
        KPMove = kp;
        KIMove = ki;
        KDMove = kd;
    }

    public void pid(double error) {

        updateTime = Timer.secondsLifeSpan();
        timeChange = updateTime - lastUpdateTime;
        if (timeChange != 0) {
            //P
            p = error * KP;
            //I
            i += timeChange * error * KI;
            //D
            d = (error - lastError) / timeChange * KD;

            pidout = p + i + d;
            lastUpdateTime = updateTime;
        }
    }

    public void reset() {
        p = 0;
        i = 0;
        d = 0;
        Timer.reset();
    }

    public enum Type {
        move,
        turn,
        arm
    }
}
