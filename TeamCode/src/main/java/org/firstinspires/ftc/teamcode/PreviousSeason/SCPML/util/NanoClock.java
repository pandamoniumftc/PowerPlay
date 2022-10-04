package org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util;

public class NanoClock extends Object {

    private long clockStart = System.nanoTime();

    public long totalTime(){
        return System.nanoTime();
    }

    public long nanoLifespan() {
        return System.nanoTime() - clockStart;
    }

    public double secondsLifeSpan() {
        return nanoLifespan() / 1000000000;
    }

    public void reset() {
        clockStart = System.nanoTime();
    }
}
