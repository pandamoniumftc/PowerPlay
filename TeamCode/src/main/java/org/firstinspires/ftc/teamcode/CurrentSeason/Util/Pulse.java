package org.firstinspires.ftc.teamcode.CurrentSeason.Util;

public class Pulse {
    public boolean prevState;

    public Pulse() {}

    public boolean update(boolean current) {
        if (current && !prevState) {
            return true;
        }
        prevState = current;

        return false;
    }
}
