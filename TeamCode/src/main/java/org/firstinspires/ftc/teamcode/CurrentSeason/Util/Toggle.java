package org.firstinspires.ftc.teamcode.CurrentSeason.Util;

public class Toggle {
    private boolean flag = false;

    public boolean state;

    public Toggle(boolean defaultState) {
        state = defaultState;
    }

    public boolean updateState(boolean update) {
        if (update && !flag) state = !state;

        if (update) flag = true;
        else flag = false;

        return state;
    }

}
