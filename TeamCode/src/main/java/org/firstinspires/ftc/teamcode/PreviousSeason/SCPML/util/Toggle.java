package org.firstinspires.ftc.teamcode.PreviousSeason.SCPML.util;

public class Toggle {
    public boolean heldLastFrame = false;
    public boolean state;
    public boolean needChange = false;
    public Toggle(boolean state) {
        this.state = state;
    }

    public void update(boolean input) {
        if (!heldLastFrame && input) {
            state = !state;
            heldLastFrame = true;
            needChange = true;
        }

        if (input) heldLastFrame = true;
        else heldLastFrame = false;
    }
}
