package org.firstinspires.ftc.teamcode.common;

import com.pedropathing.paths.callbacks.PathCallback;

public class CustomIntakeCallback implements PathCallback {
    @Override
    public boolean run() {
        return false;
    }

    @Override
    public boolean isReady() {
        return false;
    }

    @Override
    public int getPathIndex() {
        return 0;
    }
}
