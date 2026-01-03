package org.firstinspires.ftc.teamcode.pedroPathing.CallBacks;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.callbacks.PathCallback;

public class WaitCallback implements PathCallback {

    private final Follower follower;
    private final long waitTimeMs;
    private final int pathIndex;

    private long startTime = -1;
    private boolean paused = false;

    public WaitCallback(Follower follower, long waitTimeMs, int pathIndex) {
        this.follower = follower;
        this.waitTimeMs = waitTimeMs;
        this.pathIndex = pathIndex;
    }
    @Override
    public void initialize(){
        Log.i("WAIT CALLBACK STATUS: ", "INITALIZED");
        startTime = System.currentTimeMillis();
    }
    @Override
    public boolean run() {
        Log.i("WAIT CALLBACK STATUS: ", "RUNNING");
        // Triggered when callback is reached
        if (!paused) {
            follower.pausePathFollowing(); // 🔴 THIS stops the robot
            Log.i("PATH FOLLOWING STATUS: ", "PAUSED");
            startTime = System.currentTimeMillis();
            Log.i("STARTED ON: ", Long.toString(startTime));
            Log.i("CURRENT TIME: ", Long.toString(System.currentTimeMillis() - startTime));
            paused = true;
        }
        return true;

    }

    @Override
    public boolean isReady() {
        if (!paused) {
            Log.i("PATH FOLLOWING STATUS: ", "PAUSED");
            Log.i("CURRENT TIME: ", Long.toString(System.currentTimeMillis() - startTime));
            return true; // return true because we are ready to run
        }

        if (System.currentTimeMillis() - startTime >= waitTimeMs) {
            follower.resumePathFollowing(); // 🟢 Continue path
            Log.i("PAUSE FOLLOWING STATUS: ", "RUNNING");
            return false; // return false because we arent ready to run
        }

        return true; // return true because we are ready to run
    }

    @Override
    public int getPathIndex() {
        Log.i("PATH FOLLOWING INDEX: ", String.valueOf(pathIndex));
        return pathIndex;
    }
}
