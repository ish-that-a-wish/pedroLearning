package org.firstinspires.ftc.teamcode.Actions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

public class ActionToRunnable {

    public static Runnable turnRunnable(Action action) {
        return () -> Actions.runBlocking(action);
    }
}