package org.firstinspires.ftc.teamcode.Auto.Utility;

import static org.firstinspires.ftc.teamcode.Properties.*;

public class AutoSleepCalc {
    public static int getInitialSleep(){
        int sl = INITIAL_SLEEP_MS;
        // default / min sleep
        return Math.max(sl, 1000);
    }

    public static int getStep1Sleep() {
        int sl = STEP_1_SLEEP_MS;
        // default / min sleep
        return Math.max(sl, 100);
    }

    public static int getStep2Sleep() {
        int sl = STEP_2_SLEEP_MS;
        // default / min sleep
        return Math.max(sl, 100);
    }
    public static int getStep3Sleep() {
        int sl = STEP_3_SLEEP_MS;
        // default / min sleep
        return Math.max(sl, 100);
    }
}
