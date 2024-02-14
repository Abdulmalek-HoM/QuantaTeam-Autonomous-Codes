package org.firstinspires.ftc.teamcode.autoTestCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous

public class autoSelectorTest extends LinearOpMode {

    allianceSelectorClassTest selector = new allianceSelectorClassTest();

    @Override
    public void runOpMode() throws InterruptedException {

        while (!opModeIsActive() && !isStopRequested()) {
            selector.autoSelector();
        }
    }
}