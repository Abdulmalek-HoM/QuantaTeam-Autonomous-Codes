package org.firstinspires.ftc.teamcode.testingTrajectories;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

public final class forward_turn_forwardTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(-34, -70, Math.PI / 2);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
//                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
//                            .splineTo(new Vector2d(0, 60), Math.PI)
//                            .lineToX(48)
//                            .waitSeconds(3)
////                            .lineToY(-24)
////                            .waitSeconds(3)
////                            .splineTo(new Vector2d(36, -34), (2 * Math.PI))
////                            .waitSeconds(3)
////                            .lineToXLinearHeading(48, (Math.PI / 4))
                            ////                           .waitSeconds(3)
//                            .splineToLinearHeading(new Vector2d(36, 24), Math.PI)
                            ////                           .splineToLinearHeading(new Pose2d(36, 24, Math.PI), 0)

                            /////////////////////////////////////

                            .lineToY(-45) // head straight to put the pixel

                            .waitSeconds(2)
                            .splineToLinearHeading(new Pose2d(-50, -35.5, 1 * Math.PI), 0)
                            .waitSeconds(2)
                            .strafeTo(new Vector2d(-50, -10))
//                .splineToLinearHeading(new Pose2d(20, -10, 0), 0)
                            .strafeTo(new Vector2d(20, -10))
                            .splineToLinearHeading(new Pose2d(40, -35.5, 2 * Math.PI), 1)
                            .build());

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(-34, -45, Math.PI / 2))
                            .splineToLinearHeading(new Pose2d(49.87, -35.5, 2 * Math.PI), 0)
                            .waitSeconds(1)
                            .build());



        }




    }
    }


