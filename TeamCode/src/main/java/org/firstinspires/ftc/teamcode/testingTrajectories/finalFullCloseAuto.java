package org.firstinspires.ftc.teamcode.testingTrajectories;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.classes.blobDetectionTest;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
trajectory strategy:
start pose: new Pose2d(12, -61, Math.PI / 2)
                .lineToY(-47) // was -53.75
                .waitSeconds(4)
                .setTangent(2 * Math.PI)
                .strafeToLinearHeading(new Vector2d(47, -35.5), 2 * Math.PI)
                .waitSeconds(4)
                .strafeTo(new Vector2d(47, -60))
                .setTangent(2 * Math.PI)
                .lineToX(60)

 */
public class finalFullCloseAuto extends LinearOpMode {

    blobDetectionTest detector = new blobDetectionTest(telemetry);
    OpenCvCamera webCam;
    DcMotor gripperArm;
    DcMotor armBase;
    Servo gripperL;
    Servo gripperR;
    Servo tilting;

    int tpPos = blobDetectionTest.tp_zone;

    @Override
    public void runOpMode() throws InterruptedException {

        // webCam and pipeline init
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webCam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webCam.setPipeline(detector);

        armBase = hardwareMap.get(DcMotor.class, "armBase");
        gripperArm = hardwareMap.get(DcMotor.class, "gripperArm");
        gripperL = hardwareMap.get(Servo.class, "gripperL");
        gripperR = hardwareMap.get(Servo.class, "gripperR");
        tilting = hardwareMap.get(Servo.class, "tilting");

        // pidf values
        ((DcMotorEx) gripperArm).setVelocityPIDFCoefficients(0.15, 0.06, 0.02, 0.2);
        ((DcMotorEx) gripperArm).setPositionPIDFCoefficients(5);
        ((DcMotorEx) armBase).setVelocityPIDFCoefficients(0.2, 0.05, 0.01, 0.1);
        ((DcMotorEx) armBase).setPositionPIDFCoefficients(5);

        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gripperL.setDirection(Servo.Direction.REVERSE);
        gripperArm.setDirection(DcMotorSimple.Direction.REVERSE);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4) this is in the easy opencv docs
                webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);


            }
            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addLine("the camera is dead! help!");
            }
        });

        while (opModeInInit()) {
        //    telemetry.addLine("test test test. im waiting for u to press the start button");
        //    telemetry.update();
        }

        waitForStart();

        if (tpPos == 2) {
            // move forward to the spike mark
            Pose2d beginPose = new Pose2d(-35, -60, Math.PI / 2);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


                GripperArm(540, 0.2);
                sleep(500);
                tilting.setPosition(0.8);



                Actions.runBlocking(
                        drive.actionBuilder(beginPose)

                                .lineToY(-44)
                                .waitSeconds(1)
                                .build());



                gripperR.setPosition(0);
                sleep(500);

                GripperArm(10, 0.2);
                ArmBase(230,0.5);
                gripperR.setPosition(1);

                sleep(1000);

                tilting.setPosition(0.7);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(-35, -60, Math.PI / 2))

                                .splineToLinearHeading(new Pose2d(-40, -35.5, 1 * Math.PI), 0)
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(-45, 10))
                                .strafeTo(new Vector2d(20, 10))
                                .splineToLinearHeading(new Pose2d(48, -25, 2 * Math.PI), 1)
                                .build());



//            sleep(1000);
                Open_Tilting();
                GripperArm(400, 0.2);
                sleep(2000);


                gripperL.setPosition(0);
                sleep(500);
                gripperL.setPosition(1);

                GripperArm(10, 0.2);
                sleep(500);

                ArmBase(0, 0.5);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(48, -25, 2 * Math.PI))

                                .strafeTo(new Vector2d(55, -30))
                                .build());




            }

        } else if (tpPos == 1) {
            // move forward to the spike mark
            Pose2d beginPose = new Pose2d(-35, -60, Math.PI / 2);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

                GripperArm(540, 0.2);
                sleep(500);
                tilting.setPosition(0.8);


                Actions.runBlocking(
                        drive.actionBuilder(beginPose)

                                .splineToLinearHeading(new Pose2d(-35, -54, 2.094395), 0)

                                .waitSeconds(1)
                                .build());


//            ArmBase(20, 0.5);
//            sleep(2000);

                gripperR.setPosition(0);
                sleep(500);

                GripperArm(10, 0.2);
                ArmBase(230,0.5);
                gripperR.setPosition(1);

                sleep(1000);

                tilting.setPosition(0.7);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(-35, -54, 2.094395))
                                .turnTo(Math.PI / 2)
                                .lineToYSplineHeading(15,2 * Math.PI)

                                .strafeTo(new Vector2d(20, 10))
                                .splineTo(new Vector2d(45, -15), 2 * Math.PI)
//                            .splineToLinearHeading(new Pose2d(-40, -35.5, 1 * Math.PI), 0)
//                            .waitSeconds(2)
                                .build());


                Open_Tilting();
                GripperArm(400, 0.2);
                sleep(2000);


                gripperL.setPosition(0);
                sleep(500);
                gripperL.setPosition(1);

                GripperArm(10, 0.2);
                sleep(500);

                ArmBase(0, 0.5);


                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(48, -15, 2 * Math.PI))

                                .strafeTo(new Vector2d(55, -15))
                                .build());


            }
        } else {
            // move forward to the spike mark
            Pose2d beginPose = new Pose2d(12, -60, Math.PI / 2);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

                Actions.runBlocking(
                        drive.actionBuilder(beginPose)
                                .lineToYLinearHeading(-47, Math.PI / 3)
                                .build());

            }

            // put pixel on spike mark
            openArm();
            gripperR.setPosition(0);
            sleep(1000);
            gripperR.setPosition(1);

            sleep(1000);

            // set arm position to being on the back drop
            armOnBd();

            // go to bd
            Pose2d beginPose1 = new Pose2d(12, -47, Math.PI / 3);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose1);

                Actions.runBlocking(
                        drive.actionBuilder(beginPose1)
                                .setTangent(2 * Math.PI)
                                .strafeToLinearHeading(new Vector2d(35.5, -35.5), 2 * Math.PI)
                                .build());

            }


            // drop pixel
            gripperL.setPosition(0);

            sleep(2000);

            // close gripper
            gripperL.setPosition(1);

            sleep(500);

            // arm position to closed
            closeArm();

            sleep(1000);

            closingFull();

            sleep(1000);

            // park
            Pose2d beginPose2 = new Pose2d(35.5, -35.5, Math.PI * 2);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose2);

                Actions.runBlocking(
                        drive.actionBuilder(beginPose2)
                                .strafeTo(new Vector2d(47, -60))
                                .setTangent(2 * Math.PI)
                                .lineToX(60)
                                .build());

            }

            // robot is parked


        }



    }

    void openArm() {
        GripperArm(540, 0.2);
        sleep(500);
        // opening position for tilting
        tilting.setPosition(1);
    }

    void closeArm() {
        GripperArm(200, 0.2);
        sleep(500);
        // close position for tilting
        tilting.setPosition(0.3);
    }

    void armOnBd() {
        tilting.setPosition(1);
        sleep(200);
        GripperArm(400, 0.2);
        ArmBase(20, 0.8);
    }

    void GripperArm(int ticks, double power) {
        gripperArm.setTargetPosition(ticks);
        gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gripperArm.setPower(power);
    }

    void ArmBase(int ticks, double power) {
        armBase.setTargetPosition(ticks);
        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armBase.setPower(power);
    }

    void closingFull() {
        ArmBase(0, 0.5);
        GripperArm(0, 0.2);

    }
    private void Open_Tilting() {
        tilting.setPosition(0.7);
        sleep(200);
    }

}