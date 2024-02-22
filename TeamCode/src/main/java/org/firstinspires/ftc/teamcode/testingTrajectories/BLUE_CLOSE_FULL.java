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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

@Autonomous
public class BLUE_CLOSE_FULL extends LinearOpMode {

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
        ((DcMotorEx) armBase).setVelocityPIDFCoefficients(40, 0.1, 0.1, 50);
        ((DcMotorEx) armBase).setPositionPIDFCoefficients(5);
        ((DcMotorEx) gripperArm).setVelocityPIDFCoefficients(40, 0.1, 0.1, 50);
        ((DcMotorEx) gripperArm).setPositionPIDFCoefficients(5);

        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gripperL.setDirection(Servo.Direction.REVERSE);
        gripperArm.setDirection(DcMotorSimple.Direction.REVERSE);
        gripperR.setPosition(1);
        gripperL.setPosition(1);

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
            Pose2d beginPose = new Pose2d(11, 60, 3*Math.PI / 2);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


                GripperArm(540, 0.2);
                sleep(500);
                tilting.setPosition(0.8);
                sleep(1000);




                Actions.runBlocking(
                        drive.actionBuilder(beginPose)

                                .lineToY(46)
                                .waitSeconds(1)

                                .build());



                gripperR.setPosition(0);
                sleep(500);

                GripperArm(0, 0.2);
                ArmBase(230,0.5);
                gripperR.setPosition(1);

                sleep(1000);

                tilting.setPosition(0.7);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(11, 45, 4.276057))

                                .turnTo(2 * Math.PI)
                                .lineToX(40)

                                .turnTo(3*Math.PI/2 )

                                .lineToY(32)


                                .turnTo(2* Math.PI)

                                .build());



//            sleep(1000);
                Open_Tilting();
                GripperArm(400, 0.2);
                sleep(2000);


                gripperL.setPosition(0);
                sleep(500);
                gripperL.setPosition(1);

                GripperArm(0, 0.2);
                sleep(500);

                ArmBase(0, 0.5);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(42, 26, 2 * Math.PI))
                                .lineToX(50)
                                .turnTo(3 * Math.PI/2)
                                .lineToY(60)
                                .build());




            }

        } else if (tpPos == 1) {
            //  Left Spike Mark
            Pose2d beginPose = new Pose2d(11, 60, 3*Math.PI / 2);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


                GripperArm(540, 0.2);
                sleep(500);
                tilting.setPosition(0.8);


                Actions.runBlocking(
                        drive.actionBuilder(beginPose)

                                .lineToY(47)
                                .turnTo(-1.047198)
                                .waitSeconds(1)

                                .build());



                gripperR.setPosition(0);
                sleep(500);

                GripperArm(0, 0.2);
                ArmBase(230,0.5);
                gripperR.setPosition(1);

                sleep(1000);

                tilting.setPosition(0.7);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(11, 47, -1.047198))

                                .lineToY(55)

                                .turnTo(2 * Math.PI)
                                .lineToX(43)

                                .turnTo(3*Math.PI/2 )

                                .lineToY(39)


                                .turnTo(2* Math.PI)
                                .build());



//            sleep(1000);
                Open_Tilting();
                GripperArm(450, 0.2);
                sleep(2000);


                gripperL.setPosition(0);
                sleep(500);
                gripperL.setPosition(1);

                GripperArm(0, 0.2);
                sleep(500);

                ArmBase(0, 0.5);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(42, 26, 2 * Math.PI))
                                .lineToX(50)
                                .turnTo(3 * Math.PI/2)
                                .lineToY(57)
                                .build());




            }
        } else {
            // move forward to the spike mark
            Pose2d beginPose = new Pose2d(11, 60, 3*Math.PI / 2);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


                GripperArm(540, 0.2);
                sleep(500);
                tilting.setPosition(0.8);
                sleep(1000);





                Actions.runBlocking(
                        drive.actionBuilder(beginPose)

                                .lineToY(43)
                                .turnTo(4.363323)
                                .waitSeconds(1)

                                .build());



                gripperR.setPosition(0);
                sleep(500);

                GripperArm(0, 0.2);
                ArmBase(230,0.5);
                gripperR.setPosition(1);

                sleep(1000);

                tilting.setPosition(0.7);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(11, 43, 4.363323))

                                .turnTo(2 * Math.PI)
                                .lineToX(42)

                                .turnTo(3*Math.PI/2 )

                                .lineToY(26)


                                .turnTo(2* Math.PI)

                                .build());



//            sleep(1000);
                Open_Tilting();
                GripperArm(400, 0.2);
                sleep(2000);


                gripperL.setPosition(0);
                sleep(500);
                gripperL.setPosition(1);

                GripperArm(0, 0.2);
                sleep(500);

                ArmBase(0, 0.5);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(42, 26, 2 * Math.PI))
                                .lineToX(50)
                                .turnTo(3 * Math.PI/2)
                                .lineToY(60)
                                .build());




            }
        }



    }




    private void ArmBase(int Ticks, double Power) {
        armBase.setTargetPosition(Ticks);
        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armBase.setPower(Power);
    }


    private void GripperArm(int Ticks2, double Power2) {
        gripperArm.setTargetPosition(Ticks2);
        gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gripperArm.setPower(Power2);
    }

    private void Open_GripperL() {
        gripperR.setPosition(0);
        gripperL.setPosition(0);
        sleep(200);
    }


    private void Close_Tilting() {
        tilting.setPosition(0.4);
        sleep(200);
    }

    private void Open_Tilting() {
        tilting.setPosition(0.65);
        sleep(200);
    }


    private void Close_GripperL() {
        gripperR.setPosition(1);
        gripperL.setPosition(1);
        sleep(200);

    }

}