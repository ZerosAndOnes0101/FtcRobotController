package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.app.Application;
import android.media.MediaPlayer;
import android.net.Uri;
import android.os.Bundle;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.android.dx.command.Main;

@TeleOp(name = "MechanumDrive", group = "LinearOpmode")

public class MechanumDrive extends EncoderTest2 {
    private ElapsedTime runtime = new ElapsedTime();
    public MediaPlayer player1 = new MediaPlayer();
    private DistanceSensor x;
    private DistanceSensor x1;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public CRServo Trin = null;
    public CRServo Trin1 = null;
    private boolean isAOn = false;
    private double slowModeValue = 1.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        player1 = MediaPlayer.create(hardwareMap.appContext, R.raw.song);
        //  x = hardwareMap.get(DistanceSensor.class,"distance");
        //   x1 = hardwareMap.get(DistanceSensor.class, "distance1");
        //  Rev2mDistanceSensor sensorTime = (Rev2mDistanceSensor)x;
        //   Rev2mDistanceSensor sensor1Time = (Rev2mDistanceSensor)x1;
        telemetry.addData(">>", "Start");
        telemetry.update();
        Trin = hardwareMap.crservo.get("Trin");
        Trin1=hardwareMap.crservo.get("Trin1");


        //leftDrive = front left motor
        //rightDrive = right front motor
        //leftBackDrive = back left motor
        //rightBackDrive = back right motor
        //armJoint = arm motor
        //sweeper = sweeper servo
        //latch = rack and pinion motor
        //boxJoint = box moving servo


        leftDrive = hardwareMap.get(DcMotor.class, "FL");
        rightDrive = hardwareMap.get(DcMotor.class, "FR");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


            //Calculations for motor power
            double r = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rotate = -gamepad1.right_stick_x;
            final double frontLeft = -r * Math.cos(robotAngle) + rotate;
            final double frontRight = r * Math.sin(robotAngle) + rotate;
            final double rearLeft = -r * Math.sin(robotAngle) + rotate;
            final double rearRight = r * Math.cos(robotAngle) + rotate;
/*

    //        while (gamepad1.a) {
      //          x.getDistance(DistanceUnit.INCH);
      //          if (x.getDistance(DistanceUnit.INCH) >= 36) {
//Right

             //       leftDrive.setPower(-1.0);
              //      leftBackDrive.setPower(1.0);
                    rightBackDrive.setPower(-1.0);

                    rightDrive.setPower(1.0);
                }

                if (x.getDistance(DistanceUnit.INCH) <= 38) {
                    leftDrive.setPower(1.0);
                    leftBackDrive.setPower(-1.0);
                    rightBackDrive.setPower(1.0);

                    rightDrive.setPower(-1.0);

                }
                else{
                    leftDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightBackDrive.setPower(0);

                    rightDrive.setPower(0);
                }


            }

            while (gamepad1.right_bumper) {
                Trin.setPower(1.0);
            }

            while (gamepad1.x) {

                if (Math.abs(x.getDistance(DistanceUnit.INCH) - x1.getDistance(DistanceUnit.INCH)) <= 3) {
                    telemetry.addData("Alligned:", "Yes");

                } else if ((x.getDistance(DistanceUnit.INCH) >= x1.getDistance(DistanceUnit.INCH))) {
                    leftDrive.setPower(1.0);
                    leftBackDrive.setPower(1.0);
                } else {
                    rightDrive.setPower(1.0);
                    rightBackDrive.setPower(1.0);
                }


            }

*/
            if (gamepad1.right_bumper) {
                Trin.setPower(2.0);

            }
            if(gamepad1.left_bumper){
                Trin1.setPower(2.0);
            }

            if(gamepad1.y){
                Trin1.setPower(0);
            }

            else {
                Trin.setPower(0);

            }
                if (gamepad1.b) {
                    player1.start();

                } else {
                    player1.pause();
                }

                if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0) {
                    if (gamepad1.left_stick_x > 0.1 || gamepad1.left_stick_x < 0.1) {
                        //Motor Power Sets
                        leftDrive.setPower(frontLeft);
                        telemetry.addData("Power FrontLeft", frontLeft);
                        rightDrive.setPower(frontRight);
                        telemetry.addData("Power FrontRight", frontRight);
                        leftBackDrive.setPower(rearLeft);
                        telemetry.addData("Power rearLeft", rearLeft);
                        rightBackDrive.setPower(rearRight);
                        telemetry.addData("Power rearRight", rearRight);
                        telemetry.update();
                    } else {
                        //Motor Power Sets
                        leftDrive.setPower(frontLeft / slowModeValue);
                        rightDrive.setPower(frontRight / slowModeValue);
                        leftBackDrive.setPower(rearLeft / slowModeValue);
                        rightBackDrive.setPower(rearRight / slowModeValue);
                    }
                } else {
                    leftDrive.setPower(0.005);
                    rightDrive.setPower(0.005);
                    leftBackDrive.setPower(0.005);
                    rightBackDrive.setPower(0.005);

                }


            }

        }


    }
