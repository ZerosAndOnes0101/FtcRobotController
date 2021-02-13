
package org.firstinspires.ftc.teamcode ;


import android.media.MediaPlayer;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MechanumDrive;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.man.c.SkystoneDetector.Location.One;
import static org.man.c.SkystoneDetector.Location.Two;
import static org.man.c.SkystoneDetector.Location.Zero;


@Autonomous(name="SkystoneAutoMode", group="Auto")

public class SkystoneAutoMode extends MechanumDrive {

    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public Servo claw =  null;
    public DcMotor arm = null;
    public DistanceSensor x = null;


    OpenCvCamera phoneCam;
    @Override
    public void runOpMode() {
        x = hardwareMap.get(DistanceSensor.class, "distance");
        Rev2mDistanceSensor sensorTime = (Rev2mDistanceSensor) x;
        robot.init(hardwareMap);


           double sp = 0;
            claw = hardwareMap.servo.get("claw");
           leftDrive = hardwareMap.get(DcMotor.class, "FL");
           rightDrive = hardwareMap.get(DcMotor.class, "FR");
           leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
           rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
           int cameraMonitorViewId = hardwareMap.appContext
                   .getResources().getIdentifier("cameraMonitorViewId",
                           "id", hardwareMap.appContext.getPackageName());
           phoneCam = OpenCvCameraFactory.getInstance()
                   .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
           org.man.c.SkystoneDetector detector = new org.man.c.SkystoneDetector(telemetry);
           phoneCam.setPipeline(detector);
           phoneCam.openCameraDeviceAsync(
                   new OpenCvCamera.AsyncCameraOpenListener() {
                       @Override
                       public void onOpened() {
                           phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                       }
                   }
           );

        telemetry.addData("RangeFront", x.getDistance(DistanceUnit.INCH));

        telemetry.update();
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

           waitForStart();
           claw.setPosition(0.0);
            sleep(1000);
           phoneCam.closeCameraDevice();

           if (detector.getLocation() == One) {
           encoderDrive(.4,7.5,-7.5,3);
            claw.setPosition(1.0);


           }

           if (detector.getLocation() == Zero) {

               telemetry.addData("RangeFront", x.getDistance(DistanceUnit.INCH));

               telemetry.update();
               if(x.getDistance(DistanceUnit.INCH)<=47){

                   telemetry.addData("RangeFront", x.getDistance(DistanceUnit.INCH));

                   telemetry.update();
                   arm.setPower(0.8);
                   claw.setPosition(-0.1);
                   leftBackDrive.setPower(0.7);
                   rightDrive.setPower(0.7);
                   rightBackDrive.setPower(0.7);
                   leftDrive.setPower(0.7);

               }
               arm.setPower(-0.8);
               sleep(500);
               claw.setPosition(0.5);



           }
if(detector.getLocation() == Two){
    leftDrive.setPower(2.0);
    leftBackDrive.setPower(2.0);
    rightBackDrive.setPower(2.0);

    rightDrive.setPower(2.0);
    sleep(1100);

}










       }


    }


