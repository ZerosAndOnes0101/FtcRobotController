

package org.firstinspires.ftc.teamcode ;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.man.c.SkystoneDetector.Location.One;
import static org.man.c.SkystoneDetector.Location.Two;
import static org.man.c.SkystoneDetector.Location.Zero;


@Autonomous(name="Skystone Detecotor", group="Auto")

public class SkystoneAutoMode extends LinearOpMode {

    private DcMotor test;






    OpenCvCamera phoneCam;
    @Override
    public void runOpMode() throws InterruptedException {
        double sp = 0;
        test = hardwareMap.get(DcMotor.class, "te");
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        org.man.c.SkystoneDetector detector = new org.man.c.SkystoneDetector(telemetry);
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(
                () -> phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        );

        waitForStart();
        if (detector.getLocation() == One) {
            sp = 1.0;

        }
        if(detector.getLocation()==Two)   {
            sp=2.0;

        }
        else{
            sp=0.5;
        }
 Thread.sleep(1000);
phoneCam.closeCameraDevice();
        if (sp == 1.0) {

        }


        }
    }
}





