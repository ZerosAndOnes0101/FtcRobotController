package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance", group = "Sensor")

public class Distance extends LinearOpMode {
    private DistanceSensor x ;


@Override
public void runOpMode(){
  x = hardwareMap.get(DistanceSensor.class,"distance");

    Rev2mDistanceSensor sensorTime = (Rev2mDistanceSensor)x;
    telemetry.addData(">>","Start");
    telemetry.update();

    waitForStart();
    while(opModeIsActive()){
        if(( x.getDistance(DistanceUnit.INCH)<=2 )){
            telemetry.addData("Works?","Sure");
        }

        telemetry.addData("Range",x.getDistance(DistanceUnit.INCH));
        telemetry.update();

    }



}
}
