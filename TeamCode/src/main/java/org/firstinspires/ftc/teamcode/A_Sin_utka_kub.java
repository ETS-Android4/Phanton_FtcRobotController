package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import  com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "A_Sin_utka_kub", group="Autonomous")
//comment out this line before using
public class A_Sin_utka_kub extends Methods {
    private ElapsedTime runtime = new ElapsedTime();

    private static int valLeft = -1;
    private static int valRight = -1;
    private static float rectHeight = 1.5f / 8f;
    private static float rectWidth = 1.5f / 8f;
    private static float rectHeight1 = 1.5f / 8f;

    private static float rectWidth1 = 1.5f / 8f;

    private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] leftPos = {2.1f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] rightPos = {3.7f / 8f + offsetX, 4f / 8f + offsetY};

    private final int rows = 640;
    private final int cols = 480;


    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new A_Sin_utka_stenka.StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC*/
        //width = height in this case, because camera is in portrait mode.
        leftF = hardwareMap.dcMotor.get("lf");
        leftB = hardwareMap.dcMotor.get("lr");
        rightF = hardwareMap.dcMotor.get("rf");
        rightB = hardwareMap.dcMotor.get("rr");
        krut = hardwareMap.dcMotor.get("kr");
        vobla = hardwareMap.dcMotor.get("vl");
        zaxvat = hardwareMap.crservo.get("zx");
        vikidisch = hardwareMap.crservo.get("vs");
        pod = hardwareMap.dcMotor.get("pod");
        sos = hardwareMap.dcMotor.get("sos");

        initGyro();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
              telemetry.addData("Values", valLeft + "  " + valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            telemetry.update();
            sleep(100);
            if (valLeft == 255){
                vpered(400, 0.4);
            vpravo(740, 0.25);
            vikidisch_niz(1);
            vpravo(-500, 0.4);
            razvarot(-1000, 0.25);
            vpravo(1000, 0.4);
            vpered(550, 0.3);
            krut.setPower(-1);
            sleep(2000);
            krut.setPower(0);
            sleep(1);
            vpered(340, 0.4);//TODO:
            vpravo(-400, 0.25);
            vpravo(100, 0.2);
            vikidisch.setPower(1);
            sleep(500);
            kub_down(-1750);
            pod.setPower(0);
            stop_all();
            sleep(30000);}
            else if (valRight == 255){
                vpered(-950, 0.4);
                vpravo(130, 0.25);
                vikidisch_niz(1);
                vpravo(-500, 0.4);
                razvarot(-1000, 0.25);
                vpravo(1000, 0.4);
                vpered(550, 0.3);
                krut.setPower(-1);
                sleep(2000);
                krut.setPower(0);
                sleep(1);
                vpered(340, 0.4);//TODO:
                vpravo(-400, 0.25);
                vpravo(100, 0.2);
                vikidisch.setPower(1);
                sleep(500);
                kub_down(-1900);
                pod.setPower(0);
                stop_all();
                sleep(30000);
            } else {
                vpered(-900, 0.4);
                vpravo(100, 0.25);
                vikidisch_niz(1);
                vpravo(-500, 0.4);
                razvarot(-1000, 0.25);
                vpravo(1000, 0.4);
                vpered(550, 0.3);
                krut.setPower(-1);
                sleep(2000);
                krut.setPower(0);
                sleep(1);
                vpered(340, 0.4);//TODO:
                vpravo(-400, 0.25);
                vpravo(100, 0.2);
                vikidisch.setPower(1);
                sleep(500);
                kub_down(-2000);
                pod.setPower(0);
                stop_all();
                sleep(30000);
            }
            /*if(valLeft == 255){
                //Траектория 1
            }
            else if (valRight == 255){
                //Tраектория 2
            }
            else {
                //Траектория 3
            }*/
        }
    }
}