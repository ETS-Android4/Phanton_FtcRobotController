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
@Autonomous(name= "A_Krasn_utka_stenka", group="Autonomous")
//comment out this line before using
public class A_Krasn_utka_stenka extends Methods {
    private ElapsedTime runtime = new ElapsedTime();

    private static int valLeft = -1;
    private static int valRight = -1;
    private static float rectHeight = 1.5f / 8f;
    private static float rectWidth = 1.f / 8f;
    private static float rectHeight1 = 1.5f / 8f;
    private static float rectWidth1 = 1f / 8f;

    private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] leftPos = {3.5f / 8f + offsetX, 4.8f / 8f + offsetY};
    private static float[] rightPos = {5.7f / 8f + offsetX, 4.8f / 8f + offsetY};

    private final int rows = 640;
    private final int cols = 480;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC*/
        //width, height
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
            telemetry.addData(
                    "Height", rows);
            telemetry.addData("Width", cols);
            telemetry.update();
            sleep(100);
            if (valLeft == 255) {
                //sleep(9000);
                nazad(590, 0.2);
                krut.setPower(-0.7);
                sleep(1200);
                nazad(35, 0.2);
                sleep(1200);
                nazad(35, 0.2);
                sleep(1500);
                krut.setPower(0);
                sleep(1); //T
                vpered(1150, 0.4);//TODO:
                vpravo(760, 0.25); //-850
                vikidisch_niz(1);
                vlevo(400, 0.4);
                razvarot(1100, 0.3);
                vpravo(500, 0.3);
                nazad(1150, 0.4);
                //vpravo(-500, 0.4);
                kub_down(1200);
                pod.setPower(0);
                stop_all();
                sleep(30000);
            } else if (valRight == 255) {
                //sleep(9000);
                nazad(590, 0.2);
                krut.setPower(-0.7);
                sleep(1200);
                nazad(35, 0.2);
                sleep(1200);
                nazad(35, 0.2);
                sleep(1500);
                krut.setPower(0);
                sleep(1); //T
                vpered(1150, 0.4);//TODO:
                vpravo(810, 0.25); //-850
                vikidisch_mid(1);
                vlevo(400, 0.4);
                razvarot(1100, 0.3);
                vpravo(500, 0.3);
                nazad(1150, 0.4);
                //vpravo(-500, 0.4);
                kub_down(1950);
                pod.setPower(0);
                stop_all();
                sleep(30000);
            } else {
                //sleep(9000);
                vpered(-590, 0.2);
                krut.setPower(-0.7);
                sleep(1200);
                nazad(35, 0.2);
                sleep(1200);
                nazad(35, 0.2);
                sleep(1500);
                krut.setPower(0);
                sleep(1); //T
                vpered(1050, 0.4);//TODO:
                vpravo(880, 0.25); //-850
                vikidisch_verx(0.92);
                vlevo(400, 0.3);
                razvarot(1100, 0.3);
                vpravo(500, 0.3);
                nazad(1250, 0.4);
                //vpravo(-500, 0.4);
                kub_down(2950);
                pod.setPower(0);
                stop_all();
                sleep(30000);
            }
        }
    }
}