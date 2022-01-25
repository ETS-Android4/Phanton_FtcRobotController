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
@Autonomous(name= "A_Sin_Kub2", group="Autonomous")
//comment out this line before using
public class A_Sin_Kub_2 extends Methods {
    private ElapsedTime runtime = new ElapsedTime();

    private static int valLeft = -1;
    private static int valRight = -1;
    private static float rectHeight = 1f / 8f;
    private static float rectWidth = 0.5f / 8f;
    private static float rectHeight1 = 1f / 8f;
    private static float rectWidth1 = 0.5f / 8f;

    private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] leftPos = {2.7f / 8f + offsetX, 5.1f / 8f + offsetY};
    private static float[] rightPos = {5.2f / 8f + offsetX, 5.1f / 8f + offsetY};

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
        zaxvat = hardwareMap.crservo.get("zx");
        vikidisch = hardwareMap.crservo.get("vs");
        pod = hardwareMap.dcMotor.get("pod");
        sos = hardwareMap.dcMotor.get("sos");
        pisun = hardwareMap.dcMotor.get("pis");
        pis = hardwareMap.crservo.get("ps");
        pis = hardwareMap.crservo.get("ps");
        pis.setPower(-0.11);
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Values", valLeft + "  " + valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            telemetry.update();
            sleep(100);
            if (valLeft == 255) {
                vpered(400, 0.2);
                vpravo(660, 0.3);
                vikidisch_mid(1);
                vlevo(1000, 0.3);
                nazad(1050, 0.4);
                nazad(500, 0.2);
                kub_down(2000);
                sos.setPower(-1);
                nazad(50, 0.2);
                sleep(1500);
                sos.setPower(1);
                sleep(1000);
                sos.setPower(0);
                sleep(1);
                vpered(1650, 0.4);
                vlevo(250,0.4);
                vpravo(780, 0.3);
                vikidisch_mid(1);
                vlevo(1000, 0.4);
                nazad(1450, 0.4);
                //vpravo(500, 0.4);
                kub_down(1900);
                pod.setPower(0);
                stop_all();
                sleep(30000);
            } else if (valRight == 255) {
                vpered(400, 0.2);
                vpravo(745, 0.3);
                vikidisch_mid(1);
                vlevo(1000, 0.3);
                nazad(1050, 0.4);
                nazad(500, 0.2);
                kub_down(2000);
                sos.setPower(-1);
                nazad(50, 0.2);
                sleep(1500);
                sos.setPower(1);
                sleep(1000);
                sos.setPower(0);
                sleep(1);
                vpered(1650, 0.4);
                vlevo(250,0.4);
                vpravo(780, 0.3);
                vikidisch_mid(1);
                vlevo(1000, 0.4);
                nazad(1450, 0.4);
                //vpravo(500, 0.4);
                kub_down(1900);
                pod.setPower(0);
                stop_all();
                sleep(30000);
            } else {
                vpered(400, 0.2);
                vpravo(850, 0.3);
                vikidisch_verx(0.92);
                vlevo(1000, 0.3);
                nazad(1050, 0.4);
                nazad(500, 0.2);
                kub_down(2950);
                sos.setPower(-1);
                vpered(-50, 0.2);
                sleep(1500);
                sos.setPower(1);
                sleep(1000);
                sos.setPower(0);
                sleep(1);
                vpered(1660, 0.4);
                vlevo(250,0.4);
                vpravo(780, 0.3);
                vikidisch_mid(1);
                vlevo(1000, 0.4);
                nazad(1550, 0.4);
                //vpravo(500, 0.4);
                kub_down(1950);
                pod.setPower(0);
                stop_all();
                sleep(30000);
            }
        }
    }
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private StageSwitchingPipeline.Stage stageToRenderToViewport = StageSwitchingPipeline.Stage.detection;
        private StageSwitchingPipeline.Stage[] stages = StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 112, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame


            double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
            valLeft = (int) pixLeft[0];

            double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
            valRight = (int) pixRight[0];

            //create three points
            Point pointLeft = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
            Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointLeft, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointRight, 5, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols() * (leftPos[0] - rectWidth1 / 2),
                            input.rows() * (leftPos[1] - rectHeight1 / 2)),
                    new Point(
                            input.cols() * (leftPos[0] + rectWidth1 / 2),
                            input.rows() * (leftPos[1] + rectHeight1 / 2)),
                    new Scalar(0, 255, 0), 3);

            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols() * (rightPos[0] - rectWidth / 2),
                            input.rows() * (rightPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (rightPos[0] + rectWidth / 2),
                            input.rows() * (rightPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection: {
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }

                default: {
                    return input;
                }
            }
        }
    }
}