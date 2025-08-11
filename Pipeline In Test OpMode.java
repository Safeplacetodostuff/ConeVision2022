/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.testopmodes;

import static org.firstinspires.ftc.teamcode.drive.RobotFactory.createRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.FoldableArmRobot;
import org.firstinspires.ftc.teamcode.drive.robot2.FoldableArmRobot2;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Autonomous(name = "moo",group= "test")
@Config
public class ConeTest extends LinearOpMode {
    public static int flag = 0;
    public static int channel=1;
    public static double cawk=0;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    OpenCvWebcam webcam;
    private FoldableArmRobot robot;
    private double WristHoriPos = 0.6;
    private double WristVertPos = 0.8;
    private double ElbowPos = 0.69;
    private double WormAngle = 75;
    private double ClawOpen = 1;
    private double Clawclose = .6;

    @Override
    public void runOpMode() {

        Servo claw = hardwareMap.get(Servo.class, "armClaw");
        Servo wristrotate = hardwareMap.get(Servo.class, "wristRotate");
        Servo wrist = hardwareMap.get(Servo.class, "armWrist");
        Servo elbow = hardwareMap.get(Servo.class, "armElbow");
        DcMotor motor = hardwareMap.get(DcMotor.class, "armLiftRear");
        WebcamName clawcam = hardwareMap.get(WebcamName.class, "clawWebcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(clawcam, cameraMonitorViewId);
        ConeAnglePipeline pipeline = new ConeAnglePipeline();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        ElapsedTime time = new ElapsedTime();
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        telemetry.addLine("Waiting for start");
        telemetry.update();
        MultipleTelemetry multipleTelemetry;
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        waitForStart();

        while (opModeIsActive()) {

            if (flag == 0) {
                //webcam.stopStreaming();
                wrist.setPosition(WristHoriPos);
                elbow.setPosition(ElbowPos);
                wristrotate.setPosition(0.45);
                motor.setTargetPosition(toMotorTicks("armLiftRear",
                        WormAngle));
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(0.6);
                claw.setPosition(ClawOpen);
                multipleTelemetry.addData("angle", pipeline.return_angle());
                multipleTelemetry.update();


            }
            if (flag == 1) {
                wrist.setPosition(WristVertPos);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                motor.setTargetPosition(toMotorTicks("armLiftRear",
                        107));
            }
            if(flag==2){
                webcam.stopStreaming();
            }
            if(flag==3){
                motor.setPower(0);
            }
            if(flag==4){
                motor.setTargetPosition(0);
                elbow.setPosition(0.72);
                wrist.setPosition(0);
                wristrotate.setPosition(0.42);
                claw.setPosition(1);
            }
            if(flag==5){
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 0);
            }
            if (flag==6){
                wristrotate.setPosition(toservoticks(pipeline.return_angle()));
            }
            if(flag==7){
                claw.setPosition(0.6);
                time.reset();
                while (time.time()<2){;}
                wrist.setPosition(0.8);
                motor.setTargetPosition(toMotorTicks("armLiftRear",
                        50));



            }
        }


    }

    private int toMotorTicks(String name, double angleInDegree) {
        int ticks = 0;
        if (name == "armLiftRear") {
            ticks = (int) (angleInDegree * 145.1 * 28 / 360);
        }
        return ticks;
    }

    private double toservoticks(double angleInDegrees) {
        cawk=angleInDegrees;
        if (angleInDegrees<0){
            angleInDegrees+=180;
        }

        return (angleInDegrees-90) / 270 +0.42;
    }

    class ConeAnglePipeline extends OpenCvPipeline {
        boolean viewportPaused;
        Mat blankmat = new Mat(176, 144, CvType.CV_8U, Scalar.all(0));
        double angle=90;

        @Override
        public Mat processFrame(Mat input) {

            Mat proc = input.clone();
            Imgproc.cvtColor(input, proc, Imgproc.COLOR_BGRA2RGB);
            Imgproc.cvtColor(proc, proc, Imgproc.COLOR_RGB2HSV);

            Core.extractChannel(proc, proc, channel);
            Mat filtered = proc.clone();
            Imgproc.bilateralFilter(proc, filtered, 15, 175, 175);
            Imgproc.Canny(filtered, proc, 50, 255);
            MatOfPoint corners = new MatOfPoint();
            double qualityLevel = 0.01;
            double minDistance = 100;
            int blockSize = 3, gradientSize = 3;
            double k = 0.04;
            Imgproc.goodFeaturesToTrack(proc, corners,20,0.01,5);
            Point maxx = new Point(0, 0);
            Point minx = new Point(999, 999);
            Point maxy = new Point(0, 0);
            Point miny = new Point(999, 999);
            int y = 0;
            int x = 0;
            int[] cornersData = new int[(int) (corners.total() * corners.channels())];
            corners.get(0, 0, cornersData);
            for (int i = 0; i < corners.rows(); i++) {
                x = cornersData[i * 2];
                y = cornersData[i * 2 + 1];
                if (x > maxx.x) {
                    maxx.x = x;
                    maxx.y = y;
                }
                if (y > maxy.y) {
                    maxy.x = x;
                    maxy.y = y;
                }
                if (x < minx.x) {
                    minx.x = x;
                    minx.y = y;
                }
                if (y < miny.y) {
                    miny.x = x;
                    miny.y = y;
                }
                Imgproc.circle(input, new Point(x, y), 2, new Scalar(0, 255, 255), 2);
            }
            ArrayList Sides = new ArrayList<Point[]>();
            Sides.add(new Point[]{minx, maxy});
            Sides.add(new Point[]{maxx, miny});
            Sides.add(new Point[]{minx, miny});
            Sides.add(new Point[]{maxx, maxy});
            Imgproc.circle(input, maxx, 3, new Scalar(255, 0, 0), 3);
            Imgproc.circle(input, maxy, 3, new Scalar(255, 0, 0), 3);
            Imgproc.circle(input, minx, 3, new Scalar(255, 0, 0), 3);
            Imgproc.circle(input, miny, 3, new Scalar(255, 0, 0), 3);
            Collections.sort(Sides, new SortSides());
            Point[] p2 = (Point[]) Sides.get(2);
            Point[] p1 = (Point[]) Sides.get(3);
            int flag = 0;
            for (int i = 0; i < 2; i++) {
                telemetry.addData("time", i);
                telemetry.update();
                if (p1[0] == p2[i]) {
                    Point[] group2 = new Point[]{p1[0]};
                    if (maxx == p1[0]) {
                        Point[] group1 = new Point[]{minx, miny, maxy};
                        findangle(average(group1),average(group2));
                        Imgproc.circle(input, average(group1), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.circle(input, average(group2), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.line(input,average(group1),average(group2),new Scalar(0,255,0),2);
                    }
                    if (maxy == p1[0]) {
                        Point[] group1 = new Point[]{minx, miny, maxx};
                        findangle(average(group1),average(group2));
                        Imgproc.circle(input, average(group1), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.circle(input, average(group2), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.line(input,average(group1),average(group2),new Scalar(0,255,0),2);
                    }
                    if (minx == p1[0]) {
                        Point[] group1 = new Point[]{miny, maxx, maxy};
                        findangle(average(group1),average(group2));
                        Imgproc.circle(input, average(group1), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.circle(input, average(group2), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.line(input,average(group1),average(group2),new Scalar(0,255,0),2);
                    }
                    if (miny == p1[0]) {
                        Point[] group1 = new Point[]{minx, maxx, maxy};
                        findangle(average(group1),average(group2));
                        Imgproc.circle(input, average(group1), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.circle(input, average(group2), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.line(input,average(group1),average(group2),new Scalar(0,255,0),2);
                    }
                    flag = 1;
                    telemetry.addData("time", "special");

                }
                if (p1[1] == p2[i]) {
                    Point[] group2 = new Point[]{p1[1]};
                    if (maxx == p1[1]) {
                        Point[] group1 = new Point[]{minx, miny, maxy};
                        findangle(average(group1),average(group2));
                        Imgproc.circle(input, average(group1), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.circle(input, average(group2), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.line(input,average(group1),average(group2),new Scalar(0,255,0),2);
                    }
                    if (maxy == p1[1]) {
                        Point[] group1 = new Point[]{minx, miny, maxx};
                        findangle(average(group1),average(group2));
                        Imgproc.circle(input, average(group1), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.circle(input, average(group2), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.line(input,average(group1),average(group2),new Scalar(0,255,0),2);
                    }
                    if (minx == p1[1]) {
                        Point[] group1 = new Point[]{miny, maxx, maxy};
                        findangle(average(group1),average(group2));
                        Imgproc.circle(input, average(group1), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.circle(input, average(group2), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.line(input,average(group1),average(group2),new Scalar(0,255,0),2);
                    }
                    if (miny == p1[1]) {
                        Point[] group1 = new Point[]{minx, maxx, maxy};
                        findangle(average(group1),average(group2));
                        Imgproc.circle(input, average(group1), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.circle(input, average(group2), 3, new Scalar(0, 255, 0), 3);
                        Imgproc.line(input,average(group1),average(group2),new Scalar(0,255,0),2);
                    }
                    flag = 1;
                    telemetry.addData("time", "special");
                }
            }
            if (flag == 0) {
                p2 = (Point[]) Sides.get(0);
                p1 = (Point[]) Sides.get(1);
                if (dist(p1) > dist(p2)) {
                    Point[] group1 = p1.clone();
                    Point[] group2 = p2.clone();
                    findangle(average(group1),average(group2));
                    Imgproc.circle(input, average(group1), 3, new Scalar(0, 255, 0), 3);
                    Imgproc.circle(input, average(group2), 3, new Scalar(0, 255, 0), 3);
                    Imgproc.line(input,average(group1),average(group2),new Scalar(0,255,0),2);
                    telemetry.addData("time", "normal");
                } else {
                    Point[] group1 = p1.clone();
                    Point[] group2 = p2.clone();
                    findangle(average(group1),average(group2));
                    Imgproc.circle(input, average(group1), 3, new Scalar(0, 255, 0), 3);
                    Imgproc.circle(input, average(group2), 3, new Scalar(0, 255, 0), 3);
                    Imgproc.line(input,average(group1),average(group2),new Scalar(0,255,0),2);
                    telemetry.addData("time", "normal");
                }
            }


            return input;
        }
        public double return_angle(){
            return -Math.toDegrees(angle);
        }
        private void findangle(Point a,Point b){
            double dy=a.y-b.y;
            double hypo=dist(new Point[]{a,b});
            angle=Math.asin(dy/hypo);
        }
        private double dist(Point[] a) {
            return Math.sqrt((a[0].x - a[1].x) * (a[0].x - a[1].x) + (a[0].y - a[1].y) * (a[0].y - a[1].y));
        }

        private Point average(Point[] a) {
            int x = 0;
            int y = 0;
            for (int i = 0; i < a.length; i++) {
                x += a[i].x;
                y += a[i].y;
            }
            return new Point((int) x / a.length, (int) y / a.length);
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;
            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }

    class SortSides implements Comparator<Point[]> {
        private double dist(Point[] a) {
            return Math.sqrt((a[0].x - a[1].x) * (a[0].x - a[1].x) + (a[0].y - a[1].y) * (a[0].y - a[1].y));
        }

        @Override
        public int compare(Point[] points, Point[] t1) {

            return (int) (dist(points) - dist(t1));
        }
    }
}