/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.osgi.OpenCVInterface;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import org.opencv.core.*;
import java.util.List;
import java.util.ArrayList;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void startCamera() {
        new Thread(() -> {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(160, 120);

            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Auto", 160, 120);

            Mat source = new Mat();
            Mat output = new Mat();

            while (!Thread.interrupted()) {
                cvSink.grabFrame(source);
                if (!source.empty()) {
                    processFrame(source, output);
                    outputStream.putFrame(output);
                }
            }
        }).start();
    }

    public void stopCamera() {
        CameraServer.getInstance().removeCamera("USB Camera 0");
        CameraServer.getInstance().removeCamera("Auto");
    }

    private double threshold = 127;
    private int elementType = Imgproc.CV_SHAPE_ELLIPSE;
    private int kernelSize = 2;
    Mat element = Imgproc.getStructuringElement(elementType, new Size(2 * kernelSize + 1, 2 * kernelSize + 1), new Point(kernelSize, kernelSize));
    private void processFrame(Mat source, Mat output) {
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        Imgproc.threshold(output, output, threshold, 255, Imgproc.THRESH_BINARY);
        Imgproc.erode(output, output, element);
        Imgproc.dilate(output, output, element);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(output, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() > 0) {
            RotatedRect rect = contours.stream().map(contour -> Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()))).max((a, b) -> {
                Double difference = (a.size.width * a.size.height) - (b.size.width * b.size.height);
                return difference.intValue();
            }).get();
            Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2BGR);
            // source.copyTo(output);
            Imgproc.ellipse(output, rect, new Scalar(0, 0, 255));
            double length = rect.size.width;
            if (rect.size.height > rect.size.width) {
                length = rect.size.height;
                rect.angle += 90;
            }
            Point end1 = pointAddVector(rect.center.x, rect.center.y, rect.angle, length / 2);
            Point end2 = pointAddVector(rect.center.x, rect.center.y, rect.angle + 180, length / 2);
            Point end = end1;
            Point middle = new Point(output.width() / 2, output.height() / 2);
            if (distanceBetweenPoints(end2, middle) < distanceBetweenPoints(end1, middle)) {
                end = end2;
            }
            Imgproc.circle(output, new Point(end.x, end.y), 5, new Scalar(0, 255, 0));
        }
    }

    Point pointAddVector(double x, double y, double a, double d) {
        return new Point(x + Math.cos(Math.toRadians(a)) * d, y + Math.sin(Math.toRadians(a)) * d);
    }

    double distanceBetweenPoints(Point a, Point b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
