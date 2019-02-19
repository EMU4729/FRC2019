package org.usfirst.frc4729.FRC2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.core.*;
import java.util.List;
import java.util.ArrayList;
import java.util.stream.Collectors;
import java.util.Optional;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import org.usfirst.frc4729.FRC2019.Util;

public class Vision extends Subsystem {
    private boolean debug = true;

    public boolean gafferAvailable = false;
    public boolean gafferEndVisible = false;
    public double gafferOffsetX = 0;
    public double gafferAngle = 0;

    public boolean retroreflectiveAvailable = false;            
    public double retroreflectiveOffsetX = 0;
    public double retroreflectiveRelativeAngle = 0;

    public final double cameraConeHalfAngle = 61 / 2;

    private static final int numCameras = 2;

    private static final int GAFFER = 1;
    private static final int RETROREFLECTIVE = 0;
    
    private static final Size[] sizes = new Size[numCameras];
    private static final String[] labels = new String[numCameras];
    

    public Vision() {
        sizes[GAFFER] = new Size(160, 120);
        // sizes[RETROREFLECTIVE] = new Size(640, 480);
        sizes[RETROREFLECTIVE] = new Size(320, 240);
        // sizes[RETROREFLECTIVE] = new Size(160, 120);
        labels[GAFFER] = "Gaffer";
        labels[RETROREFLECTIVE] = "Retroreflective";
    }    

    public void startCameras() {
        CameraServer instance = CameraServer.getInstance();
        UsbCamera[] cameras = new UsbCamera[numCameras];
        CvSource[] outputStreams = new CvSource[numCameras];
        CvSink[] sinks = new CvSink[numCameras];

        for (int i = 0; i < numCameras; i++) {
            cameras[i] = instance.startAutomaticCapture(i);
            cameras[i].setResolution((int) sizes[i].width, (int) sizes[i].height);
            cameras[i].setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            outputStreams[i] = instance.putVideo(labels[i], (int) sizes[i].width, (int) sizes[i].height);
            sinks[i] = instance.getVideo("USB Camera " + i);
            setupCamera(i, outputStreams, sinks);
        }
        
        cameras[RETROREFLECTIVE].setExposureManual(20);
    }

    private void setupCamera(int camera, CvSource[] outputStreams, CvSink[] sinks) {
        new Thread(() -> {
            Mat source = new Mat();
            Mat output = new Mat();

            while (!Thread.interrupted()) {
                sinks[camera].grabFrame(source);

                if (!source.empty()) {
                    switch (camera) {
                    case GAFFER:
                        processGaffer(source, output);
                        break;
                    case RETROREFLECTIVE:
                        processRetroreflective(source, output);
                        break;
                    }

                    outputStreams[camera].putFrame(output);
                }
            }
        }).start();
    }

    private static final double threshold = 127;
    private static final double gafferWidthTarget = 50;
    private static final double gafferWidthTolerance = 20;
    
    private void processGaffer(Mat source, Mat output) {
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        Imgproc.threshold(output, output, threshold, 255, Imgproc.THRESH_BINARY);

        List<RotatedRect> rects = findRects(source, output, true);

        if (rects.size() > 0) {
            RotatedRect rect = rects.stream().max((a, b) -> {
                Double difference = (a.size.width * a.size.height) - (b.size.width * b.size.height);
                return difference.intValue();
            }).get();

            Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2BGR);
            if (!debug) source.copyTo(output);

            Point[] ends = findEnds(rect);

            Imgproc.circle(output, new Point(ends[0].x, ends[0].y), 5, new Scalar(255, 0, 0));
            Imgproc.circle(output, new Point(ends[1].x, ends[1].y), 5, new Scalar(255, 0, 0));

            boolean endVisible0 = endVisible(ends[0], output);
            boolean endVisible1 = endVisible(ends[1], output);

            SmartDashboard.putBoolean("endVisible0", endVisible0);
            SmartDashboard.putBoolean("endVisible1", endVisible1);

            boolean bothEndsVisible = (endVisible0 && endVisible1);
            gafferEndVisible = (endVisible0 || endVisible1);

            gafferOffsetX = rect.center.x - (output.width() / 2);
            
            sortPairByHeight(ends);
            gafferAngle = Util.normAngle(angleBetweenPoints(ends[0], ends[1]) + 90);
            
            gafferAvailable = true;//(!bothEndsVisible);// && Math.abs(rect.size.width - gafferWidthTarget) < gafferWidthTolerance); // TODO

            if (gafferAvailable) {
                Imgproc.ellipse(output, rect, new Scalar(0, 0, 255));
                Imgproc.circle(output, new Point(rect.center.x, rect.center.y), 5, new Scalar(0, 255, 0));
            }
        } else {
            gafferAvailable = false;
        }

        if (!gafferAvailable) {
            gafferOffsetX = 0;
            gafferAngle = 0;
        }
    }

    private static double targetH = 78;
    private static double rangeH = 40;
    private static double minS = 128;
    private static double targetV = 255;
    private static double rangeV = 80;
    // private static final double anglesMirroredTolerance = 20;
    private static double similarWidthsTolerance = 40;
    private static double similarHeightsTolerance = 30;

    private void processRetroreflective(Mat source, Mat output) {
        targetH = SmartDashboard.getNumber("target hue", targetH);
        rangeH = SmartDashboard.getNumber("range hue", rangeH);
        minS = SmartDashboard.getNumber("min saturation", minS);
        targetV = SmartDashboard.getNumber("target value", targetV);
        rangeV = SmartDashboard.getNumber("range value", rangeV);

        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2HSV);
        Core.inRange(output,
                     new Scalar(targetH - rangeH / 2, minS, targetV - rangeV / 2), 
                     new Scalar(targetH + rangeH / 2, 255,  targetV + rangeV / 2),
                     output);
        
        List<RotatedRect> rects = findRects(output, output, false);
        List<RotatedRect> leftRects = new ArrayList<RotatedRect>();
        List<RotatedRect> rightRects = new ArrayList<RotatedRect>();
        List<RotatedRect[]> pairs = new ArrayList<RotatedRect[]>();

        Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2BGR);
        if (!debug) source.copyTo(output);

        for (RotatedRect rect : rects) {
            double angle = retroreflectiveAngle(rect);
            if (rect.size.width > rect.size.height) {
                double height = rect.size.height;
                rect.size.height = rect.size.width;
                rect.size.width = height;
            }
            if (angle > 0 && angle < 90) {
                // SmartDashboard.putNumber("left rect", angle);
                leftRects.add(rect);
            } else if (angle >= 90 && angle < 180) {
                // SmartDashboard.putNumber("right rect", angle);
                rightRects.add(rect);
            }
        }

        for (RotatedRect left : leftRects) {
            for (RotatedRect right : rightRects) {
                // double leftAngle = retroreflectiveAngle(left);
                // double rightAngle = retroreflectiveAngle(right);
                // boolean anglesMirrored = (Math.abs((90 - leftAngle) - (rightAngle - 90)) < anglesMirroredTolerance);
                boolean properlyAdjacent = (left.center.x < right.center.x);
                boolean similarWidths = (Math.abs(left.size.width - right.size.width) < similarWidthsTolerance);
                boolean similarHeights = (Math.abs(left.size.height - right.size.height) < similarHeightsTolerance);
                if (/*anglesMirrored && */properlyAdjacent && similarWidths && similarHeights) {
                    RotatedRect[] pair = {left, right};
                    pairs.add(pair);
                }
            }
        }

        Optional<RotatedRect[]> max = pairs.stream().max((a, b) -> {
            Double difference =   (((a[0].size.width * a[0].size.height) + (a[1].size.width * a[1].size.height)) / 2)
                                - (((b[0].size.width * b[0].size.height) + (b[1].size.width * b[1].size.height)) / 2);
            return difference.intValue();
        });

        retroreflectiveAvailable = max.isPresent();
        if (retroreflectiveAvailable) {
            RotatedRect[] pair = max.get();
            retroreflectiveOffsetX =   ((pair[0].center.x + pair[1].center.x) / 2)
                                     - (output.width() / 2);
            retroreflectiveRelativeAngle = (retroreflectiveOffsetX / (output.width() / 2)) * cameraConeHalfAngle;
            pair[0].angle += 90;
            // pair[1].angle += 90;
            Imgproc.ellipse(output, pair[0], new Scalar(0, 0, 255));
            Imgproc.ellipse(output, pair[1], new Scalar(0, 0, 255));
            Imgproc.circle(output, new Point((pair[0].center.x + pair[1].center.x) / 2, (pair[0].center.y + pair[1].center.y) / 2), 5, new Scalar(0, 255, 0));
        } else {
            retroreflectiveOffsetX = 0;
            retroreflectiveRelativeAngle = 0;
        }
    }

    private int elementType = Imgproc.CV_SHAPE_ELLIPSE;
    private int kernelSize = 2;
    private Mat element = Imgproc.getStructuringElement(elementType,
                                                        new Size(2 * kernelSize + 1, 2 * kernelSize + 1),
                                                        new Point(kernelSize, kernelSize));

    private List<RotatedRect> findRects(Mat source, Mat output, boolean noiseFilter) {
        if (noiseFilter) {
            Imgproc.erode(output, output, element);
            Imgproc.dilate(output, output, element);
        }

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(output, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        return contours.stream().map(
            contour -> Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()))
        ).collect(Collectors.toList());
    }

    private Point[] findEnds(RotatedRect rect) {
        Point[] ends = new Point[2];
        double length = rect.size.width;
        double angle = rect.angle;
        if (rect.size.height > rect.size.width) {
            length = rect.size.height;
            angle += 90;
        }
        ends[0] = pointAddVector(rect.center.x, rect.center.y, angle, length / 2);
        ends[1] = pointAddVector(rect.center.x, rect.center.y, angle + 180, length / 2);
        return ends;
    }

    private boolean endVisible(Point end, Mat output) {
        // System.out.println(end.x);
        // System.out.println(end.y);
        // System.out.println(output.width());
        // System.out.println(output.height());
        // System.out.println((end.x > 2));
        // System.out.println((end.x < output.width() - 2));
        // System.out.println((end.y > 2));
        // System.out.println((end.y < output.height() - 2));
        // System.out.println("=====");
        return (end.x > 2 &&
                end.x < output.width() - 2 &&
                end.y > 2 &&
                end.y < output.height() - 2);
    }

    private void sortPairByHeight(Point[] ends) {
        if (ends[1].y > ends[0].y) {
            Point temp = ends[0];
            ends[0] = ends[1];
            ends[1] = temp;
        }
    }

    private double retroreflectiveAngle(RotatedRect rect) {
        Point[] ends = findEnds(rect);
        sortPairByHeight(ends);
        return -angleBetweenPoints(ends[0], ends[1]);
    }

    Point pointAddVector(double x, double y, double a, double d) {
        return new Point(x + Math.cos(Math.toRadians(a)) * d, y + Math.sin(Math.toRadians(a)) * d);
    }

    double distanceBetweenPoints(Point a, Point b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }

    double angleBetweenPoints(Point a, Point b) {
        return Math.toDegrees(Math.atan2(b.y - a.y, b.x - a.x));
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
                                                                                                                                                                                                                                                                        