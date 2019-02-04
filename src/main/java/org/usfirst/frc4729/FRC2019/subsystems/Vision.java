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

public class Vision extends Subsystem {
    private boolean debug = false;

    public boolean gafferAvailable = false;
    public double gafferOffsetX = 0;
    public double gafferAngle = 0;

    public boolean retroreflectiveAvailable = false;            
    public double retroreflectiveOffsetX = 0;

    private static final int GAFFER = 1;
    private static final int RETROREFLECTIVE = 2;
    private static final int gafferCameraWidth = 160;
    private static final int gafferCameraHeight = 120;
    private static final int retroreflectiveCameraWidth = 160;
    private static final int retroreflectiveCameraHeight = 120;

    public void startCameras() {
        new Thread(() -> {
            CameraServer instance = CameraServer.getInstance();
            VideoSink server = instance.getServer();
            UsbCamera camera0 = instance.startAutomaticCapture(0);
            UsbCamera camera1 = instance.startAutomaticCapture(0);
            camera0.setResolution(gafferCameraWidth, gafferCameraHeight);
            camera1.setResolution(retroreflectiveCameraWidth, retroreflectiveCameraHeight);

            CvSink cvSink = instance.getVideo();
            CvSource outputStream = instance.putVideo(label, width, height);

            Mat source = new Mat();
            Mat output = new Mat();

            int processor = GAFFER;

            while (!Thread.interrupted()) {
                cvSink.grabFrame(source);
                if (!source.empty()) {
                    switch (processor) {
                    case GAFFER:
                        server.setSource(camera0);
                        processGaffer(source, output);
                        processor = RETROREFLECTIVE;
                        break;
                    case RETROREFLECTIVE:
                        server.setSource(camera1);
                        processRetroreflective(source, output);
                        processor = GAFFER;
                        break;
                    }
                    outputStream.putFrame(output);
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

        List<RotatedRect> rects = findRects(source, output);

        if (rects.size() > 0) {
            RotatedRect rect = rects.stream().max((a, b) -> {
                Double difference = (a.size.width * a.size.height) - (b.size.width * b.size.height);
                return difference.intValue();
            }).get();

            Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2BGR);
            if (!debug) source.copyTo(output);

            Point[] ends = findEnds(rect);

            boolean bothEndsVisible = endVisible(ends[0], output) && endVisible(ends[1], output);

            gafferOffsetX = rect.center.x - (output.width() / 2);
            
            sortPairByHeight(ends);
            gafferAngle = angleBetweenPoints(ends[0], ends[1]) + 90;
            
            gafferAvailable = (!bothEndsVisible && Math.abs(rect.size.width - gafferWidthTarget) < gafferWidthTolerance);

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

    private static final double targetH = 71;
    private static final double rangeH = 20;
    private static final double minS = 128;
    private static final double targetV = 128;
    private static final double rangeV = 20;
    // private static final double anglesMirroredTolerance = 20;
    private static final double similarWidthsTolerance = 40;
    private static final double similarHeightsTolerance = 30;

    private void processRetroreflective(Mat source, Mat output) {
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2HSV);
        Core.inRange(output,
                     new Scalar(targetH - rangeH / 2, minS, targetV - rangeV / 2), 
                     new Scalar(targetH + rangeH / 2, 255,  targetV + rangeV / 2),
                     output);
        
        List<RotatedRect> rects = findRects(output, output);
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
            if (angle < 90) {
                leftRects.add(rect);
            } else {
                rightRects.add(rect);
            }
        }

        for (RotatedRect left : leftRects) {
            for (RotatedRect right : rightRects) {
                // double leftAngle = retroreflectiveAngle(left);
                // double rightAngle = retroreflectiveAngle(right);
                // boolean anglesMirrored = (Math.abs((90 - leftAngle) - (rightAngle - 90)) < anglesMirroredTolerance);
                boolean similarWidths = (Math.abs(left.size.width - right.size.width) < similarWidthsTolerance);
                boolean similarHeights = (Math.abs(left.size.height - right.size.height) < similarHeightsTolerance);
                if (/*anglesMirrored && */similarWidths && similarHeights) {
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
            retroreflectiveOffsetX   = ((pair[0].center.x + pair[1].center.x) / 2)
                                     - (output.width() / 2);
            Imgproc.ellipse(output, pair[0], new Scalar(0, 0, 255));
            Imgproc.ellipse(output, pair[1], new Scalar(0, 0, 255));
            Imgproc.circle(output, new Point((pair[0].center.x + pair[1].center.x) / 2, (pair[0].center.y + pair[1].center.y) / 2), 5, new Scalar(0, 255, 0));
        } else {
            retroreflectiveOffsetX = 0;
        }


        SmartDashboard.putBoolean("available", retroreflectiveAvailable);
        SmartDashboard.putNumber("offsetX", retroreflectiveOffsetX);
    }

    private int elementType = Imgproc.CV_SHAPE_ELLIPSE;
    private int kernelSize = 2;
    private Mat element = Imgproc.getStructuringElement(elementType,
                                                        new Size(2 * kernelSize + 1, 2 * kernelSize + 1),
                                                        new Point(kernelSize, kernelSize));

    private List<RotatedRect> findRects(Mat source, Mat output) {
        Imgproc.erode(output, output, element);
        Imgproc.dilate(output, output, element);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(output, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        return contours.stream().map(
            contour -> Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()))
        ).collect(Collectors.toList());
    }

    private Point[] findEnds(RotatedRect rect) {
        Point[] ends = new Point[2];
        double length = rect.size.width;
        if (rect.size.height > rect.size.width) {
            length = rect.size.height;
            rect.angle += 90;
        }
        ends[0] = pointAddVector(rect.center.x, rect.center.y, rect.angle, length / 2);
        ends[1] = pointAddVector(rect.center.x, rect.center.y, rect.angle + 180, length / 2);
        return ends;
    }

    private boolean endVisible(Point end, Mat output) {
        return (end.x > 1 &&
                end.x < output.width() - 1 &&
                end.y > 1 &&
                end.y < output.height() - 1);
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
                                                                                                                                                                                                                                                                        