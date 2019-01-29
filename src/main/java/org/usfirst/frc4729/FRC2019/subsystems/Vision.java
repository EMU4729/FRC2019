package org.usfirst.frc4729.FRC2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.imgproc.Imgproc;
import org.opencv.osgi.OpenCVInterface;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import org.opencv.core.*;
import java.util.List;
import java.util.ArrayList;
import java.util.stream.Collectors;
import java.util.Optional;

public class Vision extends Subsystem {
    private boolean debug = true;

    public boolean gafferAvailable = false;
    public boolean gafferEndVisible = false;
    public double gafferOffsetX = 0;
    public double gafferAngle = 0;

    public boolean retroreflectiveAvailable = false;            
    public double retroreflectiveOffsetX = 0;

    static final int GAFFER = 1;
    static final int RETROREFLECTIVE = 2;

    public void startCamera() {
        // setupCamera(GAFFER, "Gaffer", 160, 120);
        setupCamera(RETROREFLECTIVE, "Retroreflective", 160, 120);
    }

    public void stopCamera() {
        CameraServer.getInstance().removeCamera("USB Camera 0");
        CameraServer.getInstance().removeCamera("Auto");
    }

    private void setupCamera(int camera, String label, int width, int height) {
        new Thread(() -> {
            UsbCamera newCamera = CameraServer.getInstance().startAutomaticCapture();
            newCamera.setResolution(width, height);

            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo(label, width, height);

            Mat source = new Mat();
            Mat output = new Mat();

            while (!Thread.interrupted()) {
                cvSink.grabFrame(source);
                if (!source.empty()) {
                    if (camera == GAFFER) {
                        processGaffer(source, output);
                    } else if (camera == RETROREFLECTIVE) {
                        processRetroreflective(source, output);
                    }
                    outputStream.putFrame(output);
                }
            }
        }).start();
    }

    private double threshold = 127;
    private int elementType = Imgproc.CV_SHAPE_ELLIPSE;
    private int kernelSize = 2;
    private Mat element = Imgproc.getStructuringElement(elementType, new Size(2 * kernelSize + 1, 2 * kernelSize + 1), new Point(kernelSize, kernelSize));
    
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

            Imgproc.ellipse(output, rect, new Scalar(0, 0, 255));

            Point[] ends = findEnds(rect);

            Imgproc.circle(output, new Point(rect.center.x, rect.center.y), 5, new Scalar(0, 255, 0));

            boolean bothEndsVisible = endVisible(ends[0], output) && endVisible(ends[1], output);

            gafferOffsetX = rect.center.x - (output.width() / 2);
            
            sortPairByHeight(ends);
            gafferAngle = angleBetweenPoints(ends[0], ends[1]);
            gafferAngle += 90;
            
            if (bothEndsVisible) {
                gafferAvailable = false;
                gafferEndVisible = false;
                gafferOffsetX = 0;
                gafferAngle = 0;
            } else {
                gafferAvailable = true;
            }

            // SmartDashboard.putBoolean("available", available);
            // SmartDashboard.putNumber("end.x", end.x);
            // SmartDashboard.putNumber("end.y", end.y);
            // SmartDashboard.putNumber("output.width()", output.width());
            // SmartDashboard.putNumber("output.height()", output.height());
            // SmartDashboard.putBoolean("endVisible", endVisible);
            // SmartDashboard.putNumber("offset.x", offset.x);
            // SmartDashboard.putNumber("offset.y", offset.y);
            // SmartDashboard.putNumber("angle", angle);
        }
    }

    private static final double targetH = 71;
    private static final double rangeH = 20;
    private static final double minS = 128;
    private static final double minV = 128;
    // private static final double anglesMirroredTolerance = 20;
    private static final double similarWidthsTolerance = 40;
    private static final double similarHeightsTolerance = 30;

    private void processRetroreflective(Mat source, Mat output) {
        //remove colours that arent green
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2HSV);
        Core.inRange(output,
                     new Scalar(targetH - rangeH / 2, minS, minV), 
                     new Scalar(targetH + rangeH / 2, 255, 255),
                     output);
        
        List<RotatedRect> rects = findRects(output, output);
        List<RotatedRect> leftRects = new ArrayList<RotatedRect>();
        List<RotatedRect> rightRects = new ArrayList<RotatedRect>();
        List<RotatedRect[]> pairs = new ArrayList<RotatedRect[]>();

        Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2BGR);

        for (RotatedRect rect : rects) {
            Imgproc.ellipse(output, rect, new Scalar(0, 0, 255));
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
                double leftAngle = retroreflectiveAngle(left);
                double rightAngle = retroreflectiveAngle(right);
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

        if (max.isPresent()) {
            RotatedRect[] pair = max.get();
            retroreflectiveAvailable = true;
            retroreflectiveOffsetX =  ((pair[0].center.x + pair[1].center.x) / 2)
                                     - (output.width() / 2);
            Imgproc.circle(output, new Point((pair[0].center.x + pair[1].center.x) / 2, (pair[0].center.y + pair[1].center.y) / 2), 5, new Scalar(0, 255, 0));
        } else {
            retroreflectiveAvailable = false;
            retroreflectiveOffsetX = 0;
        }

        SmartDashboard.putBoolean("available", retroreflectiveAvailable);
        SmartDashboard.putNumber("offsetX", retroreflectiveOffsetX);
    }

    private List<RotatedRect> findRects(Mat source, Mat output) {
        Imgproc.erode(output, output, element);
        Imgproc.dilate(output, output, element);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(output, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        
        return contours.stream().map(contour -> Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()))).collect(Collectors.toList());
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
                                                                                                                                                                                                                                                                        