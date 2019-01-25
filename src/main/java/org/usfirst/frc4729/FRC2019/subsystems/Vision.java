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

public class Vision extends Subsystem {
    private boolean debug = true;

    public boolean gafferAvailable = false;
    public boolean gafferEndVisible = false;
    public Point gafferCenter = new Point(0, 0);
    public Point gafferOffset = new Point(0, 0);
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
                        processRectroreflective(source, output);
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

            double length = rect.size.width;
            if (rect.size.height > rect.size.width) {
                length = rect.size.height;
                rect.angle += 90;
            }

            Point end1 = pointAddVector(rect.center.x, rect.center.y, rect.angle, length / 2);
            Point end2 = pointAddVector(rect.center.x, rect.center.y, rect.angle + 180, length / 2);
            Point end = end1;
            Point middle = new Point(output.width() / 2, output.height() / 2);
            double distanceDifference = distanceBetweenPoints(end1, middle) - distanceBetweenPoints(end2, middle);
            
            if (distanceDifference > 3) {
                end = end2;
            } else if (distanceDifference <= 3 && distanceDifference > -3) {
                if (end2.y > end1.y) {
                    end = end2;
                }
            }

            Imgproc.circle(output, new Point(end.x, end.y), 5, new Scalar(0, 255, 0));
            gafferEndVisible = (end.x > 1 &&
                          end.x < output.width() - 1 &&
                          end.y > 1 &&
                          end.y < output.height() - 1);
            boolean bothEndsVisible = (end1.x > 1 &&
                                       end1.x < output.width() - 1 &&
                                       end1.y > 1 &&
                                       end1.y < output.height() - 1 &&
                                       end2.x > 1 &&
                                       end2.x < output.width() - 1 &&
                                       end2.y > 1 &&
                                       end2.y < output.height() - 1);
            gafferCenter.x = output.width() / 2;
            gafferCenter.y = output.height() / 2;
            gafferOffset.x = end.x - gafferCenter.x;
            gafferOffset.y = end.y - gafferCenter.y;
            
            if (end1.y < end2.y) {
                gafferAngle = angleBetweenPoints(end2, end1);
            } else {
                gafferAngle = angleBetweenPoints(end1, end2);
            }
            gafferAngle = Math.toDegrees(gafferAngle);
            gafferAngle += 90;
            
            if (bothEndsVisible) {
                gafferAvailable = false;
                gafferEndVisible = false;
                gafferOffset.x = 0;
                gafferOffset.y = 0;
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

    private static final double targetH = 110;
    private static final double rangeH = 40;
    private static final double minS = 128;
    private static final double minV = 128;
    private static final double anglesMirroredTolerance = 20;
    private static final double similarWidthsTolerance = 20;
    private static final double similarHeightsTolerance = 20;

    private void processRectroreflective(Mat source, Mat output) {
        //remove colours that arent green
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2HSV);
        Core.inRange(output, new Scalar(targetH - rangeH / 2, minS, minV), 
                             new Scalar(targetH + rangeH / 2, 255, 255), output);
          
        List<RotatedRect> rects = findRects(source, output);
        List<RotatedRect> right = new ArrayList<RotatedRect>();
        List<RotatedRect> left = new ArrayList<RotatedRect>();
        List<List<RotatedRect>> pairs = new ArrayList<List<RotatedRect>>();

        for (RotatedRect rect : rects) {
            if (rect.angle < 90) {
                left.add(rect);
            } else {
                right.add(rect);
            }
        }

        for (RotatedRect r : right) {     // right = r
            for (RotatedRect l : left) {  // left = l
                boolean anglesMirrored = (Math.abs((r.angle - 90) - (90 - l.angle)) < anglesMirroredTolerance);
                boolean similarWidths = (Math.abs(r.size.width - l.size.width) < similarWidthsTolerance);
                boolean similarHeights = (Math.abs(r.size.height - l.size.height) < similarHeightsTolerance);
                if (anglesMirrored && similarWidths && similarHeights) {
                    List<RotatedRect> pair = new ArrayList<RotatedRect>();
                    pair.add(r);
                    pair.add(l);
                    pairs.add(pair);
                }
            }
        }

        List<RotatedRect> pair = pairs.stream().max((a, b) -> {
            Double difference =   (((a.get(0).size.width * a.get(0).size.height) + (a.get(1).size.width * a.get(1).size.height)) / 2)
                                - (((b.get(0).size.width * b.get(0).size.height) + (b.get(1).size.width * b.get(1).size.height)) / 2);
            return difference.intValue();
        }).get();

        if (pair != null) {
            retroreflectiveAvailable = true;
            retroreflectiveOffsetX = ((pair.get(0).center.x + pair.get(1).center.x) / 2) - (output.width() / 2);
        } else {
            retroreflectiveAvailable = false;
            retroreflectiveOffsetX = 0;
        }
    }

    private List<RotatedRect> findRects(Mat source, Mat output) {
        Imgproc.erode(output, output, element);
        Imgproc.dilate(output, output, element);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(output, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        
        return contours.stream().map(contour -> Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()))).collect(Collectors.toList());
    }

    Point pointAddVector(double x, double y, double a, double d) {
        return new Point(x + Math.cos(Math.toRadians(a)) * d, y + Math.sin(Math.toRadians(a)) * d);
    }

    double distanceBetweenPoints(Point a, Point b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }

    double angleBetweenPoints(Point a, Point b) {
        return Math.atan2(b.y - a.y, b.x - a.x);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
                                                                                                                                                                                                                                                                        