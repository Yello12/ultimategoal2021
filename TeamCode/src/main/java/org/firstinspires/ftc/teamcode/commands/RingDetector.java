package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;

import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RingDetector extends OpenCvPipeline {
    enum RingStack {
        NONE,
        ONE,
        FOUR
    }

    private final double MIN_ONE_SIZE = 2000;
    private final double MIN_FOUR_SIZE = 6000;
    private double cur_area = 0;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    RingStack height;

    Mat erode = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat dilate = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

    public RingDetector() {
        this.height = RingStack.NONE;
    }


    @Override
    public Mat processFrame(Mat frame) {

        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        if (hsv.empty()) {
            this.height = RingStack.NONE;
            return frame;
        }

        Scalar low_hsv = new Scalar(Constants.CAM_LOW_H, Constants.CAM_LOW_S, Constants.CAM_LOW_V);
        Scalar high_hsv = new Scalar(Constants.CAM_HIGH_H, Constants.CAM_HIGH_S, Constants.CAM_HIGH_V);

        Mat thresh = new Mat();
        Core.inRange(hsv, low_hsv, high_hsv, thresh);


        Imgproc.erode(thresh, thresh, erode);
        Imgproc.dilate(thresh, thresh, dilate);
        Imgproc.dilate(thresh, thresh, dilate);
        Imgproc.erode(thresh, thresh, erode);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.size() > 0) {
            MatOfPoint2f[] contours_poly = new MatOfPoint2f[contours.size()];
            Rect[] bounding_boxes = new Rect[contours.size()];
            for (int i = 0; i < contours.size(); i++) {
                contours_poly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contours_poly[i], 3, true);
                bounding_boxes[i] = Imgproc.boundingRect(new MatOfPoint(contours_poly[i].toArray()));
            }
            int largest_area = -1;
            int largest_index = 0;
            for (int i = 0; i < contours.size(); i++) {
                int area = bounding_boxes[i].width * bounding_boxes[i].height;
                int x = bounding_boxes[i].x;
                if (area > largest_area && x >= 180) {
                    largest_index = i;
                    largest_area = area;
                }
            }
            cur_area = largest_area;
            if (cur_area >= MIN_FOUR_SIZE) {
                this.height = RingStack.FOUR;
            } else if (cur_area >= MIN_ONE_SIZE) {
                this.height = RingStack.ONE;
            } else {
                this.height = RingStack.NONE;
            }
            Imgproc.rectangle(frame, bounding_boxes[largest_index], new Scalar(255, 0, 0));
            Imgproc.drawContours(frame, contours, -1, new Scalar(0, 255, 0), 3, 8);
            dashboardTelemetry.addData("Area", largest_area);

            dashboardTelemetry.update();
        }

        return frame;
    }

    public RingStack getHeight() {
        return this.height;
    }
}