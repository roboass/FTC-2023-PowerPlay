package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.CopyOnWriteArraySet;


public class ImageDetector extends OpenCvPipeline {

    private Mat workingMatrix = new Mat();
    public static String color = "GREEN";
    public static double redValue, greenValue, blueValue;
    public int rows = 720, cols = 1080;
    int HEIGHT = 50, WIDTH = 50;

    Point ss = new Point(600  , 450);

    Point dj = new Point(ss.x + HEIGHT, ss.y + WIDTH);

    Mat pos;

    public ImageDetector() {}

    @Override
    public void init(Mat frame)
    {
        pos = frame.submat(new Rect(ss, dj));
    }
    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if(workingMatrix.empty()) {
            return input;
        }

        Imgproc.rectangle(workingMatrix, ss, dj, new Scalar(255, 0, 0), 4);

        redValue = HEIGHT*WIDTH*255 - Core.sumElems(pos).val[1];
        greenValue = HEIGHT*WIDTH*255 - Core.sumElems(pos).val[2];
        blueValue = HEIGHT*WIDTH*255 - Core.sumElems(pos).val[3];

        if(redValue < 50 && greenValue < 50 && blueValue < 50){
            color = "BLACK";
        } else if(redValue > 200 && greenValue > 200 & blueValue > 200){
            color = "WHITE";
        } else if(redValue < 50 && greenValue > 200 && blueValue < 50){
            color = "GREEN";
        }

        return workingMatrix;
    }
    public String getColor() {
        return color;
    }

}
