package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;


    /*
     *  Pipeline just removing all the duplicated from BasicDetectionPipeline. This will
     *  most likely serve as the first Centerstage pipeline, if not the final
     *
     *  jpc
     */

public class CenterstagePipeline extends OpenCvPipeline {
    Telemetry telemetry;

    public CenterstagePipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    // the bounds for detecting a specific color
    // they are made public static because then they can be edited with EOCV-Sim
    public static Scalar lowerBound1 = new Scalar(0, 162, 0, 0);          // RED
                // J. M. Alexander Middle School
                // (0, 162, 0, 0)
    public static Scalar upperBound1 = new Scalar(130, 255, 255, 255);
                // J. M. Alexander Middle School
                // (130, 255, 255, 255)

    public static Scalar lowerBound2 = new Scalar(0, 0, 150, 0);          // BLUE
                // J. M. Alexander Middle School
                // (0, 0, 155, 0)
    public static Scalar upperBound2 = new Scalar(255, 255, 255, 255);
                // J. M. Alexander Middle School
                // (255, 255, 255, 255)


    // the states that can be detected
    public enum detectionStates {
        ONE,
        TWO,
        THREE
    }

    // declares the current detected state to ONE
    detectionStates state = detectionStates.ONE;

    // an array of the matrices to release
    private ArrayList<Mat> matsToRelease = new ArrayList<>();

    // releases all the memory leaks
    public void releaseMemoryLeaks() {
        // runs for every mat in matsToRelease
        for (Mat m: matsToRelease) {
            // releases the current mat, m
            m.release();
        }
    }

    // adds all these variables to the list of mats to release
    public void roundupMemory(Mat... Mats) {
        matsToRelease.addAll(Arrays.asList(Mats));
    }

    // finds and then returns the largest contour
    public MatOfPoint findLargestContour(ArrayList <MatOfPoint> detectedCountours) {
        // return a blank MatOfPoint if there's no detected contours
        if (detectedCountours.isEmpty()) {
            return new MatOfPoint();
        }

        // the biggest contour size is defined as the first contour's width times height, or area
        int biggestContourSize = detectedCountours.get(0).width() * detectedCountours.get(0).height();
        // the biggest contour is defined as the first contour
        MatOfPoint biggestContour = detectedCountours.get(0);

        // runs for each contour, m
        for (MatOfPoint m:detectedCountours) {
            // the current area is the contour's width times height
            int currentArea = m.width() * m.height();
            // runs if the current area is larger than the previously recorded largest
            if (currentArea > biggestContourSize) {
                // sets the biggest contour size to be the current contour area
                biggestContourSize = currentArea;
                // sets sets the biggest contour d=to be the current contour
                biggestContour = m;
            }
        }

        // returns the biggest contour
        return biggestContour;
    }

    @Override
    public Mat processFrame(Mat input) {
        // input is the webcam image

        // releases the memory leaks from the previous loop
        releaseMemoryLeaks();

        // used to store the color shifted input
        Mat colorShiftedIMG = new Mat();

        // used to store different processed versions of the input
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();

        // converts the image's color from RGB to the YCrCb color space
        Imgproc.cvtColor(input, colorShiftedIMG, Imgproc.COLOR_RGB2YCrCb);
        // blurs the image a little
        Imgproc.medianBlur(colorShiftedIMG, colorShiftedIMG, 5);

        // gets exampleMat, a color shifted version of the input image, and converts it to
        // a binary image based on whether each pixel is within a certain range
        // this binary output image is stored as mask{num} for each corresponding range
        Core.inRange(colorShiftedIMG, lowerBound1, upperBound1, mask1);
        Core.inRange(colorShiftedIMG, lowerBound2, upperBound2, mask2);

        // detects the contours in mask{num} and stores them in the array detectedContours{num}
        ArrayList<MatOfPoint> detectedContours1 = new ArrayList<>();
        Imgproc.findContours(mask1,detectedContours1,new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        ArrayList<MatOfPoint> detectedContours2 = new ArrayList<>();
        Imgproc.findContours(mask2,detectedContours2,new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        // finds and stores the biggest contour in each detectedContours
        MatOfPoint biggestContour1 = findLargestContour(detectedContours1);
        MatOfPoint biggestContour2 = findLargestContour(detectedContours2);

        // adds all the masks together so they can all display at the same time
        Mat sum = new Mat();
        Core.add(mask1, mask2, sum);

        // creates and draws a bounding box around the largest contour in each mask
        Rect boundingBox1 = Imgproc.boundingRect(biggestContour1);
        Imgproc.rectangle(sum, boundingBox1, new Scalar(255, 255, 255));
        Rect boundingBox2 = Imgproc.boundingRect(biggestContour2);
        Imgproc.rectangle(sum, boundingBox2, new Scalar(255, 255, 255));

        // finding the moments of each contour
        Moments moments1 = Imgproc.moments(biggestContour1);
        Moments moments2 = Imgproc.moments(biggestContour2);

        // math for finding the average x and y position of the middle of the contours
        double avgX1 = moments1.m10/moments1.m00;
        double avgY1 = moments1.m01/moments1.m00;

        double avgX2 = moments2.m10/moments2.m00;
        double avgY2 = moments2.m01/moments2.m00;

        // variable just holding whether red or blue is bigger
        boolean redIsBigger = moments1.m00 > moments2.m00;

        // quick telemetry for diagnosing
        telemetry.addData("x1", avgX1);
        telemetry.addData("y1", avgY1);
        telemetry.addData("is red bigger?", redIsBigger);



        /*
        // size of smallest group of pixels to remove, about 5 pixels
        Mat strElement = Imgproc.getStructuringElement(CV_SHAPE_ELLIPSE, new Size(5, 5));

        // new mats for eroding the little pixels on the screen not a part of the main contour
        Mat erodedBinaryMat1 = new Mat();
        Mat erodedBinaryMat2 = new Mat();

        // running erode function to the size of strElement, and saving to erodedBinaryMat
        Imgproc.erode(mask1, erodedBinaryMat1, strElement);
        Imgproc.erode(mask2, erodedBinaryMat2, strElement);


        // new mats for the dilated contours
        Mat dialatedBinaryMat1 = new Mat();
        Mat dialatedBinaryMat2 = new Mat();


        // dilating pixels back out for each contour we want to keep
        Imgproc.dilate(erodedBinaryMat1, dialatedBinaryMat1, strElement);
        Imgproc.dilate(erodedBinaryMat2, dialatedBinaryMat2, strElement);

         */









        // resets the state to be ONE
        state = detectionStates.ONE;           // RIGHT SIDE

        // changes the state from ONE to TWO if the position of the prop is on the center line
        if (((avgY1 >= 30 && avgY1 <= 170) && moments1.m00 > 6000) || ((avgY2 >= 30 && avgY2 <= 170) && moments2.m00 > 6000)) {                              // Based off of x = 490, about 8800 big
            state = detectionStates.TWO;       // CENTER SIDE
        }

        // changes the state to THREE if the prop is on the right line
        if (((avgY1 >= 360 && avgY1 <= 550) && moments1.m00 > 11000) || ((avgY2 >= 360 && avgY2 <= 550) && moments2.m00 > 11000)) {                             // Based off of x = 150, about 16000 big
            state = detectionStates.THREE;     // LEFT SIDE
        }

        // displays the detected state in telemetry
        telemetry.addData("Detected State", state);
        telemetry.addLine();
        telemetry.addData("Size of detected contour", moments1.m00);
        telemetry.addLine();
        // quick telemetry for diagnosing
        telemetry.addData("x1", avgX1);
        telemetry.addData("y1", avgY1);
        telemetry.addData("x2", avgX2);
        telemetry.addData("y2", avgY2);
        telemetry.addData("Is red bigger?", redIsBigger);
        telemetry.update();

        // adds these variables to the array of things to release
        roundupMemory(mask1, mask2, colorShiftedIMG);

        // returns the sum of the masks
        return sum;
        //return mask2;
    }

    // can be called get the get the current state
    public detectionStates getState() {
        return state;
    }
}
