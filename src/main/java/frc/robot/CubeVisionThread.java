package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a thread that is responsible for reading images from the camera and
 * detecting the cubes in the camera's view. If it detects cubes in the view, it
 * will render the contours of the cubes, highlighting all of the detected
 * cubes.
 */
public class CubeVisionThread extends Thread {

	public CvSink cvSink;
	// public CvSource HSVOutputStream, rectOutputStream;
	public List<RotatedRect> rects = new ArrayList<>();
	// public List<RotatedRect> outputRects = new ArrayList<>();
	public Mat hierarchy = new Mat(), mat = new Mat(), image = new Mat();
	public List<MatOfPoint> finalContours = new ArrayList<>();
	public CvSource HSVOutputStream, rectOutputStream;
	int rectChoice;
	public SelectorType selectionMethod = SelectorType.bottom;

	int smallestI = -1;

	/**
	 * The method by which the vision algorithm selects a cube for the getError
	 * method
	 */
	public enum SelectorType {
		rightmost, leftmost, bottom, largest;
	}

	public CubeVisionThread() {

	}

	public int resolutionWidth = 320;
	public int resolutionHeight = 240;

	/**
	 * Update the smart dashboard with diagnostics values.
	 */
	public void updateDiagnostics() {
		// place smart dashboard output here to refresh regularly in either auto or
		// teleop modes.
		// SmartDashboard.putNumber(Robot.VISION_ERROR, getError());
	}

	public static final int FPS = 7;

	/**
	 * Run the vision thread.
	 */
	@Override
	public void run() {
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(resolutionWidth, resolutionHeight);
		camera.setExposureAuto();
		// camera.setWhiteBalanceManual(10);
		// camera.setWhiteBalanceManual(value);
		camera.setFPS(30);
		CvSink cvSink = CameraServer.getInstance().getVideo();
		HSVOutputStream = CameraServer.getInstance().putVideo("Edges", resolutionWidth, resolutionHeight);
		rectOutputStream = CameraServer.getInstance().putVideo("Final", resolutionWidth, resolutionHeight);
		cvSink.grabFrame(mat);
		Mat base = new Mat();
		Mat mat = new Mat();
		Mat grey = new Mat();
		Mat edges = new Mat();

		Size blurSize = new Size(9, 9);
		Scalar colorStart = new Scalar(20, 108, 139);
		Scalar colorEnd = new Scalar(35, 255, 255);
		Size erodeSize = new Size(10, 10);
		Size dilateSize = new Size(10, 10);
		Size edgeDilateSize = new Size(4, 4);
		boolean previousState = true;
		while (!Thread.interrupted()) {
			//"State" is used to enable and disable image processing during competition, in the intrest of FPS
			boolean state = SmartDashboard.getBoolean("Enable processing", true);
			if (state) {
				if (!previousState) {
					camera.setFPS(FPS);
					camera.setExposureManual(33);
				}
				//Recive the inital image
				cvSink.grabFrame(base);
				//Blurs the image for ease of processing
				Imgproc.blur(base, mat, blurSize);
				//Converts from the RGB scale to HSV because HSV is more useful
				Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
				//COnverst Mat to a black and white image where pixils in the given range appear white
				Core.inRange(mat, colorStart, colorEnd, mat);
				//Erode and then dilate to sharpen the corners
				Imgproc.erode(mat, mat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, erodeSize));
				Imgproc.dilate(mat, mat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, dilateSize));

				//Makes a greyscale version of the origional image for edge detection
				Imgproc.cvtColor(base, grey, Imgproc.COLOR_BGR2GRAY);
				//Makes a black and white image where white pixils are edges
				Imgproc.Canny(grey, edges, 100, 200);

				//Dilate and erode to convers separate edges to continuous lines
				Imgproc.dilate(edges, edges, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, edgeDilateSize));
				Imgproc.erode(edges, edges, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));

				//Erases the edge pixils from the color filtered image
				Core.bitwise_not(edges, edges);
				Core.bitwise_and(mat, edges, mat);
				//Outputs the current image
				HSVOutputStream.putFrame(edges);
				finalContours.clear();
				//FInds the outlines of the white rectangles of the current image
				Imgproc.findContours(mat, finalContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
				rects.clear();
				//Makes rectangle objects for each of the contours
				for (int i = 0; i < finalContours.size(); i++) {
					RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(finalContours.get(i).toArray()));
					if (rect.size.height > 10 && rect.size.width > 10 && rect.angle < 10)
						rects.add(rect);
				}

				//Selects one of the rectangles based on the current selection method
				switch (selectionMethod) {
				case bottom: {
					double closest = -1;
					smallestI = -1;
					for (int i = 0; i < rects.size(); i++) {
						double y = rects.get(i).center.y;
						if (y > closest) {
							closest = y;
							smallestI = i;
						}
					}
					break;
				}
				case rightmost: {
					double closest = Integer.MIN_VALUE;
					for (int i = 0; i < rects.size(); i++) {
						if (rects.get(i).center.x > closest) {
							closest = rects.get(i).center.x;
							smallestI = i;
						}
					}
					break;
				}
				case leftmost: {
					double closest = Integer.MAX_VALUE;
					for (int i = 0; i < rects.size(); i++) {
						if (rects.get(i).center.x < closest) {
							closest = rects.get(i).center.x;
							smallestI = i;
						}
					}
					break;
				}
				case largest: {
					double closest = -1;
					for (int i = 0; i < rects.size(); i++) {
						if (rects.get(i).size.height * rects.get(i).size.height < closest) {
							closest = rects.get(i).size.height * rects.get(i).size.height;
							smallestI = i;
						}
					}
				}
				}

				//Draws the rectangles over the origional image (purely for human use)
				rectChoice = smallestI;
				for (int i = 0; i < rects.size(); i++) {
					Point[] vertices = new Point[4];
					rects.get(i).points(vertices);
					MatOfPoint points = new MatOfPoint(vertices);
					//The selected rectangle is green, others are white
					if (i != smallestI)
						Imgproc.drawContours(base, Arrays.asList(points), -1, new Scalar(255, 255, 255), 5);
					else
						Imgproc.drawContours(base, Arrays.asList(points), -1, new Scalar(0, 255, 0), 5);
				}
				//Outputs the origional image with rectangles areound cubes
				rectOutputStream.putFrame(base);

				Thread.yield();
			} 
			//If processing isn't enabled, increase FPS and output an unfiltered image
			else {
				if (previousState) {
					camera.setFPS(30);
					camera.setExposureAuto();
				}
				cvSink.grabFrame(base);
				rectOutputStream.putFrame(base);
			}
			previousState = state;
		}
	}

	/**
	 * Used to determine the displacement of the chosen cube
	 * 
	 * @return The number of pixels the cube is off center
	 */
	public int getError() {
		if (rectChoice >= 0 && rects.size() > rectChoice && rects.get(rectChoice) != null) {
			try {
				return (int) (160 - rects.get(rectChoice).center.x);
			} catch (NullPointerException e) {
				return Integer.MAX_VALUE;
			}
		} else {
			return 0;
		}
	}

	/**
	 * Sets the selector method to be used to select a cube
	 * 
	 * @param method The selection method to be used
	 */

	public void setSelectionMethod(SelectorType method) {
		selectionMethod = method;
	}

}
