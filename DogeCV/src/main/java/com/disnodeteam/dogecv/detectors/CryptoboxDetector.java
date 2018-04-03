package com.disnodeteam.dogecv.detectors;

import android.util.Log;

import com.disnodeteam.dogecv.OpenCVPipeline;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.math.Line;
import com.disnodeteam.dogecv.math.Lines;
import com.disnodeteam.dogecv.math.MathFTC;
import com.disnodeteam.dogecv.math.Points;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class CryptoboxDetector extends OpenCVPipeline {

	public enum CryptoboxDetectionMode {
		RED, BLUE
	}

	public enum CryptoboxSpeed {
		VERY_FAST, FAST, BALANCED, SLOW, VERY_SLOW
	}

	public CryptoboxDetectionMode detectionMode = CryptoboxDetectionMode.RED;
	public double downScaleFactor = 0.5;
	public boolean rotateMat = false;
	public CryptoboxSpeed speed = CryptoboxSpeed.BALANCED;

	public DogeCVColorFilter colorFilterRed = new LeviColorFilter(LeviColorFilter.ColorPreset.RED);
	public DogeCVColorFilter colorFilterBlue = new LeviColorFilter(LeviColorFilter.ColorPreset.BLUE);

	private Mat workingMat = new Mat();
	private Mat mask = new Mat();
	private Size newSize = new Size();
	
	private StringBuffer output = "";

	public int getWidth() {
		return mask.width();
	}

	@Override
	public Mat processFrame(Mat rgba, Mat gray) {
		downScaleFactor = 0.5;
		Size initSize = rgba.size();
		newSize = new Size(initSize.width * downScaleFactor, initSize.height * downScaleFactor);
		rgba.copyTo(workingMat);

		Imgproc.resize(workingMat, workingMat, newSize);
		if (rotateMat) {
			Mat tempBefore = workingMat.t();
			Core.flip(tempBefore, workingMat, 1); //mRgba.t() is the transpose
			tempBefore.release();
		}

		switch (detectionMode) {
			case RED:
				Mat redMask = workingMat.clone();
				colorFilterRed.process(redMask, mask);
				redMask.release();
				break;
			case BLUE:
				Mat blueMask = workingMat.clone();
				colorFilterBlue.process(blueMask, mask);
				blueMask.release();
				break;
		}

		// Find a list of vertical line segments
		List<Line> lines = Lines.getOpenCvLines(mask, 1, 55);
		lines = Lines.linearExtend(lines, 4, newSize);

		List<Line> linesVertical = new ArrayList<>();
		for (Line line : lines) {
			if (Lines.getAngularDistance(line, new Line(new Point(0, 0), new Point(100, 0))) > 45) {
				linesVertical.add(line);
			}
		}

		List<Line> rails = new ArrayList<>();

		// If there are none, we haven't found anything
		if (linesVertical.size() == 0) {
			// TODO: found nothing
		} else {
			// Sort lines based on the x positions of their centers
			Collections.sort(linesVertical, new Comparator<Line>() {
				@Override
				public int compare(Line line1, Line line2) {
					return Double.compare(line1.center().x, line2.center().x);
				}
			});

			// Get the leftmost and rightmost lines
			Line left = linesVertical.get(0);
			Line right = linesVertical.get(linesVertical.size() - 1);

			// The distance between separate lines that form the same rail should never be more than half the width of a column (i.e. 1/6 of the full cryptobox width, if it's entirely visible)
			double columnLength = Lines.getPerpendicularDistance(left, right) / 6;

			// Group lines that form the same rail
			List<List<Line>> groupings = new ArrayList<>();

			// Start at the first (leftmost) line
			for (int j = 0; j < linesVertical.size(); ) {
				List<Line> group = new ArrayList<>();
				group.add(linesVertical.get(j));

				// Iterate through subsequent lines until the end of the list is reached or the line is too far away to be in the group
				int i = j + 1;
				while (i < linesVertical.size() && Lines.getPerpendicularDistance(linesVertical.get(j), linesVertical.get(i)) < columnLength) {
					group.add(linesVertical.get(i));
					i++;
				}
				groupings.add(group);

				// Start the next iteration from the end of the previous group
				j = i;
			}

			// Detect opposite sides of cryptobox rails as one line
			for (int i = 0; i < groupings.size() - 1; i++) {
				// Get the center point between the center of this group and the next one
				Point center = new Line(Lines.getMeanPoint(groupings.get(i)), Lines.getMeanPoint(groupings.get(i + 1))).center();

				int y = (int) MathFTC.clip(0.6 * center.y, 0, mask.height());
				double max = 1.4 * center.y;
				if (center.y < 125) {
					y = 1;
					max = 250;
				}

				// Test different points along the vertical line between the two groups to see if they're the right color
				int count = 0;
				while (y < mask.height() && y < max && count < 10) {
					if (mask.get(y, (int) center.x)[0] > 0) {
						count++;
					}
					y += 10;
				}

				// If at least 10 points were found of the right color
				if (count >= 10) {
					// Combine the two groups
					groupings.get(i).addAll(groupings.get(i + 1));
					groupings.remove(i-- + 1);
				}
			}

			// Remove groups that aren't rails
			for (int i = 0; i < groupings.size(); i++) {
				Point center = Lines.getMeanPoint(groupings.get(i));
				int y = (int) MathFTC.clip(0.2 * center.y, 0, mask.height());
				double max = 1.8 * center.y;
				if (center.y < 50) {
					y = 1;
					max = (int) (0.8 * mask.height()); // This was missing the parentheses around the multiplication before, and always evaluated to 0. Not sure if it works better like this
				}
				int minX = (int) MathFTC.clip(center.x - 5, 0, mask.width() - 1);
				int maxX = (int) MathFTC.clip(center.x + 5, 0, mask.width() - 1);

				// Count the number of correctly colored pixels in the input image
				int count = 0;
				while (y < mask.height() && y < max && count < 10) {
					if (mask.get(y, (int) center.x)[0] > 0 || mask.get(y, minX)[0] > 0 || mask.get(y, maxX)[0] > 0) {
						count++;
					}
					y += 4;
				}

				// If there weren't enough found, remove this group
				if (count <= 9) {
					groupings.remove(i--);
				}
			}

			// If there are still more groups that there should be
			if (groupings.size() > 4) {
				// Sort by standard deviation of the x-components of the component lines (to remove the worst later)
				Collections.sort(groupings, new Comparator<List<Line>>() {
					@Override
					public int compare(List<Line> g1, List<Line> g2) {
						return Double.compare(Lines.stdDevX(g2), Lines.stdDevX(g1));
					}
				});
			}

			for (List<Line> grouping : groupings) {
				rails.add(Lines.constructLine(Lines.getMeanPoint(grouping), Lines.getMeanAngle(grouping), 400));
			}
		}

		for (Line line : rails) {
			output.append(String.format("[%.4f, %.4f, %.4f, %.4f]", line.x1, line.y1, line.x2, line.y2));
		}
		output.append('\n');

		for (int i = 0; i < groupings.size(); i++) {
			groupings.set(i, Lines.resize(groupings.get(i), 1 / downScaleFactor));
		}

		// Draw the detected rails to the output display
		for (Line line : rails) {
			line.resize(1 / downScaleFactor);
			Imgproc.line(rgba, line.point1, line.point2, new Scalar(20, 165, 240), 20);
		}

		for (int i = 0; i < rails.size() - 1; i++) {
			// Find the segment of the perpendicular bisector of the first rail between the first and second rails
			Line connection = Lines.getPerpendicularConnector(rails.get(i), rails.get(i + 1), rgba.size());

			// Find the point between the two rails
			Point centerPoint = connection.center();

			// Label the point in the display
			Imgproc.putText(rgba, "Col #" + i, new Point(centerPoint.x, centerPoint.y - 15), 0, 1.5, new Scalar(0, 255, 255), 2);
			Imgproc.circle(rgba, centerPoint, 15, new Scalar(0, 255, 0), 6);
		}

		// Display potentially useful information
		Imgproc.putText(rgba, "DogeCV 1.1 Crypto: " + newSize.toString() + " - " + speed.toString() + " - " + detectionMode.toString(), new Point(5, 30), 0, 1.2, new Scalar(0, 255, 255), 2);

		return rgba;

	}
	
	public String getOutput() {
		return output.toString();
	}

	public Point drawSlot(int slot, List<Rect> boxes) {
		Rect leftColumn = boxes.get(slot); //Get the pillar to the left
		Rect rightColumn = boxes.get(slot + 1); //Get the pillar to the right

		int leftX = leftColumn.x; //Get the X Coord
		int rightX = rightColumn.x; //Get the X Coord

		int drawX = ((rightX - leftX) / 2) + leftX; //Calculate the point between the two
		int drawY = leftColumn.height + leftColumn.y; //Calculate Y Coord. We wont use this in our bot's opetation, buts its nice for drawing

		return new Point(drawX, drawY);
	}

	public static double[] getPosition(Line left, Line right, Size size, double f, double x0, double y0) {
		Line connector = Lines.getPerpendicularConnector(left, right, size);
		double u1 = connector.x1 - x0;
		double u2 = connector.x2 - x0;
		double v1 = y0 - connector.y1;
		double v2 = y0 - connector.y2;

		double y1 = 7.63 * f * Math.sqrt(1 / ((Math.pow(u1, 2) + 1) - (2 * v1 * (u1 * u2 + 1) / v2) + (Math.pow(v1 / v2, 2) * (Math.pow(u2, 2) + 1))));
		double y2 = 7.63 * f * Math.sqrt(1 / ((Math.pow(u2, 2) + 1) - (2 * v2 * (u1 * u2 + 1) / v1) + (Math.pow(v2 / v1, 2) * (Math.pow(u1, 2) + 1))));

		double x1 = y1 * u1 / f;
		double x2 = y2 * u2 / f;

		return new double[]{x1, y1, x2, y2};
	}

	public Size getFrameSize() {
		return newSize;
	}

}
