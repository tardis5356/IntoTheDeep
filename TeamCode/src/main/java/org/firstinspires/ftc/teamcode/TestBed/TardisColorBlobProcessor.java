package org.firstinspires.ftc.teamcode.TestBed;

import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public abstract class TardisColorBlobProcessor extends ColorBlobLocatorProcessor {


    public static abstract class Blob
    {
        /**
         * Get the OpenCV contour for this blob
         * @return OpenCV contour
         */
        public abstract MatOfPoint getContour();

        /**
         * Get the contour points for this blob
         * @return contour points for this blob
         */
        public abstract Point[] getContourPoints();

        /**
         * Get the area enclosed by this blob's contour
         * @return area enclosed by this blob's contour
         */
        public abstract int getContourArea();

        /**
         * Get the density of this blob, i.e. ratio of
         * contour area to convex hull area
         * @return density of this blob
         */
        public abstract double getDensity();

        /**
         * Get the aspect ratio of this blob, i.e. the ratio
         * of longer side of the bounding box to the shorter side
         * @return aspect ratio of this blob
         */
        public abstract double getAspectRatio();

        /**
         * Get a "best fit" bounding box for this blob
         * @return "best fit" bounding box for this blob
         */
        public abstract RotatedRect getBoxFit();

        public double getScore(double targetAspect, double targetDensity, double targetArea){
            double score = Math.abs((getContourArea()-targetArea)/targetArea) + Math.abs((getAspectRatio()-targetAspect)/targetAspect) + Math.abs((getDensity()-targetDensity)/targetDensity);

            return score;
        }
    }


    //@Deprecated
    public static class Util
    {

        /**
         * Sort a list of Blobs based on area
         * @param sortOrder sort order
         * @param blobs List of Blobs to operate on
         */
        public static void sortByArea(SortOrder sortOrder, List<Blob> blobs)
        {
            blobs.sort(new Comparator<Blob>()
            {
                public int compare(Blob c1, Blob c2)
                {
                    int tmp = (int)Math.signum(c2.getContourArea() - c1.getContourArea());

                    if (sortOrder == SortOrder.ASCENDING)
                    {
                        tmp = -tmp;
                    }

                    return tmp;
                }
            });
        }



        /**
         * Sort a list of Blobs based on density
         * @param sortOrder sort order
         * @param blobs List of Blobs to operate on
         */
        public static void sortByDensity(SortOrder sortOrder, List<Blob> blobs)
        {
            blobs.sort(new Comparator<Blob>()
            {
                public int compare(Blob c1, Blob c2)
                {
                    int tmp = (int)Math.signum(c2.getDensity() - c1.getDensity());

                    if (sortOrder == SortOrder.ASCENDING)
                    {
                        tmp = -tmp;
                    }

                    return tmp;
                }
            });
        }

        public static void sortByScore(SortOrder sortOrder, List<Blob> blobs, double tDense, double tArea, double tAspect){
            blobs.sort(new Comparator<Blob>() {
                //@Override
                public int compare(Blob c1, Blob c2) {
                    int tmp = (int)Math.signum(c2.getScore(tAspect,tDense,tArea)-c1.getScore(tAspect,tDense,tArea));
                    if (sortOrder == SortOrder.ASCENDING)
                    {
                        tmp = -tmp;
                    }

                    return tmp;
                }
            });
        }

        /**
         * Sort a list of Blobs based on aspect ratio
         * @param sortOrder sort order
         * @param blobs List of Blobs to operate on
         */
        public static void sortByAspectRatio(SortOrder sortOrder, List<Blob> blobs)
        {
            blobs.sort(new Comparator<Blob>()
            {
                public int compare(Blob c1, Blob c2)
                {
                    int tmp = (int)Math.signum(c2.getAspectRatio() - c1.getAspectRatio());

                    if (sortOrder == SortOrder.ASCENDING)
                    {
                        tmp = -tmp;
                    }

                    return tmp;
                }
            });
        }
    }

}
