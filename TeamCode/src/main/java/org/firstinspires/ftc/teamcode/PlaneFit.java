package org.firstinspires.ftc.teamcode;

import java.util.List;

public class PlaneFit {
    private double ticksA;
    private double ticksB;
    private double ticksC;

    private double shiftA;
    private double shiftB;
    private double shiftC;

    public PlaneFit( List<CalibrationPoint> points){
        double x2 = 0;
        double xy = 0;
        double xt = 0;
        double xl = 0;
        double y2 = 0;
        double yt = 0;
        double yl = 0;
        double x = 0;
        double y = 0;
        double t = 0;
        double l = 0;

        for(CalibrationPoint pt: points){
            x2 += pt.x * pt.x;
            xy += pt.x * pt.y;
            xt += pt.x * pt.ticks;
            xl += pt.x * pt.legs;
            y2 += pt.y * pt.y;
            yt += pt.y * pt.ticks;
            yl += pt.y * pt.legs;
            x += pt.x;
            y += pt.y;
            t += pt.ticks;
            l += pt.legs;
        }
        double N = (double)points.size();
        double den = (-2.0 * y * x * xy + x2 * y * y + x * x * y2 + N * xy * xy - x2 * N * y2);
        if(den == 0)
        {
            ticksA = ticksB = ticksC = shiftA = shiftB = shiftC = 0;
        }
        else{
            ticksA = -(y * x * yt - y * y * xt + y * t * xy - N * xy * yt + N * y2 * xt - t * x * y2) / den;
            ticksB =  (x * x * yt - x2 * N * yt + N * xy * xt - y * x * xt + y * x2 * t - t * x * xy) / den;
            ticksC = -(x * xy * yt - y2 * x * xt - yt * x2 * y - xy * xy * t + y2 * x2 * t + y * xy * xt) / den;

            shiftA = -(y * x * yl - y * y * xl + y * l * xy - N * xy * yl + N * y2 * xl - l * x * y2) / den;
            shiftB =  (x * x * yl - x2 * N * yl + N * xy * xl - y * x * xl + y * x2 * l - l * x * xy) / den;
            shiftC = -(x * xy * yl - y2 * x * xl - yl * x2 * y - xy * xy * l + y2 * x2 * l + y * xy * xl) / den;
        }
    }

    public int getTicks(int x, int y){
        return (int)Math.round(ticksA * (double)x + ticksB * (double)y + ticksC);
    }

    public int getShift(int x, int y){
        return (int)Math.round(shiftA * (double)x + shiftB * (double)y + shiftC);
    }
}
