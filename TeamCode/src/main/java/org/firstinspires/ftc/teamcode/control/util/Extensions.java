package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Extensions
{
    public static double HEADING = 0;

    public static double cubeInput(double input, double factor)
    {
        double t = factor * Math.pow(input, 3);
        double r = input * (1 - factor);

        return t + r;
    }

    public static Pose2d toFieldRelative(Pose2d original, double angleInRadians)
    {
        Vector2d vec = original.vec().rotated(-angleInRadians);
        return new Pose2d(vec, original.getHeading());
    }
}
