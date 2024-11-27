package org.firstinspires.ftc.teamcode.usefulStuff;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class QuikMaths {
    public static final double PI = 3.14159265358979323846;
    public static double RadToDeg = 180/PI;
    public static double DegToRad = 0.01745329;
    public static double FtToM = 0.3048;
    public static double MToFt = 3.2808399;
    public static double MiToFt = 5280;
    public static double FtToMi = 1/5280;
    public static double InToFt = 1/12;
    public static double FtToIn = 12;

    public static float Clamp(float min, float max, float n){
        if (n>max){
            return max;
        }
        if (n<min){
            return min;
        }
        return n;
    }

    public static int LargestOfThree(double zero, double first, double second){
        if(zero>first&&zero>second){
            return 0;
        }else if(first>zero&&first>second){
            return 1;
        }else {
            return 2;
        }
    }

    public static double csc(double a){
        return 1/Math.sin(a);
    }
    public static double sec(double a){
        return 1/Math.cos(a);
    }
    public static double cot(double a){
        return 1/Math.tan(a);
    }
    public static double lerp(double v0, double v1, double t) {
        return (1 - t) * v0 + t * v1;
    }

    public static Vector2d a2s(double A){

        if(A<90 && A>0){
            double C = 90;
            double B = 180- (A+C);
            double c = Math.min(Math.abs(QuikMaths.csc(A%90)),Math.abs(QuikMaths.sec(A%90)));


            Vector2d ab = new Vector2d(c*Math.sin(A),c*Math.sin(B));
            return ab;
        }else if (A>90){
            double C = 90;
            double B = 180- (A+C);
            double c = Math.min(Math.abs(QuikMaths.csc(A%90)),Math.abs(QuikMaths.sec(A%90)));


            Vector2d ab = new Vector2d(c*Math.sin(A),-c*Math.sin(B));
            return ab;
        }else if (A<0 && A>-90){
            double C = 90;
            double B = 180- (A+C);
            double c = Math.min(Math.abs(QuikMaths.csc(A%90)),Math.abs(QuikMaths.sec(A%90)));


            Vector2d ab = new Vector2d(-c*Math.sin(A),c*Math.sin(B));
            return ab;
        }else if(A<-90){
            double C = 90;
            double B = 180- (A+C);
            double c = Math.min(Math.abs(QuikMaths.csc(A%90)),Math.abs(QuikMaths.sec(A%90)));


            Vector2d ab = new Vector2d(-c*Math.sin(A),-c*Math.sin(B));
            return ab;
        }

        return null;
    }

    public static double Atan2(float x, float y){
        return Math.atan2(y,x);
    }
    public static double Atan2(double x, double y){
        return Math.atan2(y,x);
    }
    public static double Atan2(@NonNull Vector2d vector2d){
        return Math.atan2(vector2d.y, vector2d.x);
    }
    public static double Atan2(@NonNull Pose2d pose2d){
        return Math.atan2(pose2d.position.y, pose2d.position.x);
    }

    public static double GetAngleFromVector(float x, float y){
        return Atan2(-y, x) * RadToDeg;
    }
    public static double GetAngleFromVector(double x, double y){
        return Atan2(-y, x) * RadToDeg + 90;
    }
    public static double GetAngleFromVector(Vector2d vector2d){
        return Atan2(-vector2d.y, vector2d.x) * RadToDeg;
    }
    public static double GetAngleFromVector(Pose2d pose2d){
        return Atan2(-pose2d.position.y, pose2d.position.x) * RadToDeg;
    }

    public static void QuikMeths(){
        QuikMeths();
    }

}