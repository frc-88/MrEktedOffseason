package frc.robot.Utils;

import java.lang.Math;

public class DriveUtils{
    public static double signedPow(double base, int exp){
        double value = 0;
        
        if(base < 0 && exp%2==0){
            value = -Math.pow(base,exp);
        }
        else{
            value = Math.pow(base,exp);
        }

        return value;
    }


}