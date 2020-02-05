/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class RobotMap {

    public static CANSparkMax rghtFront = new CANSparkMax(1, MotorType.kBrushless);
	public static CANSparkMax rghtFollower = new CANSparkMax(2, MotorType.kBrushless);
	public static CANSparkMax leftFront = new CANSparkMax(3, MotorType.kBrushless);
    public static CANSparkMax leftFollower = new CANSparkMax(4, MotorType.kBrushless);
    
    public static DifferentialDrive m_drive = new DifferentialDrive(leftFront, rghtFront);

    public static Double objHeightcm = 7.85;
    public static Double objWidthcm = 7.85;
    public static Double alpha = 1.387;
    public static Double beta = 0.728;
    public static Double ratio = 1.0;
    public static double k_p = 0.38;
    public static double k_i = 0.000001;

    // use ratios

    public static Double calculateDist (Double h, Double w, Double x, Double y){

        //determine whether the object would be out of the frame
        if (ratio >1.1){
            //width went out
            if (x> (315/2)){
                w = (w-(315.0-x))*2;
            }
            else if (x < (315/2)){
                w = (w-x)*2;
            }
        }
        else if(ratio < 0.9){
            //height went out
            if (y> (207/2)){
                h = (h-(207-y))*2;
            }
            else if (x < (315/2)){
                h = (h-y)*2;
            }
        }
        double ans = (((objWidthcm*315)/(alpha*w))+((objHeightcm*207)/(beta*h)))/2;
        return ans;
    }

    public static double[] calculateAngle (double dist, double x, double y){
        double[] ans ={0,0};
        ans[0] = Math.toDegrees(Math.atan(((x-157.5)*Math.tan(3.14/6))/157.5));
        ans[1] = Math.toDegrees(Math.atan(((103.5-y)*Math.tan(3.14/9))/103.5));
        return ans;
    }
}
