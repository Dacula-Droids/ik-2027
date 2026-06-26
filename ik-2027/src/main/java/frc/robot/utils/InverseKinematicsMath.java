package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class InverseKinematicsMath {

    public static Angle calculateWristAngle(double x, double y){
      double d = Math.hypot(x, y); //uses Pythagorean theorem to calculate distance

      if (d> Constants.IKConstants.maxLength){
        d = d - 0.1; //if the distance is greater than the max length put safety feature
      }
      
      //law of cosines formula
      double numerator = Math.pow(Constants.IKConstants.shoulderLength, 2) + Math.pow(Constants.IKConstants.wristLength, 2) - Math.pow(d, 2);
      double denominator = 2 * Constants.IKConstants.shoulderLength * Constants.IKConstants.wristLength;
      double calculatedInternalWristAngle = Math.toDegrees(Math.acos(numerator / denominator));
      double wristMotorAngleFeed = calculatedInternalWristAngle - Constants.WristConstants.wristAngleOffsetDegrees; //If the wrist is at a 45 degree angle at its zero position and the math says it needs to move to 125 degrees, then the subtraction makes it move 80 degrees from its starting position to match the IK math (125 degrees)
      return Degrees.of(wristMotorAngleFeed);
    }

    

}
