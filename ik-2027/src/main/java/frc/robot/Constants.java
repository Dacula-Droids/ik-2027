package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class Constants {
    public static class WristConstants {
        public static final int WristMotorID = 1;
        public static final double WristGearRatio = 12.0;
        public static final Angle wristLowerSoftLimit = Degrees.of(45);
        public static final Angle wristUpperSoftLimit = Degrees.of(135);
        public static final Angle wristStartingPosition = Degrees.of(0);
        public static final Mass wristMass = Units.Pounds.of(8.7342717); // 8.7342717 lbs
        public static final Distance wristCenterOfMassFromPivot = Units.Inches.of(9);
        public static final double wristAngleOffsetDegrees = 45; //The angle the wrist is at when its zeroed out in degrees
    }

    public static class IntakeConstants {
        public static final int IntakeMotorID = 2;
        public static final double IntakeGearRatio = 1.0;
    }

    public static class ShoulderConstants {
        public static final int ShoulderMotorID = 3;
        public static final double ShoulderGearRatio = 12.0;
        public static final Angle shoulderLowerSoftLimit = Degrees.of(0);
        public static final Angle shoulderUpperSoftLimit = Degrees.of(180);
        public static final Angle shoulderStartingPosition = Degrees.of(90);
        public static final Mass shoulderMass = Units.Pounds.of(10); // 10 lbs
        public static final Distance shoulderCenterOfMassFromPivot = Units.Inches.of(5);
    }

    public static class IKConstants{
        public static final double shoulderLength = 10; //Inches
        public static final double wristLength = 7; //Inches
        public static final double maxLength = Constants.IKConstants.shoulderLength + Constants.IKConstants.wristLength;
    }
}
