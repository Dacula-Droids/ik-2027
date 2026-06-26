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
    }
}
