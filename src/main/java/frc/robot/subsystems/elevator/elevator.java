package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
public class elevator extends SubsystemBase {
    private final SparkMax elevatorMotor1;
    private final SparkMax elevatorMotor2;
    private final double elevatorMotor1SpeedUp = 1.0;
    private final double elevatorMotor1SpeedDown = 1.0;
    private final double elevatorMotor2SpeedUp = 1.0;
    private final double elevatorMotor2SpeedDown = 1.0;

    public elevator() {
        elevatorMotor1 = new SparkMax(9, MotorType.kBrushed);
        elevatorMotor2 = new SparkMax(10, MotorType.kBrushed);
    }

    /**
     * Method for elevator control.
     * 
     * @param rtValue Value for Right Trigger (RT).
     * @param ltValue Value for Left Trigger (LT).
     */
    public void runElevator(double rtValue, double ltValue) {
        // Control codes for right motor
        if (rtValue > 0.1) {
            elevatorMotor1.set(-elevatorMotor1SpeedUp * rtValue);
        } else if (ltValue > 0.1) {
            elevatorMotor1.set(elevatorMotor1SpeedDown * ltValue);
        } else {
            elevatorMotor1.set(0);
        }

        // Control codes for left motor
        if (rtValue > 0.1) {
            elevatorMotor2.set(-elevatorMotor2SpeedUp * rtValue);
        } else if (ltValue > 0.1) {
            elevatorMotor2.set(elevatorMotor2SpeedDown * ltValue);
        } else {
            elevatorMotor2.set(0);
        }
    }

    @Override
    public void periodic() {
    }
}