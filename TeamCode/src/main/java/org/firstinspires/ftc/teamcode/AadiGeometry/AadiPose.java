package org.firstinspires.ftc.teamcode.AadiGeometry;

import org.firstinspires.ftc.teamcode.SubSystems.SystemState;

public class AadiPose extends AadiVector{
    private double turretAngle;

    public AadiPose(double armLength, double shoulderAngle, AadiVector.WRIST_ANGLE wristAngle, double turretAngle){
        super(armLength,shoulderAngle, wristAngle );
        setTurretAngle(turretAngle);
    }

    public void setTurretAngle(double turretAngle) {
        this.turretAngle = turretAngle;
    }

    public AadiVector getAadiVector(AadiPose aadiPose) {
        return new AadiVector(aadiPose.getArmLength(aadiPose),aadiPose.getShoulderAngle(aadiPose), aadiPose.getWristAngle(aadiPose) );
    }

    public double getTurretAngle(AadiPose aadiPose) {
        return turretAngle;
    }

    public double turretAngleToRadians(double turretAngle){
        return turretAngle / SystemState.TURRET_ANGLE_TO_RADIANS_RATIO;
    }

    public static double turretRadiansToAngle(double turretRadians){
        return turretRadians * SystemState.TURRET_ANGLE_TO_RADIANS_RATIO;
    }

}
