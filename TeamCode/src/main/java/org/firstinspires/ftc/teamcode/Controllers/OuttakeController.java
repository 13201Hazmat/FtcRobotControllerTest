package org.firstinspires.ftc.teamcode.Controllers;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeSlides;

public class OuttakeController {
    public OuttakeSlides outtakeSlides;
    public OuttakeArm outtakeArm;
    LinearOpMode currentOpMode;


    public OuttakeController(OuttakeSlides outtakeSlides, OuttakeArm outtakeArm, LinearOpMode currentOpMode){
        this.outtakeSlides = outtakeSlides;
        this.outtakeArm = outtakeArm;
        this.currentOpMode = currentOpMode;
    }

    public void moveTransferToPickup(){
        outtakeArm.closeGrip();
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.PICKUP);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.PICKUP);
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.PICKUP);
    }

    public Action moveTransferToPickupAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveTransferToPickup();
                return true;
            }
        };
    }

    public void movePickupToTransfer(){
        outtakeArm.closeGrip();
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.TRANSFER);
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER);
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER);
    }

    public Action movePickupToTransferAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                movePickupToTransfer();
                return true;
            }
        };
    }


    public void moveTransferToReadyForTransfer(){
        outtakeArm.closeGrip();
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER);
        safeWaitMilliSeconds(100);
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
    }

    public Action moveTransferToReadyForTransferAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveTransferToReadyForTransfer();
                return true;
            }
        };
    }

    public void moveReadyForTransferToTransfer(){
        outtakeArm.openGrip();
        ElapsedTime timeoutTimer = new ElapsedTime(MILLISECONDS);
        if (outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER) &&
                outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER)) {
            timeoutTimer.reset();
            while (!outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER) &&
                    timeoutTimer.time() < 2000) {
                outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER);
            }
            outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER);
            outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.TRANSFER);
        } else {
            outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER);
            outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER);
            outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
        }
    }

    public Action moveReadyForTransferToTransferAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveReadyForTransferToTransfer();
                return true;
            }
        };
    }

    public void moveTravelToReadyForTransfer(){
        outtakeArm.closeGrip();
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER);
        safeWaitMilliSeconds(200);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
        safeWaitMilliSeconds(100);
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER);
    }

    public Action moveTravelToReadyForTransferAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveTravelToReadyForTransfer();
                return true;
            }
        };
    }

    public void moveReadyForTransferToDropLevel(OuttakeSlides.OUTTAKE_SLIDE_STATE outtakeSlideStateDropLevel){
        outtakeArm.closeGrip();
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.DROP);
        if(outtakeSlideStateDropLevel == OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOW_LINE ||
            outtakeSlideStateDropLevel == OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST) {
            safeWaitMilliSeconds(100);
        }
        outtakeSlides.moveOuttakeSlides(outtakeSlideStateDropLevel);
    }

    public Action moveReadyForTransferToDropLevelMidAction(){
        return new Action(){

            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveReadyForTransferToDropLevel(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LEVEL_MID);
                return true;
            }
        };
    }


    public void moveReadyForTransferToTravel(){
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRAVEL);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.TRAVEL);
        safeWaitMilliSeconds(100);
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRAVEL);
    }

    public Action moveReadyForTransferToTravelAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveReadyForTransferToTravel();
                return true;
            }
        };
    }

    public void moveDropToTravel(){
        outtakeArm.closeGrip();
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRAVEL);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.TRAVEL);
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRAVEL);
    }

    public Action moveDropToTravelAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveDropToTravel();
                return true;
            }
        };
    }

    public void safeWaitMilliSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!currentOpMode.isStopRequested() && timer.time() < time) {
        }
    }

}
