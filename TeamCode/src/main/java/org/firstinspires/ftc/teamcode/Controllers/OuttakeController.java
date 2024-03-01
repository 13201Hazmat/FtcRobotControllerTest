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
        safeWaitMilliSeconds(100);
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.PICKUP);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.PICKUP);
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.PICKUP);
        safeWaitMilliSeconds(150);
    }

    public Action moveTransferToPickupAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveTransferToPickup();
                return false;
            }
        };
    }

    public void movePickupToTransfer(){
        outtakeArm.closeGrip();
        safeWaitMilliSeconds(50);
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
                return false;
            }
        };
    }


    public void moveTransferToReadyForTransfer(){
        outtakeArm.closeGrip();
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER);
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
        safeWaitMilliSeconds(200);//200
    }

    public Action moveTransferToReadyForTransferAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveTransferToReadyForTransfer();
                return false;
            }
        };
    }

    public void movePickupToReadyForTransfer(){
        outtakeArm.closeGrip();
        safeWaitMilliSeconds(50);
        movePickupToTransfer();
        safeWaitMilliSeconds(50);
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER);
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
        safeWaitMilliSeconds(200);
    }

    public Action movePickupToReadyForTransferAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                movePickupToReadyForTransfer();
                return false;
            }
        };
    }

    public void moveReadyForTransferToTransfer(){
        outtakeArm.openGrip();
        //safeWaitMilliSeconds(150);
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER);
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.TRANSFER);
        safeWaitMilliSeconds(150);//400
    }

    public Action moveReadyForTransferToTransferAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveReadyForTransferToTransfer();
                return false;
            }
        };
    }

    public void moveReadyForTransferToDropLevel(OuttakeSlides.OUTTAKE_SLIDE_STATE outtakeSlideStateDropLevel){
        outtakeArm.closeGrip();
        outtakeSlides.moveOuttakeSlides(outtakeSlideStateDropLevel);
        safeWaitMilliSeconds(100);

        switch (outtakeSlideStateDropLevel) {
            case DROP_LOWEST_AUTO:
                outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.DROP_LOWEST_AUTO);
                safeWaitMilliSeconds(100);
                outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP_LOWEST_AUTO);
                safeWaitMilliSeconds(100);
            case DROP_LOWEST:
                outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.DROP_LOWEST);
                safeWaitMilliSeconds(100);
                outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP_LOWEST);
                safeWaitMilliSeconds(100);
                break;
            case DROP_LOW_LINE:
                outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.DROP_LOW_LINE);
                safeWaitMilliSeconds(100);
                outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP_LOW_LINE);
                safeWaitMilliSeconds(100);
                break;
            case DROP_BELOW_MID:
                outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.DROP_BELOW_MID);
                safeWaitMilliSeconds(100);
                outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP_BELOW_MID);
                break;
            case DROP_LEVEL_MID:
                outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.DROP_LEVEL_MID);
                safeWaitMilliSeconds(100);
                outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP_LEVEL_MID);
                break;
            case DROP_BELOW_HIGH:
                outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.DROP_BELOW_HIGH);
                safeWaitMilliSeconds(100);
                outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP_BELOW_HIGH);
                break;
            case DROP_LEVEL_HIGH:
                outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.DROP_LEVEL_HIGH);
                safeWaitMilliSeconds(100);
                outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP_LEVEL_HIGH);
                break;
            case DROP_HIGHEST:
                outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.DROP_HIGHEST);
                safeWaitMilliSeconds(100);
                outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP_HIGHEST);
                break;
        }
    }

    public Action moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE outtakeSlideState){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveReadyForTransferToDropLevel(outtakeSlideState);
                return false;
            }
        };
    }

    public void moveDropToReadyforTransfer(){
        outtakeArm.closeGrip();
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
        safeWaitMilliSeconds(100);
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER);
    }

    public Action moveDropToReadyforTransferAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveDropToReadyforTransfer();
                return false;
            }
        };
    }

    public Action dropOnePixelAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                outtakeArm.autoDropYellowPixel();
                return false;
            }
        };
    }

    public Action dropTwoPixelAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                outtakeArm.autoDropTwoPixel();
                return false;
            }
        };
    }

    public void moveOuttakeToEndState(){
        if (outtakeSlides.outtakeSlidesState != OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER &&
            outtakeSlides.outtakeSlidesState != OuttakeSlides.OUTTAKE_SLIDE_STATE.PICKUP) {
            moveDropToReadyforTransfer();
            safeWaitMilliSeconds(800);
        }
    }

    public Action moveOuttakeToEndStateAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveOuttakeToEndState();
                return false;
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
