// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.multisubsystem.MoveArm;
import frc.robot.subsystems.ElevatorSubsystem;

/** Add your docs here. */

public class ArmUtils {
    private GAME_PIECE piece;
    private PIVOT_DIRECTION side;
    private ARM_STATE state;
    private RobotContainer robotContainer;
    public ElevatorSubsystem elevator;
    public ArmUtils(RobotContainer container, GAME_PIECE defaultPiece, PIVOT_DIRECTION defaultSide, ARM_STATE defaultState) {
        piece = defaultPiece;
        side = defaultSide;
        state = defaultState;
        robotContainer = container;
        elevator = container.telescopingArm;
    }
    public enum GAME_PIECE{
        CONE,
        CUBE
    }

    public enum PIVOT_DIRECTION {
        FORWARD,
        REVERSE
    }

    public enum ARM_STATE {
        LOW,
        MID,
        HIGH,
        STATION_SINGLE,
        STATION_DOUBLE,
        STOW
    }

    public void setState(ARM_STATE state) {
        this.state = state;
    }

    public void setPivotDirection(PIVOT_DIRECTION side) {
        this.side = side;
    }

    public void setGamePiece(GAME_PIECE piece) {
        this.piece = piece;
    }

    public ARM_STATE getArmState() {
        return state;
    }

    public PIVOT_DIRECTION getPivotDirection() {
        return side;
    }

    public GAME_PIECE getGamePiece() {
        return piece;
    }

    public int wristSetpoint() {
        int setpoint = 0;
        if (piece == GAME_PIECE.CONE) {
            switch(state) {
                case LOW: setpoint = WristConstants.LOW_WRIST_TICKS; break;
                case MID: setpoint = WristConstants.CONE_MID_GOAL_WRIST_TICKS; break;
                case HIGH: setpoint = WristConstants.CONE_HIGH_GOAL_WRIST_TICKS; break;
                case STATION_SINGLE: setpoint = WristConstants.SINGLE_PICKUP_WRIST_TICKS; break;
                case STATION_DOUBLE: setpoint = WristConstants.CONE_DOUBLE_PICKUP_WRIST_TICKS; break;
                case STOW: setpoint = 0; break;
            }
        } else if (piece == GAME_PIECE.CUBE) {
            switch(state) {
                case LOW: setpoint = WristConstants.LOW_WRIST_TICKS; break;
                case MID: setpoint = WristConstants.CUBE_MID_GOAL_WRIST_TICKS; break;
                case HIGH: setpoint = WristConstants.CUBE_HIGH_GOAL_WRIST_TICKS; break;
                case STATION_SINGLE: setpoint = WristConstants.SINGLE_PICKUP_WRIST_TICKS; break;
                case STATION_DOUBLE: setpoint = WristConstants.CUBE_DOUBLE_PICKUP_WRIST_TICKS; break;
                case STOW: setpoint = 0; break;
            }
        }
        if(side == PIVOT_DIRECTION.REVERSE) setpoint *= -1;
        return setpoint;
    }

    public int pivotSetpoint() {
        int setpoint = 0;
        if (piece == GAME_PIECE.CONE) {
            switch(state) {
                case LOW: setpoint = PivotConstants.LOW_PIVOT_TICKS; break;
                case MID: setpoint = PivotConstants.CONE_MID_GOAL_PIVOT_TICKS; break;
                case HIGH: setpoint = PivotConstants.CONE_HIGH_GOAL_PIVOT_TICKS; break;
                case STATION_SINGLE: setpoint = PivotConstants.SINGLE_PICKUP_PIVOT_TICKS; break;
                case STATION_DOUBLE: setpoint = PivotConstants.CONE_DOUBLE_PICKUP_PIVOT_TICKS; break;
                case STOW: setpoint = 0; break;
            }
        } else if (piece == GAME_PIECE.CUBE) {
            switch(state) {
                case LOW: setpoint = PivotConstants.LOW_PIVOT_TICKS; break;
                case MID: setpoint = PivotConstants.CUBE_MID_GOAL_PIVOT_TICKS; break;
                case HIGH: setpoint = PivotConstants.CUBE_HIGH_GOAL_PIVOT_TICKS; break;
                case STATION_SINGLE: setpoint = PivotConstants.SINGLE_PICKUP_PIVOT_TICKS; break;
                case STATION_DOUBLE: setpoint = PivotConstants.CUBE_DOUBLE_PICKUP_PIVOT_TICKS; break;
                case STOW: setpoint = 0; break;
            }
        }
        if(side == PIVOT_DIRECTION.REVERSE) setpoint *= -1;
        return setpoint;
    }

    public int armSetpoint() {
        int setpoint = 0;
        if (piece == GAME_PIECE.CONE) {
            switch(state) {
                case LOW: setpoint = ArmConstants.LOW_EXT_TICKS; break;
                case MID: setpoint = ArmConstants.CONE_MID_GOAL_EXT_TICKS; break;
                case HIGH: setpoint = ArmConstants.CONE_HIGH_GOAL_EXT_TICKS; break;
                case STATION_SINGLE: setpoint = ArmConstants.SINGLE_PICKUP_EXT_TICKS; break;
                case STATION_DOUBLE: setpoint = ArmConstants.CONE_DOUBLE_PICKUP_EXT_TICKS; break;
                case STOW: setpoint = 0; break;
            }
        } else if (piece == GAME_PIECE.CUBE) {
            switch(state) {
                case LOW: setpoint = ArmConstants.LOW_EXT_TICKS; break;
                case MID: setpoint = ArmConstants.CUBE_MID_GOAL_EXT_TICKS; break;
                case HIGH: setpoint = ArmConstants.CUBE_HIGH_GOAL_EXT_TICKS; break;
                case STATION_SINGLE: setpoint = ArmConstants.SINGLE_PICKUP_EXT_TICKS; break;
                case STATION_DOUBLE: setpoint = ArmConstants.CUBE_DOUBLE_PICKUP_EXT_TICKS; break;
                case STOW: setpoint = 0; break;
            }
        }
        return setpoint;
    }

    public Command getMotionCommand() {
        return new MoveArm(robotContainer, this);
    }

    public SequentialCommandGroup getMotionCommand(ARM_STATE armState) { //suited best for arm position buttons
        return new SequentialCommandGroup(new InstantCommand(()-> setState(armState)), getMotionCommand());
    }

    public SequentialCommandGroup getMotionCommand(ARM_STATE armState, GAME_PIECE piece, PIVOT_DIRECTION side) { //suited best for autonomous commands
        return new SequentialCommandGroup(new InstantCommand(()-> {
                setState(armState);
                setGamePiece(piece);
                setPivotDirection(side);
            }), 
            getMotionCommand());
    }   
}
