MODULE TRobLModule
    LOCAL CONST jointtarget home := [[0, -30, 0, 0, 0, 0], [0, 9E9, 9E9, 9E9, 9E9, 9E9]];
    LOCAL VAR egmident egm_id;
    LOCAL VAR egm_minmax egm_condition := [-0.1, 0.1];

    PROC main()
        WHILE TRUE DO
            MoveAbsJ home, v200, fine, tool0;

            EGMGetId egm_id;
            EGMSetupUC ROB_L, egm_id, "raw", "ROB_L", \Joint;

            EGMActJoint egm_id
                \J1:=egm_condition
                \J2:=egm_condition
                \J3:=egm_condition
                \J4:=egm_condition
                \J5:=egm_condition
                \J6:=egm_condition
                \J7:=egm_condition
                \MaxSpeedDeviation:=500.0;

            EGMRunJoint egm_id, EGM_STOP_HOLD, \J1 \J2 \J3 \J4 \J5 \J6 \J7 \CondTime:=5;

            EGMReset egm_id;

            WaitTime 5;
        ENDWHILE

    ERROR
        IF ERRNO = ERR_UDPUC_COMM THEN
            TPWrite "Communication timeout";
            TRYNEXT;
        ENDIF
    ENDPROC
ENDMODULE
