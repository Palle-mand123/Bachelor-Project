PROGRAM _INIT
    // Initialisering
	PWM_Period      := 2000;
	COLOR_OUT       := FALSE;
	DISTANCE_1_OUT  := TRUE;
	DISTANCE_2_OUT  := TRUE;
	DISTANCE_3_OUT  := TRUE;

	MinDuty         := 400;
	MaxDuty         := 800;

	SERVO_1         := FALSE;
	SERVO_2         := FALSE;
	SERVO_3         := FALSE;

	ColorDetected   := FALSE;      // sikr nulstilling ved opstart
END_PROGRAM

PROGRAM _CYCLIC
    // 0) Håndter Start/Stop-knapper og nulstil ColorDetected ved STOP
	StartRTrig(CLK := BTN_1);
	StopRTrig (CLK := BTN_2);
	IF StartRTrig.Q THEN
		SystemRun := TRUE;
	ELSIF StopRTrig.Q THEN
		SystemRun       := FALSE;
		ColorDetected   := FALSE;  // nulstil latch ved stop
	END_IF

	// 1) Farvesensor tændes kun indtil vi har fundet en farve
	IF SystemRun 
		AND NOT ColorDetected 
		THEN
		COLOR_OUT := TRUE;
	ELSE
		COLOR_OUT := FALSE;
	END_IF

	// 2) Lås farven straks vi får et HIGH på R, G eller B
	IF COLOR_OUT 
		AND (COLOR_R OR COLOR_G OR COLOR_B)
		THEN
		ColorDetected := TRUE;
	END_IF

	// 3) Dist1/Servo1 (rød)
	Dist1StopTrig(
		CLK := DISTANCE_1_IN
	AND COLOR_R
	AND SystemRun
	AND SICK
	AND ColorDetected
	);
	IF Dist1StopTrig.Q THEN
		InDist1Stop := TRUE;
		SERVO_1     := TRUE;
		COLOR_R		  := FALSE;
		ColorDetected := FALSE;
		COLOR_OUT	  := TRUE;
	END_IF
	Dist1StopTimer(
		IN := InDist1Stop,
		PT := 200    // 200*10ms = 2s stop-periode
		);
	IF Dist1StopTimer.Q THEN
		InDist1Stop   := FALSE;
		SERVO_1       := FALSE;
		ColorDetected := FALSE;   // klar til næste farve
		COLOR_R		  := FALSE;
		ColorDetected := FALSE;
		COLOR_OUT	  := TRUE;
	END_IF

	// 4) Dist2/Servo2 (grøn)
	Dist2StopTrig(
		CLK := DISTANCE_2_IN
	AND COLOR_G
	AND SystemRun
	AND SICK
	AND ColorDetected
	);
	IF Dist2StopTrig.Q THEN
		InDist2Stop := TRUE;
		SERVO_2     := TRUE;
		COLOR_G		  := FALSE;
		ColorDetected := FALSE;
		COLOR_OUT	  := TRUE;
	END_IF
	Dist2StopTimer(
		IN := InDist2Stop,
		PT := 200
		);
	IF Dist2StopTimer.Q THEN
		InDist2Stop   := FALSE;
		SERVO_2       := FALSE;
		COLOR_G		  := FALSE;
		ColorDetected := FALSE;
		COLOR_OUT	  := TRUE;
	END_IF

	// 5) Dist3/Servo3 (blå)
	Dist3StopTrig(
		CLK := DISTANCE_3_IN
	AND COLOR_B
	AND SystemRun
	AND SICK
	AND ColorDetected
	);
	IF Dist3StopTrig.Q THEN
		InDist3Stop := TRUE;
		SERVO_3     := TRUE;
		COLOR_B		  := FALSE;
		ColorDetected := FALSE;
		COLOR_OUT	  := TRUE;
	END_IF
	Dist3StopTimer(
		IN := InDist3Stop,
		PT := 200
		);
	IF Dist3StopTimer.Q THEN
		InDist3Stop   := FALSE;
		SERVO_3       := FALSE;
		ColorDetected := FALSE;
		COLOR_B		  := FALSE;
		ColorDetected := FALSE;
		COLOR_OUT	  := TRUE;
	END_IF

	// 6) Båndets kørsel (override hvis en stop-fase er aktiv)
	IF InDist1Stop 
		OR InDist2Stop 
		OR InDist3Stop 
		THEN
		PWM_Output := 0;
	ELSIF SystemRun 
		AND SICK 
		AND ColorDetected
		THEN
		PWM_Output := PWM_Output := MinDuty + ( (AI1 - 25000) * 400 ) / 7000;

	ELSE
		PWM_Output := 0;
	END_IF
END_PROGRAM

PROGRAM _EXIT
    // Afslutning
END_PROGRAM