#pragma config(Sensor, S1, ultrasense, sensorEV3_Ultrasonic)
#pragma config(Sensor, S2, calbutton, sensorEV3_Touch)
#pragma config(Sensor, S3, colorsense, sensorEV3_Color)
#pragma config(Sensor, S4, homesense, sensorEV3_Touch)
#pragma config(Motor, motorA, motorL, tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor, motorB, motorR, tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor, motorC, grapmotor, tmotorEV3_Medium, PIDControl, encoder)

// Bruges til slå specifikke opgaver til/fra
#define task1 true  // Kør af banen of skift til anden grå linje
#define task2 true  // Flyt flaske bag den sorte streg
#define task3 true  // Kør over vippe
#define task4 true  // Find den 3. af 4 grå streger
#define task5 false // Flyt flasken til midten af skydeskiven
#define task6 true  // Kør rundt om flasken
#define task7 true  // Kør gennem hjørneformet forhindring
#define task8 true  // Kør rundt om flasken (gentagelse af task6)
#define task9 true  // Kør ind til midten af målstregen

bool changetask = false;  // bruges til at definere hvornår robotten er imellem to opgaver
bool racedone = false;	// Sættes automatisk til sandt når task9 er gennemført
bool celebration = false; // Sættes til true sammen med racedone, og sættes til false for at afslutte hele programmet.
int curr_task = 0;		  // For at teste specifikke udfordringer, skift dette tal
int dir = 1;			  // sæt 1 for at køre på venstre side af grå streg, 0 for at køre på højre
int black_counter;		  // Bruges til at holde styr på antallet af krydsede sorte linjer
float perfect_line;		  // variabel til at holde information om den kalibrerede linje
float speed = 20;		  // Robottens hastighed i PID-loopet.

// Variabel til at holde sensor aflæsning
int line_sensor_val;

float white_val = 80; // Variabel til værdien af den hvide del af banen
float gray_val = 43;  // Variabel til værdien af den grå del af banen
float black_val = 5;  // Variabel til værdien af den sorte del af banen

//
// LINEFOLLOW PID FUNKTION
//

void Linefollow_PID(bool enable_tracking) // Funktionen der bruges til at følge linjen ved brug af PID-loop
{
	if (enable_tracking == true)
	{
		// PID konstanter
		float Kp = 0.15;
		float Ki = 0.000001;
		float Kd = 0.028;
		/*
		float Kp = 0.1;
		float Ki = 0.000001;
		float Kd = 0.02;
*/
		// variabler brugt til PID
		float deltaErr = 0;
		float turn = 0;
		float errors = 0;
		float error_sum = 0;
		float prev_error = 0;
		float color_diff;
		int multi;

		// brugt til at ændre siden som robotten følger (ændrer fortegnet for fejlen)
		if (dir == 1)
		{
			multi = 1;
		}
		else if (dir == 0)
		{
			multi = -1;
		}

		// Udregn størrelsen af fejlen : Proportionel
		errors = multi * (SensorValue(colorsense) - perfect_line);

		color_diff = (white_val - gray_val) / 2; // Udregner størrelsen af farveforskellen mellem hvid, error og grå
		if (errors > color_diff)				 // Hvis fejlen er større end den målte hvide farve
		{
			errors = color_diff; // sæt fejlen lig den hvide farve
		}
		else if (errors < (-color_diff)) // Hvis fejlen er større end den målte grå farve
		{
			errors = (-color_diff); // sæt fejlen lig den grå farve
		}

		// Udregn sum af fejl : Integral
		error_sum += errors;
		// Udregn fejl delta : Differentiale
		deltaErr = errors - prev_error;

		// Game nuværende som forrige
		prev_error = errors;

		// Calculate PID
		turn = (errors * Kp) + (error_sum * Ki) + (deltaErr * Kd);

		// Set motor speed
		setMotorSpeed(motorL, speed - ((turn * speed) / 10));
		setMotorSpeed(motorR, speed + ((turn * speed) / 10));
	}
}

//
// FUNKTION TIL AT DREJE X ANTAL GRADER
// Indsæt en værdi i dreje(xx); for at dreje det antal grader.

void dreje(float turn_degrees)
{																		   //turn_degrees er lig antal grader bilen drejer. Positiv = højre, negativ = venstre
	float hjul_om = 6.5;												   //hjulets omkreds i cm
	float sporvidde = 12.4;												   //sporvidde på bilen
	float correction = 1;												   //float til at lave små corrections på mængden bilen drejer
	float calc_turn = correction * (sporvidde * (turn_degrees / hjul_om)); //udregning af antal grader motoren skal dreje
	resetMotorEncoder(motorL);
	resetMotorEncoder(motorR);
	setMotorTarget(motorL, calc_turn, 10);
	setMotorTarget(motorR, -calc_turn, 10);
	while (abs(getMotorEncoder(motorL)) < (abs(calc_turn) - 4))
	{ //de minus 4 er en buffer in case motoren ikke rammer target præcis
	} //while loopet eksisterer for at sikre at robotten er helt drejet før den udfører ny kode
}

//
// FUNKTION TIL AT KØRE ET ANTAL CM
// Indsæt en værdi i move(xx); for at kære ligeud

void drive(float CM)
{
	float forwardT = (360 / (5.5 * PI)) * CM;
	resetMotorEncoder(motorL);
	resetMotorEncoder(motorR);
	setMotorTarget(motorL, forwardT, 25);
	setMotorTarget(motorR, forwardT, 25);
	while (abs(getMotorEncoder(motorL)) < (abs(forwardT) - 4))
	{ //de minus 4 er en buffer in case motoren ikke rammer target pr�cis
	} //while loopet eksisterer for at sikre at robotten er k�rt f�rdig f�r den udf�rer ny kode
}

//
// FUNKTION TIL AT KALIBRERE GRÅ/HVID FARVE
// Køres én gang i starten

void color_calibrate() // Funktion til kalibrering af farvesensor
{
	bool color_cal;
	if (SensorValue(calbutton) == 1) // Hvis knappen er trykket ned i mere end 2 sekunder igangsættes farvekalibrering
	{
		clearTimer(T2); // reset af timer2
		while (SensorValue(calbutton) == 1 && color_cal == false)
		{
			setLEDColor(ledOrangeFlash);
			if (time1[T2] > 2000) // hvis timer 2 tælller over 2000 ms
			{
				color_cal = true; // igangsættes kalibrering
				setLEDColor(ledGreenFlash);
			}
		}
	}
	int calstate = 0;
	while (color_cal == true) // Når kalibrering er startet,
	{
		if (calstate == 0)
		{
			displayCenteredBigTextLine(2, "Calibrating");
			displayCenteredBigTextLine(4, "color sensor");
			sleep(2000);
			calstate++;
		}
		if (calstate == 1)
		{
			displayCenteredBigTextLine(2, "gray: ", "%f", SensorValue[colorsense]);
			displayCenteredTextLine(4, "");
			displayCenteredTextLine(5, "press button to calibrate");
			if (SensorValue[calbutton] == 1)
			{
				gray_val = SensorValue[colorsense];
				eraseDisplay();
				displayCenteredBigTextLine(2, "Gray calibrated");
				setLEDColor(ledGreen);
				sleep(1500);
				eraseDisplay();
				setLEDColor(ledOff);
				calstate++;
			}
		}
		if (calstate == 2)
		{
			displayCenteredBigTextLine(2, "white: ", "%f", SensorValue[colorsense]);
			displayCenteredTextLine(4, "");
			displayCenteredTextLine(5, "press button to calibrate");
			if (SensorValue[calbutton] == 1)
			{
				white_val = SensorValue[colorsense];
				eraseDisplay();
				displayCenteredBigTextLine(2, "White calibrated");
				setLEDColor(ledGreen);
				sleep(1500);
				eraseDisplay();
				setLEDColor(ledOff);
				calstate++;
			}
		}
		if (calstate == 3)
		{
			perfect_line = gray_val + ((white_val - gray_val) / 2);
			displayCenteredBigTextLine(2, "Calibration");
			displayCenteredBigTextLine(4, "done...");
			sleep(1000);
			displayCenteredBigTextLine(8, "Place robot");
			displayCenteredBigTextLine(10, "on the line");
			sleep(2000);
			eraseDisplay();
			curr_task++;
			color_cal = false;
		}
	}
}

task main()
{
	perfect_line = gray_val + ((white_val - gray_val) / 2); // udregner den perfekte linje én gang i starten
	while (racedone == false)								// Main loop til at køre når race endnu ikke er færdig
	{
		line_sensor_val = SensorValue(colorsense); // Værdien der læses fra farvesensoren. Skal næsten altid bruges, og er derfor i starten af vores loop.

		if (curr_task == 0) // Det initielle stadie af robotten. Setup placeres her
		{
			color_calibrate();
		}

		if (curr_task == 1)
		{
			if (task1 == true) // Betingelser for udførelse af opgave 1
			{
				Linefollow_PID(true);
				if (black_counter == 1)
				{
				}
				if (black_counter == 2)
				{
				}
				// Indsæt opgave 1 loop her **********************
			}
			else
			{
				curr_task++;
				black_counter++;
			}
		}

		if (curr_task == 2)
		{
			if (task2 == true) // Betingelser for udførelse af opgave 2
			{
				if (black_counter == 3)
				{
				}
				if (black_counter == 4)
				{
				}
				// Indsæt opgave 2 loop her **********************
			}
			else
			{
				curr_task++;
				black_counter++;
			}
		}

		if (curr_task == 3)
		{
			if (task3 == true) // Betingelser for udførelse af opgave 3
			{
				if (black_counter == 5)
				{
				}
				if (black_counter == 6)
				{
				}
				// Indsæt opgave 3 loop her **********************
			}
			else
			{
				curr_task++;
				black_counter++;
			}
		}

		if (curr_task == 4) // Betingelser for udførelse af opgave 4
		{
			if (task4 == true)
			{
				if (black_counter == 7)
				{
					float distanceR = getMotorEncoder(motorR);
					float distanceL = getMotorEncoder(motorL);
					float distance = (distanceR + distanceL) / 2;
					Linefollow_PID(false);
					dreje(-45);
					resetMotorEncoder(motorL);
					resetMotorEncoder(motorR);
					while (distance < 204.1)
					{
						distanceR = getMotorEncoder(motorR);
						distanceR = getMotorEncoder(motorL);
						distance = (distanceR + distanceL) / 2;
						motor[motorR] = 10; //kører med farten 10
						motor[motorR] = 10; //kører med farten 10
					}
					dreje(+45);
					Linefollow_PID(true);
				}
			}
			else
			{
				curr_task++;
			}
		}

		if (curr_task == 5)
		{
			if (task5 == true) // Betingelser for udførelse af opgave 5
			{
				// Indsæt opgave 5 loop her **********************
			}
			else
			{
				curr_task++;
				black_counter++;
			}
		}

		if (curr_task == 6 || curr_task == 8)
		{
			if (task6 == true && curr_task == 6 || task8 == true && curr_task == 8) // Betingelser for udførelse af opgave 6 og 8
			{
				// Indsæt opgave 6 og 8 loop her **********************
				if (black_counter == 10 || black_counter == 12)
				{
					drive(4);
					Linefollow_PID(false);
					dreje(80);
					delay(200);
					while (line_sensor_val > perfect_line)
					{
						setMotorSpeed(motorR, 12);
						setMotorSpeed(motorL, 10);
					}
					delay(200);
					dreje(-80);
					Linefollow_PID(true);
					curr_task++;
				}
			}
			else
			{
				curr_task++;
			}
		}

		if (curr_task == 7)
		{
			if (task7 == true) // Betingelser for udførelse af opgave 7
			{
				// Indsæt opgave 7 loop her **********************
			}
			else
			{
				curr_task++;
			}
		}

		if (curr_task == 9)
		{
			if (task9 == true) // Betingelser for udførelse af opgave 9
			{
				// Indsæt opgave 9 loop her **********************
			}
			else
			{
				curr_task++;
			}
		}
	}

	while (racedone == true && celebration == true) // Når der ikke er flere opgaver efter den diste udførte, er robotten færdig
	{
		// Afspil lyd
		// Kør rundt i cirkler
		// �?ben og luk grappen
	}
}
