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
float speed = 25;		  // Robottens hastighed i PID-loopet.

//Variable til at dreje med
float turn_degrees;		//variabel man ændrer for at sætte drejningsvinklen. Positiv = højre, negativ er venstre

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
		errors = multi * (line_sensor_val - perfect_line);

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

//task grap_homing();
task color_calibrate();
task main()
{
	perfect_line = gray_val + ((white_val - gray_val) / 2); // udregner den perfekte linje én gang i starten
	while (racedone == false) // Main loop til at køre når race endnu ikke er færdig
	{
		line_sensor_val = SensorValue(colorsense); // Værdien der læses fra farvesensoren. Skal næsten altid bruges, og er derfor i starten af vores loop.
		
		Linefollow_PID(true);

		startTask(color_calibrate);

		if (curr_task == 1)
		{
			if (task1 == true) // Betingelser for udførelse af opgave 1
			{
				// Indsæt opgave 1 loop her **********************
			}
			else
			{
				curr_task++;
				black_counter = black_counter++;
			}
		}

		if (curr_task == 2)
		{
			if (task2 == true) // Betingelser for udførelse af opgave 2
			{
				// Indsæt opgave 2 loop her **********************
			}
			else
			{
				curr_task++;
				black_counter = black_counter++;
			}
		}

		if (curr_task == 3)
		{
			if (task3 == true) // Betingelser for udførelse af opgave 3
			{
				// Indsæt opgave 3 loop her **********************
			}
			else
			{
				curr_task++;
				black_counter = black_counter++;
			}
		}

		if (task4 == true && black_counter==6 ) // Betingelser for udførelse af opgave 4
			{
				linetrack=false;
				turn_degrees = -45;
				dreje();
	 			//et længde ud
				turn_degrees = +45;
				linetrack=true
				kør frem
				black_counter++;
				linetrack=false
				drej til venstre
				kør et stykke fram 
				drej til højre, linetrack=true
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
				black_counter = black_counter++;
			}
		}

		if (curr_task == 6 || curr_task == 8)
		{
			if (task6 == true && curr_task == 6 || task8 == true && curr_task == 8) // Betingelser for udførelse af opgave 6 og 8
			{
				// Indsæt opgave 6 og 8 loop her **********************
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
		// Åben og luk grappen
	}
}

/*
task grap_homing()
{
}*/

task color_calibrate() // Funktion til kalibrering af farvesensor
{
	bool color_cal;
	if (SensorValue(calbutton) == 1) // Hvis knappen er trykket ned i mere end 2 sekunder igangsættes farvekalibrering
	{
		clearTimer(T2); // reset af timer2
		while (SensorValue(calbutton) == 1)
		{
			setLEDColor(ledOrangeFlash);
			if (time1[T2] > 2000) // hvis timer 2 tælller over 2000 ms
			{
				color_cal = true; // igangsættes kalibrering
				setLEDColor(ledGreenFlash);
				sleep(500);
			}
		}
	}

	while (color_cal == true) // Når kalibrering er startet,
	{
		int calstate = 0;

		if (calstate == 0)
		{
			displayCenteredTextLine(2, "Calibrating");
			displayCenteredTextLine(3, "color sensor");
			sleep(1000);
			calstate++;
		}

		if (calstate == 1)
		{
			displayCenteredTextLine(2, "gray: ", SensorValue[colorsense]);
			displayCenteredTextLine(3, "press button to calibrate");
			if (SensorValue[calbutton] == 1)
			{
				gray_val = SensorValue[colorsense];
				eraseDisplay();
				displayCenteredTextLine(2, "Gray calibrated");
				setLEDColor(ledGreen);
				sleep(500);
				eraseDisplay();
				setLEDColor(ledOff);
				calstate++;
			}
		}

		if (calstate == 2)
		{
			displayCenteredTextLine(2, "white: ", SensorValue[colorsense]);
			displayCenteredTextLine(3, "press button to calibrate");
			if (SensorValue[calbutton] == 1)
			{
				white_val = SensorValue[colorsense];
				eraseDisplay();
				displayCenteredTextLine(2, "White calibrated");
				setLEDColor(ledGreen);
				sleep(500);
				eraseDisplay();
				setLEDColor(ledOff);
				calstate++;
			}
		}
		if (calstate == 3)
		{
			perfect_line = gray_val + ((white_val - gray_val) / 2);
			displayCenteredTextLine(2, "Calibration done");
			sleep(500);
			eraseDisplay();
			color_cal = false;
		}
	}
}
