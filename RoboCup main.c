#pragma config(Sensor, S1, ultrasonic, sensorEV3_Ultrasonic)
#pragma config(Sensor, S2, touchsens, sensorEV3_Touch)
#pragma config(Sensor, S3, colorsens, sensorEV3_Color)
#pragma config(Sensor, S4, homesens, sensorEV3_Touch)
#pragma config(Motor, motorA, MotorL, tmotorEV3_Large, PIDControl, encoder)
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
bool linetrack = true;	// Bruges til at skifte mellem linetracking og frikørsel
bool racedone = false;	// Sættes automatisk til sandt når task9 er gennemført
bool celebration = false; // Sættes til true sammen med racedone, og sættes til false for at afslutte hele programmet.
int curr_task = 0;		  // For at teste specifikke udfordringer, skift dette tal
int dir = 1;			  // sæt 1 for at køre på venstre side af grå streg, 0 for at køre på højre
int black_counter;		  // Bruges til at holde styr på antallet af krydsede sorte linjer
float perfect_line;		  // variabel til at holde information om den kalibrerede linje
float speed = 25;		  // Robottens hastighed i PID-loopet.

// Variabel til at holde sensor aflæsning
int line_sensor_val;

float white_val = 73; // Variabel til værdien af den hvide del af banen
float gray_val = 40;  // Variabel til værdien af den grå del af banen
float black_val = 5;  // Variabel til værdien af den sorte del af banen

task Linefollow_PID();
//task grap_homing();
//task color_calibrate();
task main()
{
	perfect_line = gray_val + ((white_val - gray_val) / 2);
	while (racedone == false) // Main loop til at køre når race endnu ikke er færdig
	{
		line_sensor_val = SensorValue(colorsens); // Værdien der læses fra farvesensoren. Skal næsten altid bruges, og er derfor i starten af vores loop.

		//Line tacking function, kan slås til/fra ved at sætte linetrack = true; / linetrack = false;
		if (linetrack == true)
		{
			startTask(Linefollow_PID);
		}

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

		if (curr_task == 4)
		{
			if (task4 == true) // Betingelser for udførelse af opgave 4
			{
				// Indsæt opgave 4 loop her **********************
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
				black_counter = black_counter++;
			}
		}

		if (curr_task == 6 || curr_task == 8) // Det her vil ikke virke. Skal nok få det ordnet.
		{
			if (task6 == true || task8 == true) // Betingelser for udførelse af opgave 6 og 8
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
task Linefollow_PID() // Funktionen der bruges til at følge linjen ved brug af PID-loop
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

	int color_diff = white_val - gray_val / 2;
	if (errors > color_diff)
		errors = color_diff;
	else if (errors > -color_diff)
		errors = -color_diff;

	// Udregn sum af fejl : Integral
	error_sum += errors;

	// Udregn fejl delta : Differentiale
	deltaErr = errors - prev_error;

	// Game nuværende som forrige
	prev_error = errors;

	// Calculate PID
	turn = (errors * Kp) + (error_sum * Ki) + (deltaErr * Kd);

	// Set motor speed
	setMotorSpeed(MotorL, speed - ((turn * speed) / 10));
	setMotorSpeed(motorR, speed + ((turn * speed) / 10));
}
/*
task grap_homing()
{
}*/

task color_calibrate() // Funktion til kalibrering af farvesensor
{
	bool color_cal;
	if (SensorValue(touchsens) == 1) // Hvis knappen er trykket ned i mere end 2 sekunder igangsættes farvekalibrering
	{
		clearTimer(T2); // reset af timer2
		while (SensorValue(touchsens) == 1)
		{
			setLEDColor(ledOrangeFlash);
			if (time1[T2] > 2000) // hvis timer 2 tælller over 2000 ms
			{
				color_cal = true; // igangsættes kalibrering
				setLEDColor(ledGreenFlash);
			}
		}
	}
	while (color_cal == true) // Når kalibrering er startet,
	{
		int data;
		int statebutton;
		data = SensorValue[colorsens];

		if (SensorValue(touchsens) == 1)
		{
			statebutton++;
			setLEDColor(ledRedFlash);
			sleep(700);
		}
		if (statebutton == 1) // kalibrering af grå farve
		{
			displayCenteredTextLine(2, "Calibrating");
			displayCenteredTextLine(3, "color sensor");
		}

		displayCenteredTextLine(2, "gray: ", SensorValue[colorsens]);
		if (statebutton == 2) // kalibrering af grå farve
		{
			gray_val = SensorValue[colorsens];
			setLEDColor(ledGreenFlash);
			sleep(700);
			setLEDColor(ledRedFlash);
			sleep(700);
		}
		displayCenteredTextLine(2, "white: ", SensorValue[colorsens]);
		if (statebutton == 3) // kalibrering af hvid farve
		{
			white_val = SensorValue[colorsens];
			setLEDColor(ledGreenFlash);
			sleep(700);
			setLEDColor(ledRedFlash);
			sleep(700);
			perfect_line = gray_val + ((white_val - gray_val) / 2);
			eraseDisplay();
			color_cal = false;
		}
	}
}