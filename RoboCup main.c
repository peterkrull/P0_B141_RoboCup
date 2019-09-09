
// Kode til konfigurering af I/O til Minstorms EV3
#pragma config(Sensor, S1, ultrasonic, sensorEV3_Ultrasonic)
#pragma config(Sensor, S2, touchsens, sensorEV3_Touch)
#pragma config(Sensor, S3, colorsens, sensorEV3_Color)
#pragma config(Motor, motorA, motorR, tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor, motorB, motorL, tmotorEV3_Large, PIDControl, encoder)
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

bool changetask = false;  	// bruges til at definere hvornår robotten er imellem to opgaver
bool linetrack = true;		// Bruges til at skifte mellem linetracking og frikørsel
bool racedone = false;		// Sættes automatisk til sandt når task9 er gennemført
bool celebration = false; 	// Sættes til true sammen med racedone, og sættes til false for at afslutte hele programmet.
int curr_task = 0;		  	// For at teste specifikke udfordringer, skift dette tal
int dir = 1;			  	// sæt 1 for at køre på venstre side af grå streg, 0 for at køre på højre
float perfect_line;		  	// variabel til at holde information om den kalibrerede linje
float speed = 10; 			// Robottens hastighed i PID-loopet. 

// PID værdier
float Kp = 0.15;
float Ki = 0.000001;
float Kd = 0.028;

// Variabel til at holde sensor aflæsning
int line_sensor_val;

// Konstante variabler til lysværdierne af stregerne
const float white_val = 64;
// bane 64 - papir 73
const float gray_val = 27;
// bane 38 - papir 40
const float black_val = 5;
//bane 5 - papir 7
const float speed = 20;


task Linefollow_PID();
task main()
{
	clearTimer(T1);
	perfect_line = gray_val + ((white_val - gray_val) / 2); // Udregner værdien af den perfekte grå/hvide linje ud fra de forudbestemte værdier

	while (racedone == false) // Main loop til at køre når race endnu ikke er færdig
	{
		int colorsens;
		line_sensor_val = SensorValue(colorsens); // Værdien der læses fra farvesensoren. Skal næsten altid bruges, og er derfor i starten af vores loop.

		//Line tacking function, kan slås til/fra ved at sætte linetrack = true; / linetrack = false;
		if (linetrack == true)
		{
			startTask(Linefollow_PID);
		}

		/* Selv-diskvalificerende
		short disctime = 30000;
		if (timer1 > disctime)
		{
			racedone = true;
		}
		*/
	
		// Betingelser for udførelse af opgave 1
		if (task1 == true && curr_task == 1)
		{
			// Indsæt opgave 1 loop her **********************
		}

		// Betingelser for udførelse af opgave 2
		else if (task2 == true && curr_task == 2 || task1 == false && curr_task == 1 && task2 == true)
		{
			if (curr_task == 1)
			{
				curr_task = 2;
			}

			// Indsæt opgave 2 loop her **********************
		}

		// Betingelser for udførelse af opgave 3
		else if (task3 == true && curr_task == 3 || task2 == false && curr_task == 2 && task3 == true)
		{
			if (curr_task == 2)
			{
				curr_task = 3;
			}

			// Indsæt opgave 3 loop her **********************
		}

		// Betingelser for udførelse af opgave 4
		else if (task4 == true && curr_task == 4 || task3 == false && curr_task == 3 && task4 == true)
		{
			if (curr_task == 3)
			{
				curr_task = 4;
			}

			// Indsæt opgave 4 loop her **********************
		}

		// Betingelser for udførelse af opgave 5
		else if (task5 == true && curr_task == 5 || task4 == false && curr_task == 4 && task5 == true)
		{
			if (curr_task == 4)
			{
				curr_task = 5;
			}

			// Indsæt opgave 5 loop her **********************
		}

		// Betingelser for udførelse af opgave 6 og 8 (de er ens)
		else if (task6 == true && curr_task == 6 || task5 == false && curr_task == 5 && task6 == true	 // opgave 6
				 || task8 == true && curr_task == 8 || task7 == false && curr_task == 7 && task8 == true) // opgave 8
		{
			if (curr_task == 6)
			{
				curr_task = 7;
			}

			else if (curr_task == 8)
			{
				curr_task = 9;
			}

			// Indsæt opgave 6 og 8 loop her **********************
		}

		// Betingelser for udførelse af opgave 7
		else if (task7 == true && curr_task == 7 || task6 == false && curr_task == 6 && task7 == true)
		{
			if (curr_task == 6)
			{
				curr_task = 7;
			}

			// Indsæt opgave 7 loop her **********************
		}

		// Betingelser for udførelse af opgave 9
		else if (task9 == true && curr_task == 9 || task8 == false && curr_task == 8 && task9 == true)
		{
			if (curr_task == 8)
			{
				curr_task = 9;
			}

			// Indsæt opgave 9 loop her **********************
		}
	}

	while (racedone == true && celebration == true) // Når der ikke er flere opgaver efter den diste udførte, er robotten færdig
	{
		// Afspil lyd
		// Kør rundt i cirkler
		// �?ben og luk grappen
	}
}

task Linefollow_PID() // Funktionen der bruges til at følge linjen ved brug af PID-loop
{
	// variabler brugt til PID
	// PID konstanter findes i toppen
	float deltaErr = 0;
	float turn = 0;
	float errors;
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

	// Udregn sum af fejl : Integral
	error_sum += errors;

	// Udregn fejl delta : Differentiale
	deltaErr = errors - prev_error;

	// Game nuværende som forrige
	prev_error = errors;

	// Calculate PID
	turn = (errors * Kp) + (error_sum * Ki) + (deltaErr * Kd);

	// Set motor speed
	setMotorSpeed(motorR, speed - ((turn * speed) / 10));
	setMotorSpeed(motorL, speed + ((turn * speed) / 10));
}

/*UDATERET KODE, KAN MÅSKE BRUGES SENERE
  Var det oprindelige forsøg på at identificere de forskellige farver og handle på dem.

// Sæt initial states for R og L
int stateR = 1;
int stateL = 0;
int stateonce = 0;
if (gray_val - gray_black_var <= line_sensor_val <= gray_val + white_gray_var) // is gray
{
	setLEDColor(ledGreen); // Sæt LED farve til grøn, når robotten er på en grå linje
	colorread = 1;
	for (stateonce = 0; stateonce < 1; stateonce++)
	{
		if (stateR > stateL)
		{
			stateR = 0;
			stateL = 1;
		}
		else if (stateR < stateL)
		{
			stateR = 1;
			stateL = 0;
		}
	}
	setMotorSpeed(motorR, 10);
	setMotorSpeed(motorL, 10);
}

else if (0 < line_sensor_val < gray_val - gray_black_var) // is black
{
	setLEDColor(ledOrange); // Sæt LED farve til orange, når robotten er på en sort linje
	colorread = 2;
	setMotorSpeed(motorR, 0);
	setMotorSpeed(motorL, 0);
}

else if (gray_val + white_gray_var < line_sensor_val < 100) // is white
{

	setLEDColor(ledRed); // Sæt LED farve til rød, når robotten er på det hvide
	colorread = 3;

	if (stateR < stateL) // Turning left
	{
		setMotorSpeed(motorR, 0);
		setMotorSpeed(motorL, 10);
		state = 0;
	}
	else if (stateR > stateL) // turning right
	{
		setMotorSpeed(motorR, 10);
		setMotorSpeed(motorL, 0);
		state = 0;
	}
*/
