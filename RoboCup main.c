#pragma config(Sensor, S1, ultrasense, sensorEV3_Ultrasonic)
#pragma config(Sensor, S2, calbutton, sensorEV3_Touch)
#pragma config(Sensor, S3, colorsense, sensorEV3_Color)
#pragma config(Sensor, S4, homesense, sensorEV3_Touch)
#pragma config(Motor, motorA, motorL, tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor, motorB, motorR, tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor, motorC, klomotor, tmotorEV3_Medium, PIDControl, encoder)

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

bool changetask = false;   // bruges til at definere hvornår robotten er imellem to opgaver
bool racedone = false;	 // Sættes automatisk til sandt når task9 er gennemført
bool count_blacks = true;  // Bruges til at tænde og slukke for tælleren af sorte linjer
int curr_task = 6;		   // For at teste specifikke udfordringer, skift dette tal
int dir = 0;			   // sæt 1 for at køre på venstre side af grå streg, 0 for at køre på højre
int black_counter = 9;	 // Bruges til at holde styr på antallet af krydsede sorte linjer
float perfect_line;		   // variabel til at holde information om den kalibrerede linje
float speed = 30;		   // Robottens hastighed i PID-loopet.
const float stdspeed = 20; // Standard
int sens;

// Variabel til at holde sensor aflæsning
int line_sensor_val;

float white_val = 64; // Variabel til værdien af den hvide del af banen
float gray_val = 37;  // Variabel til værdien af den grå del af banen
float black_val = 5;  // Variabel til værdien af den sorte del af banen

//
// LINEFOLLOW PID FUNKTION
//

void Linefollow_PID(bool enable_tracking) // Funktionen der bruges til at følge linjen ved brug af PID-loop
{
	if (enable_tracking == true)
	{
		// PID konstanter
		float Kp = 0.2;
		float Ki = 0.00001;
		float Kd = 0.035;
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
		setMotorSpeed(motorL, -speed - ((turn * speed) / 10));
		setMotorSpeed(motorR, -speed + ((turn * speed) / 10));
	}
}

//
// FUNKTION TIL AT køre en distance med PID tændt
//

void PID_distance(float cm)
{
	float maal = (360 / (5.5 * PI)) * cm;	  //Formel for at beregne hvor mange "ticks" den skal k?re for en hvis l?ngde(der er indsat 10cm)
	resetMotorEncoder(motorL); //resetter venstre motors encoder
	resetMotorEncoder(motorR); //resetter højre motors encoder
	float distanceR = getMotorEncoder(motorR); //giver værdien for h?jre og venste's encoder
	float distanceL = getMotorEncoder(motorL);
	float distance = (distanceR + distanceL) / 2; //gennemsnit for de to tick v?rdier
	
	while (distance < maal)
	{ // while loop der stoppe n?r robotten har k?rt en x l?ngde
		maal = (360 / 5.5 * PI) * cm;
		distanceR = getMotorEncoder(motorR);
		distanceL = getMotorEncoder(motorL);
		distance = (distanceR + distanceL) / 2;
		Linefollow_PID(true); //linefollow
	}
	delay(200);
}

//
// FUNKTION TIL AT DREJE X ANTAL GRADER
// Indsæt en værdi i dreje(xx); for at dreje det antal grader.

void dreje(float turn_degrees)
{																		   //turn_degrees er lig antal grader bilen drejer. Positiv = h�jre, negativ = venstre
	float hjul_om = 5.5;												   //hjulets omkreds i cm
	float sporvidde = 13.4;												   //sporvidde p� bilen
	float correction = 1.032;											   //float til at lave sm� corrections p� m�ngden bilen drejer
	float calc_turn = correction * (sporvidde * (turn_degrees / hjul_om)); //udregning af antal grader motoren skal dreje
	resetMotorEncoder(motorL);
	resetMotorEncoder(motorR);
	setMotorTarget(motorL, -calc_turn, 10);
	setMotorTarget(motorR, calc_turn, 10);
	while (abs(getMotorEncoder(motorL)) < (abs(calc_turn) - 6))
	{ //de minus 4 er en buffer in case motoren ikke rammer target pr�cis
	} //while loopet eksisterer for at sikre at robotten er helt drejet f�r den udf�rer ny kode
	delay(200);
}

//
// FUNKTION TIL AT K�?RE ET ANTAL CM
// Indsæt to værdier i drive(x, y); for at køre ligeud for en afstand (x) ved en hastighed (y)

void drive(float CM, int speedX = stdspeed)
{
	float forwardT = (360 / (5.5 * PI)) * CM; //udregning af rotation i grader motoren skal køre
	resetMotorEncoder(motorL);				  //5,5 er hjulstørrelsen
	resetMotorEncoder(motorR);
	setMotorTarget(motorL, -forwardT, abs(speedX));
	setMotorTarget(motorR, -forwardT, abs(speedX));
	while (abs(getMotorEncoder(motorL)) < (abs(forwardT) - 6))
	{ //de minus 4 er en buffer in case motoren ikke rammer target pr?cis
	} //while loopet eksisterer for at sikre at robotten er k?rt f?rdig f?r den udf?rer ny kode
	delay(200);
}

//
// FUNKTION TIL AT SCANNE EFTER OBJEKTER SÅ SOM EN FLASKE
//

void scan(float venstre_scan = 45, float hojre_scan = 45)
{
	int old_scan_dist = 256;
	int scan_directionL;
	int scan_directionR;
	float hjul_omA = 5.5;														  //hjulets omkreds i cm
	float sporviddeA = 13.4;													  //sporvidde pï¿½ bi
	float correctionA = 1;														  //float til at lave smï¿½ corrections pï¿½ mï¿½ngden bil
	float first_turn = correctionA * (sporviddeA * ((-venstre_scan) / hjul_omA)); //udregning af antal grader motoren skal dreje fï¿½
	resetMotorEncoder(motorL);
	resetMotorEncoder(motorR);
	setMotorTarget(motorL, -first_turn, 10);
	setMotorTarget(motorR, first_turn, 10);
	while (abs(getMotorEncoder(motorL)) < (abs(first_turn) - 6))
	{
	}
	float second_turn = correctionA * (sporviddeA * ((hojre_scan + venstre_scan) / hjul_omA));
	resetMotorEncoder(motorL);
	resetMotorEncoder(motorR);
	setMotorTarget(motorL, -second_turn, 8);
	setMotorTarget(motorR, second_turn, 8);
	while (abs(getMotorEncoder(motorL)) < (abs(second_turn) - 6))
	{												   //de minus 4 er en buffer in case motoren ikke rammer target praecis
		if (getUSDistance(ultrasense) < old_scan_dist) //scanner for objekter tæt på og gemmer motorposition for nærmeste sted
		{
			scan_directionL = getMotorEncoder(motorL);
			scan_directionR = getMotorEncoder(motorR);
			old_scan_dist = getUSDistance(ultrasense);
		}
	}
	delay(500); //sikrer den fører
	setMotorTarget(motorL, scan_directionL, 5);
	setMotorTarget(motorR, scan_directionR, 5);
	while (abs(getMotorEncoder(motorL)) < (abs(scan_directionL) - 6))
	{ //de minus 4 er en buffer in case motoren ikke rammer target praecis
	}
}

//
// FUNKTION TIL AT T�?LLE SORTE STREGER
//

void black_line_counter()
{
	if (time1[T2] > 3000 && SensorValue(colorsense) < 20 && SensorValue(calbutton) == 0 && count_blacks == true)
	{
		black_counter++;
		clearTimer(T2);
	}
}

//
// klo cal
//

void klo_cal(int klo_pos = 1) //kalibrering af kloen (bruger timer4)
{int kalibration = 0;
	clearTimer(timer4);
	while(kalibration == 0){
		while(time1[timer4] < 500){
			setMotorSpeed(klomotor, 20);//for at sikre switchen ikke er trykket fra starten
		}
		setMotorSpeed(klomotor, -70);
		if (getTouchValue(homesense)== 1){ //inds�t switchnavn p� "a"'s plads
			setMotorSpeed(klomotor, 0);
			resetMotorEncoder(klomotor);
			setMotorTarget(klomotor, klo_pos, 100);
			while(getMotorEncoder(klomotor)< klo_pos-4){
			}
			kalibration = 1;
		}
	}
}

//
// FUNKTION TIL AT KALIBRERE GR�?/HVID FARVE
// Køres én gang i starten

void color_calibrate() // Funktion til kalibrering af farvesensor
{
	bool color_cal = false;
	while (color_cal == false)
	{
		clearTimer(T3);
		setLEDColor(ledGreen);			 // reset af timer3
		if (SensorValue(calbutton) == 1) // Hvis knappen er trykket ned i mere end 2 sekunder igangsættes farvekalibrering
		{
			while (SensorValue(calbutton) == 1)
			{
				setLEDColor(ledOrange);
				if (time1[T3] > 2000) // hvis timer 2 tælller over 2000 ms
				{
					color_cal = true; // igangsættes kalibrering
					setLEDColor(ledGreenFlash);
				}
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
			eraseDisplay();
			calstate++;
		}
		else if (calstate == 1)
		{
			char gray_val_LCD[15];
			sens = SensorValue[colorsense];
			sprintf(gray_val_LCD, "Gray val: %3d", sens);
			displayCenteredBigTextLine(2, gray_val_LCD);
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
		else if (calstate == 2)
		{
			char white_val_LCD[15];
			sens = SensorValue[colorsense];
			sprintf(white_val_LCD, "White val: %3d", sens);
			displayCenteredBigTextLine(2, white_val_LCD);
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
		else if (calstate == 3)
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
	klo_cal();
	perfect_line = gray_val + ((white_val - gray_val) / 2); // udregner den perfekte linje én gang i starten
	while (racedone == false)								// Main loop til at køre når race endnu ikke er færdig
	{
		line_sensor_val = SensorValue(colorsense); // Værdien der læses fra farvesensoren. Skal næsten altid bruges, og er derfor i starten af vores loop.
		black_line_counter();
		if (curr_task == 0) // Det initielle stadie af robotten. Setup placeres her
		{
			color_calibrate(); // farvekalibrering kører i starten
		}

		if (curr_task == 1)
		{
			if (task1 == true) // Betingelser for udførelse af opgave 1
			{
				if (black_counter == 0)
				{
					speed = 20;
					Linefollow_PID(true);
				}
				if (black_counter == 1)
				{
					for (int i; i < 1; i++)
					{
						count_blacks = false;
						Linefollow_PID(false);
						dreje(+45);
						drive(30);
						dreje(-45);
						count_blacks = true;
					}

					Linefollow_PID(true);
				}
				if (black_counter == 2)
				{
					for (int i; i < 1; i++)
					{
						count_blacks = false;
						Linefollow_PID(false);
						dreje(-45);
						drive(30);
						dreje(+45);
						count_blacks = true;
					}
					curr_task++;
				}
			}
			else
			{
				curr_task++;
				black_counter++;
			}
		}

		if (curr_task == 2)
		{

			if (task2 == true) // Betingelser for udførelse af opgave 2 - Khadar
			{
				if (black_counter == 2)
				{
					Linefollow_PID(true);
				}
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
				if (black_counter == 4)
				{
					Linefollow_PID(true);
				}
				if (black_counter == 5)
				{
					for (int i = 0; i < 1; i++)
					{
						drive(20, 30);
						dreje(-90);
					}
					Linefollow_PID(true);
				}
				if (black_counter == 6)
				{
					for (int i = 0; i < 1; i++)
					{
						playTone(1200, 200);
						Linefollow_PID(false);
						drive(40, 60);
						delay(500);
						PID_distance(80);
						delay(500);
						drive(20);
						delay(500);
						PID_distance(32);
						delay(500);
						dreje(-90);
						delay(500);
					}
					curr_task++;
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
				if (black_counter == 6)
				{
					Linefollow_PID(true);
				}
				else if (black_counter == 7)
				{
					dreje(-45);
					drive(40);
					while (SensorValue(colorsense) > perfect_line)
					{
						setMotorSpeed(motorR, 20);
						setMotorSpeed(motorL, 20);
					}
					dreje(+45);
					curr_task++;
				}
				else
				{
					curr_task++;
				}
			}
		}

		if (curr_task == 5)
		{
			if (task5 == true) // Betingelser for udførelse af opgave 5
			{
				if (black_counter == 7)
				{
					Linefollow_PID(true);
				}
				else if (black_counter == 8)
				{
					// Gør noget, når den sorte linje er opfanget
				}
				else if (black_counter == 9)
				{
					// Gør noget, når den ANDEN sorte linje er opfanget
				}
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
				if (black_counter == 9 || black_counter == 11)
				{
					Linefollow_PID(true);
				}
				if (black_counter == 10 || black_counter == 12)
				{
					count_blacks = false;
					Linefollow_PID(false);
					drive(4);
					dreje(45);
					drive(15);
					while (SensorValue(colorsense) > perfect_line)
					{
						setMotorSpeed(motorR, -28);
						setMotorSpeed(motorL, -20);
					}
					dreje(30);
					count_blacks = true;
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

				if (black_counter == 10)
				{
					Linefollow_PID(true);
				}
				if (black_counter == 11)
				{	
					Linefollow_PID(false);
					for (int i; i < 1; i++)
					{
						setMotorTarget(klomotor, 0, 100);
						drive(42);
						dreje(-35);
						drive(30);
						dreje(45);
						drive(15);
						dreje(45);
						drive(20);
						dreje(-40);
						drive(35);
					}
					curr_task++;
				}
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
				if (black_counter == 12)
				{
					Linefollow_PID(true);
				}
				// Indsæt opgave 9 loop her **********************
			}
			else
			{
				curr_task++;
			}
			if (curr_task == 10)
			{
				// Afspil lyd
				// Kør rundt i cirkler
				// �?ben og luk grappen
			}
		}
	}
}
