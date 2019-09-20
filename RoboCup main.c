#pragma config(Sensor, 	S1, 	ultrasense, sensorEV3_Ultrasonic)
#pragma config(Sensor, 	S2, 	calbutton, 	sensorEV3_Touch)
#pragma config(Sensor, 	S3, 	colorsense, sensorEV3_Color)
#pragma config(Sensor, 	S4, 	homesense, 	sensorEV3_Touch)
#pragma config(Motor, 	motorA, motorL, 	tmotorEV3_Large, 	PIDControl, encoder)
#pragma config(Motor, 	motorB, motorR, 	tmotorEV3_Large, 	PIDControl, encoder)
#pragma config(Motor, 	motorC, klomotor, 	tmotorEV3_Medium, 	PIDControl, encoder)

// Sættes til true når task9 er gennemført
bool racedone = false;
// Bruges til at tænde og slukke for tælleren af sorte linjer
bool count_blacks = true;
// Bruges til at holde værdien af den nuværende opgave
int curr_task = 2;
// Bruges til at skifte siden af linescanning
int dir = -1;
// Bruges til at holde styr på antallet af krydsede sorte linjer
int black_counter = 0;
// variabel til at holde information om den kalibrerede linje
float perfect_line;
// Holder styr på om kloen er kalibreret
bool klo_kalibreret = false;

// Variabel til værdien af den hvide del af banen
float white_val = 64;
// Variabel til værdien af den grå del af banen
float gray_val = 37;
// Variabel til værdien af den sorte del af banen
float black_val = 20;

// Encoderværdien for en åben klo
const int klo_aaben = 7500;
// Encoderværdien for en lukket klo
const int klo_luk = 4500;
// Encoderværdien for en løftet klo
const int klo_loeft = 0;

// Sætter motorhastigheden på begge motorer
void driveSpeed(int Left, int Right);
{
	setMotorSpeed(motorL, -Left);
	setMotorSpeed(motorR, -Right);
}

// Bruges til at følge en linje ved hjælp af et PID udregninger
void Linefollow_PID(float speed = 20)
{
	// PID konstanter
	float Kp = 0.2;
	float Ki = 0.00001;
	float Kd = 0.035;

	// variabler brugt til PID
	float deltaErr = 0;
	float turn = 0;
	float errors = 0;
	float error_sum = 0;
	float prev_error = 0;
	float color_diff;

	// Udregn størrelsen af fejlen : Proportionel
	errors = dir * (SensorValue(colorsense) - perfect_line);

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

// Bruges til at køre en bestemt distance med PID lonefollowing
void PID_distance(float cm, float speed = 20)
{
	float maal = (360 / (5.5 * PI)) * cm;	  //Formel for at beregne hvor mange "ticks" den skal k?re for en hvis l?ngde(der er indsat 10cm)
	resetMotorEncoder(motorL);				   //resetter venstre motors encoder
	resetMotorEncoder(motorR);				   //resetter højre motors encoder
	float distanceR = getMotorEncoder(motorR); //giver værdien for h?jre og venste's encoder
	float distanceL = getMotorEncoder(motorL);
	float distance = (distanceR + distanceL) / 2; //gennemsnit for de to tick v?rdier

	while (distance < maal)
	{ // while loop der stoppe n?r robotten har k?rt en x l?ngde
		distanceR = -getMotorEncoder(motorR);
		distanceL = -getMotorEncoder(motorL);
		distance = (distanceR + distanceL) / 2;
		Linefollow_PID(speed); //linefollow
	}
	delay(200);
}

// Funktion som stopper begge motorer
void driveStop()
{
	setMotorSpeed(motorL, 0);
	setMotorSpeed(motorR, 0);
}

// Bruges til at dreje(x) antal grader
void dreje(float turn_degrees)
{
	driveStop();
	float hjul_om = 5.5;												   //hjulets omkreds i cm
	float sporvidde = 13.4;												   //sporvidde p� bilen
	float correction = 1.032;											   //float til at lave sm� corrections p� m�ngden bilen drejer
	float calc_turn = correction * (sporvidde * (turn_degrees / hjul_om)); //udregning af antal grader motoren skal dreje
	resetMotorEncoder(motorL);
	resetMotorEncoder(motorR);

	while (abs(getMotorEncoder(motorL)) < (abs(calc_turn) - 6))
	{
		if (turn_degrees > 0)
		{
			driveSpeed(10,-10);
		}
		if (turn_degrees < 0)
		{
			driveSpeed(-10,10);
		}
	}
	driveStop();
	//delay(200);
}

// Bruges til at drive(x) antal centimeter
void drive(float CM, int speedX = 20)
{
	driveStop();
	float forwardT = (360 / (5.5 * PI)) * CM; //udregning af rotation i grader motoren skal køre
	resetMotorEncoder(motorL);				  //5,5 er hjulstørrelsen
	resetMotorEncoder(motorR);

	while (abs(getMotorEncoder(motorL)) < (abs(forwardT) - 6))
	{
		if (CM < 0)
		{
			driveSpeed(-speedX,-speedX);
		}
		if (CM > 0)
		{
			driveSpeed(speedX,speedX);
		}
	}
	driveStop();
	//delay(200);
}

// Bruges til at scanne efter en flaske på banen
void scan(float venstre_scan = 45, float hojre_scan = 45)
{
    float old_scan_dist = 256.0;
    int scan_directionL;
    int scan_directionR;
    float hjul_omA = 5.5;                                                         //hjulets omkreds i cm
    float sporviddeA = 13.4;                                                      //sporvidde p�?�?�?¯�?�??�?¿ï¿½
    float correctionA = 1;                                                        //float til at lave sm�?�?�?¯�?�??�?¿�?�??�?½ corrections p�?�?�?¯�?�??�?¿�?�??�?½ m�?�?ï¿½
    float first_turn = correctionA * (sporviddeA * ((-venstre_scan) / hjul_omA)); //udregning af antal grader motoren skal dreje f�?�?�?¯�?�??
    resetMotorEncoder(motorL);
    resetMotorEncoder(motorR);
    while (abs(getMotorEncoder(motorL)) < (abs(first_turn) - 6))
    {
        driveSpeed(-25,25);
    }
    driveStop();
    float second_turn = correctionA * (sporviddeA * (((hojre_scan + venstre_scan)) / hjul_omA));
    resetMotorEncoder(motorL);
    resetMotorEncoder(motorR);
    while (abs(getMotorEncoder(motorL)) < (abs(second_turn) - 6))
    {
        driveSpeed(5,-5);
        if (getUSDistance(ultrasense) < old_scan_dist) //scanner for objekter t�?�?�?¦t p�?�?�?¥ og gemmer motorposition for n�?�?�?¦rmest
        {
            playTone(50, 5);
            scan_directionL = getMotorEncoder(motorL);
            scan_directionR = getMotorEncoder(motorR);
            old_scan_dist = getUSDistance(ultrasense);
        }
    }
    driveStop();
    //delay(500); //test
    while (abs(getMotorEncoder(motorL)) < (abs(scan_directionL) - 50)||abs(getMotorEncoder(motorL)) > (abs(scan_directionL) - 6))
    {
        driveSpeed(-10,10);
    }
    driveStop();
}

// Mario coin lyd
void coinSound()
{
	int B_4 = 987;
	int E_5 = 1322;
	//Duration
	int Whole = 37;
	int Quarter = 12;

	int notes[][] = {
		{B_4, Whole},
		{E_5, 74},
	};

	for (int i = 0; i < 19; i++) //change the "62" to the new number of notes in the piece
	{
		playTone(notes[i][0], notes[i][1]);
		while (bSoundActive)
			;
		wait1Msec(20);
	}
}

// Tæller hvor mange sorte linjer robotten kører over. on/off ved count_blacks = true/false
void black_line_counter() //timer2
{
	if (time1[T2] > 3000 && SensorValue(colorsense) < black_val && SensorValue(calbutton) == 0 && count_blacks == true)
	{
		coinSound();
		black_counter++;
		clearTimer(T2);
	}
}

// Bruges til at kalibrere kloens placering
void klo_cal(int klo_pos = 4500) //timer4
{
	clearTimer(timer4);
	while (klo_kalibreret == false)
	{
		while (time1[timer4] < 500)
		{
			setMotorSpeed(klomotor, 20); //for at sikre switchen ikke er trykket fra starten
		}
		setMotorSpeed(klomotor, -70);
		if (getTouchValue(homesense) == 1)
		{ //inds�t switchnavn p� "a"'s plads
			setMotorSpeed(klomotor, 0);
			resetMotorEncoder(klomotor);
			setMotorTarget(klomotor, klo_pos, 100);
			while (getMotorEncoder(klomotor) < klo_pos - 4)
			{
			}
			klo_kalibreret == true;
		}
	}
}

// Holder stille mens kloen åbnes
void aaben_klo()
{
    while (getMotorEncoder(klomotor) < (klo_aaben - 6)) // �?bner kloen
    {
        setMotorSpeed(klomotor, 100); // Slip flasken
    }
	setMotorSpeed(klomotor, 0);
}

// Holder stille mens kloen lukkes
void luk_klo()
{
	while (getMotorEncoder(klomotor) < (klo_luk - 6) || getMotorEncoder(klomotor) > (klo_luk + 6)) // Lukker
	{
		if (getMotorEncoder(klomotor) < (klo_luk - 6))
		{
			setMotorSpeed(klomotor, 100); // Sænk kloen
		}
		else if (getMotorEncoder(klomotor) > (klo_luk + 6))
		{
			setMotorSpeed(klomotor, -100); // Luk kloen
		}
	}
	setMotorSpeed(klomotor, 0);
}

// Holder stille mens kloen lukkes og løftes
void loeft_klo()
{
    while (getMotorEncoder(klomotor) > (klo_loeft + 6)) // Lukker og løfter
    {
        setMotorSpeed(klomotor, -100); // løft kloen
    }
	setMotorSpeed(klomotor, 0);
}

// Bruges til at kalibrere de grå og hvide farver på banen
void color_cal() //timer3
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
			int sens = SensorValue[colorsense];
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
			int sens = SensorValue[colorsense];
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

// Mario theme sang + nedtælling
void introSong()
{

	int G = 392;
	int A = 440;
	int C_4 = 524;
	int E_4 = 656;
	int A_4 = 877;
	int G_4 = 788;
	//Duration
	int Whole = 37;
	int Quarter = 12;

	int notes[][] = {
		{E_4, Quarter},
		{E_4, Quarter},
		{-2, Quarter},
		{E_4, Quarter},
		{-2, Quarter},
		{C_4, Quarter},
		{E_4, Quarter},
		{-2, Quarter},
		{G_4, 40},
		{-2, Quarter},
		{G, 40},
		{-2, Whole},
		{A, 60},
		{-2, Quarter},
		{A, 60},
		{-2, Quarter},
		{A, 60},
		{-2, Quarter},
		{A_4, 80},
	};

	for (int i = 0; i < 19; i++) //change the "62" to the new number of notes in the piece
	{
		playTone(notes[i][0], notes[i][1]);
		while (bSoundActive)
			;
		wait1Msec(20);
	}
}

// Musik til når banen er gennemført.
void CelebrationMusic()
{
	int C = 261;
	int D = 293;
	int Dsh = 310;
	int E = 329;
	int F = 348;
	int G = 392;
	int Gsh = 412;
	int Ash = 466;
	int C_4 = 524;
	int D_4 = 590;
	int Dsh_4 = 621;
	int E_4 = 656;
	int F_4 = 695;
	int G_4 = 788;
	int Gsh_4 = 830;
	int Ash_4 = 929;
	int C_5 = 1054;
	//Duration
	int Dwhole = 50;
	int Whole = 37;
	int Triplets = 12;

	int notes[][] = {
		{C, Triplets},
		{E, Triplets},
		{G, Triplets},
		{C_4, Triplets},
		{E_4, Triplets},
		{G_4, Whole},
		{E_4, Whole},
		{-2, Triplets},
		{C, Triplets},
		{Dsh, Triplets},
		{Gsh, Triplets},
		{C_4, Triplets},
		{Dsh_4, Triplets},
		{Gsh_4, Whole},
		{Dsh_4, Whole},
		{-2, Triplets},
		{D, Triplets},
		{F, Triplets},
		{Ash, Triplets},
		{D_4, Triplets},
		{F_4, Triplets},
		{Ash_4, Whole},
		{Ash_4, Triplets},
		{Ash_4, Triplets},
		{Ash_4, Triplets},
		{C_5, 130},
	};

	for (int i = 0; i < 26; i++) //change the "62" to the new number of notes in the piece
	{
		playTone(notes[i][0], notes[i][1]);
		while (bSoundActive)
			;
		wait1Msec(20);
	}
}

//
// Funktionerne til de individuelle opgaver.
//

void task1()
{
	if (black_counter == 0)
	{
        //luk_klo();
		Linefollow_PID(30);
	}
	if (black_counter == 1)
	{
		for (int i; i < 1; i++)
		{
			dreje(+45);
			drive(30);
			dreje(-45);
		}
		Linefollow_PID(30);
	}
	if (black_counter == 2)
	{
		for (int i; i < 1; i++)
		{
			dreje(-45);
			drive(30);
			dreje(+45);
		}
		curr_task++;
	}
}
void task2()
{
	if (black_counter < 2)
	{
		if (klo_kalibreret == false){
		klo_cal(klo_aaben);}
		black_counter = 2;
	}
	if (black_counter == 2)
	{
		Linefollow_PID(40);
	}
	if (black_counter == 3)
	{
		int flaskevej = 0;
		while (flaskevej == 0) // på langsiden
		{
			setMotorTarget(klomotor, klo_aaben, 100);
			drive(20, 50);
			dreje(90);
			PID_distance(10);
			scan(20,20);
			flaskevej++;
		}

		while (flaskevej == 1) // på vej
		{
			if (getUSDistance(ultrasense) >= 20)
			{
				driveSpeed(10,10);
			}

			if (getUSDistance(ultrasense) >= 8.5 && getUSDistance(ultrasense) < 20 || getUSDistance(ultrasense) < 7)
			{
				driveSpeed(4,4);
			}
			if (getUSDistance(ultrasense) < 8.5 && getUSDistance(ultrasense) >= 7)
			{
				flaskevej++;
			}
		}
		while (flaskevej == 2) // løfter flasken.
		{
			driveStop();
			delay(500);
			int motorlcode = getMotorEncoder(motorL);
			int motorrcode = getMotorEncoder(motorR);
			setMotorTarget(motorL, motorlcode - 100, 3);
			setMotorTarget(motorR, motorrcode - 100, 3);
			loeft_klo(); // Løft klo mens robotten står stille
			setLEDColor(ledRed);
			delay(3000);
			flaskevej++;
		}

		while (flaskevej == 3) // k6rer til den sorte streg og lægger flasken
		{
			drive(20, 30);
			//black_counter = 4;
			delay(2000);
			driveStop();
			aaben_klo();
			setLEDColor(ledGreen);
			delay(2000);
			flaskevej++;
		}

		while (flaskevej == 4) // kører baglæns.
		{
			drive(-20, 30);
			dreje(-135);
			flaskevej++;
		}

		while (flaskevej == 5) // kører nu til langsiden
		{
			setMotorTarget(klomotor, klo_luk, 100);
			while (SensorValue(colorsense) > perfect_line)  // Imens sensoren læser hvid
			{
				driveSpeed(20,20);
			}
			drive(10)
			dreje(45);
			flaskevej++;
		}
		curr_task++;
	}
	if (black_counter == 4)
	{
	}
	// Indsæt opgave 2 loop her **********************
}
void task3()
{
	if (black_counter < 4)
	{
		black_counter = 4;
	}
	if (black_counter == 4)
	{
		Linefollow_PID();
	}
	if (black_counter == 5)
	{
		for (int i; i < 1; i++)
		{
			drive(25, 30); // Kør frem til linjen
			dreje(-90);	// drej 90 grader for at komme rigtigt på næste linje
		}
		Linefollow_PID(); // følg linjen igen
	}
	if (black_counter == 6)
	{
		for (int i; i < 1; i++)
		{
			drive(50, 60);		   // Kør HURTIGT op over rampen
			PID_distance(70);	  // Følg rampen med PID 80
			drive(20);			   // Kør 20 cm ned over rampen
			PID_distance(55);	  // Tænd for PID over 32 CM
			dreje(-90);			   // Dreje 90 grader tilbage på sporet
		}
		curr_task++;
	}
}
void task4()
{
	if (black_counter < 6) // Betingelser for udførelse af opgave 4
	{
		black_counter = 6;
	}

	if (black_counter == 6)
	{
		Linefollow_PID();
	}
	if (black_counter == 7)
	{
		PID_distance(20);
		dreje(-45);
		drive(26); // stod på 40 før
		while (SensorValue(colorsense) > perfect_line)
		{
			driveSpeed(-20,-20);
		}
		dreje(+45);
		curr_task++;
	}
}
void task5()
{
	if (black_counter < 7)
	{
		if (klo_kalibreret == false){
		klo_cal(klo_aaben);}
		black_counter = 7;
	}
	if (black_counter == 7)
	{
		Linefollow_PID(); // Følg linjen
	}
	if (black_counter == 8)
	{
		for (int i; i < 1; i++)
		{
			PID_distance(22);					// Følg linjen i 20 cm
			dreje(-90);							// Drej 90 grader mod den nye linje
			setMotorTarget(klomotor, klo_aaben, 60); // �?ben kloen
		}
		Linefollow_PID(); // følg linjen
	}
	if (black_counter == 9)
	{
		for (int i; i < 1; i++)
		{
			drive(60, 20); //Kør frem til midten uden PID											   // reset motorencoder
			scan(80, 40);	
			resetMotorEncoder(motorL);
			resetMotorEncoder(motorR);										   // scan efter flaske, og peg på den
			while (getUSDistance(ultrasense) > 8.5 || getUSDistance(ultrasense) < 7) // Imens ultrasense er mellem 7.8 og 70 cm
			{	
				driveSpeed(20,20);
			}
			int motorlcode = getMotorEncoder(motorL);
			int motorrcode = getMotorEncoder(motorR);
			setMotorTarget(motorL, motorlcode - 100, 5);
			setMotorTarget(motorR, motorrcode - 100, 5);
            loeft_klo(); // Kloen lukker og løfter flasken

            while (getMotorEncoder(motorL) < 0)            // Imens motorencoderen er over nulpunktet
            {
                driveSpeed(-20,-20);
            }
            drive(-22);										// Kør yderligere 20 cm tilbage
			aaben_klo();                                    // Kloen åbnes og flasken stilles
			drive(-20);									    // Kør yderligere 20 cm tilbage
			dreje(-180);
			setMotorTarget(klomotor, klo_loeft, 100);								    // drej tilbage mod banen
			drive(50);									    // Kør ud af skydeskive
			while (SensorValue(colorsense) > perfect_line)  // Imens sensoren læser hvid
			{
				driveSpeed(20,20);
			}
			drive(10);  // Kør yderligere 20 frem
			dreje(-45); // dreje tilbage på banen
		}
		curr_task++;
	}
}
void task6_8()
{
	if (curr_task == 6 && black_counter < 9)
	{
		black_counter = 9;
	}
	else if (curr_task == 8 && black_counter < 11)
	{
		black_counter = 11;
	}

	if (black_counter == 9 || black_counter == 11)
	{
		Linefollow_PID(30);
	}
	else if (black_counter == 10 || black_counter == 12)
	{
		drive(4);
		dreje(45);
		drive(15);
		clearTimer(T1);
		while (SensorValue(colorsense) > perfect_line || time1[T1] < 2000)
		{
			driveSpeed(20,28);
		}
		if (curr_task == 6)
		{
			drive(7);
			dreje(40);
		}
		if (curr_task == 8)
		{
			drive(5);
			dreje(30);
		}
		
		Linefollow_PID();
		curr_task++;
	}
}
void task7()
{
	if (black_counter < 10)
	{
		black_counter = 10;
	}
	if (black_counter == 10)
	{
		Linefollow_PID(30);
	}
	if (black_counter == 11)
	{
		for (int i; i < 1; i++)
		{
			//setMotorTarget(klomotor, klo_loeft, 100);
			drive(42);
			dreje(-35);
			drive(16);
			clearTimer(timer3);
			while (time1[timer3] < 3100)
			{
				driveSpeed(38,24);
			}
			dreje(-50);
			drive(30);
		}
		curr_task++;
	}
}
void task9()
{
	if (black_counter < 12)
	{
		black_counter = 12;
	}
	if (black_counter == 12)
	{
		Linefollow_PID(30);
	}
	if (black_counter == 13)
	{
		dreje(-50);
		drive(16);
		dreje(30);
		drive(8);
		PID_distance(152,30);
		dreje(90); //drejer 90 grader
		curr_task++;
	}

}
void task10()
{
	CelebrationMusic();
	racedone = true;
}

task main()
{
	perfect_line = gray_val + ((white_val - gray_val) / 2); // Udregner den perfekte linje én gang i starten
	while (racedone == false)								// Main loop til at køre når race endnu ikke er færdig
	{
		black_line_counter(); // Den sorte tæller kører altid, med mindre count_blacks bliver sat til false

		if (curr_task == 0) // done
		{
			klo_cal();   // Kloen kalibreres som det første, så vi ved hvor den er
			//color_cal(); // farvekalibrering kører i starten
			introSong();
			curr_task++;
		}

		if (curr_task == 1) // done
		{
			task1();
		}

		if (curr_task == 2) // somewhat
		{
			task2();
		}

		if (curr_task == 3) // somewhat
		{
			task3();
		}

		if (curr_task == 4) // done
		{
			task4();
		}

		if (curr_task == 5) // done?
		{
			task5();
		}

		if (curr_task == 6 || curr_task == 8) // done
		{
			task6_8();
		}

		if (curr_task == 7) // tweaking
		{
			task7();
		}

		if (curr_task == 9) // somewhat
		{
			task9();
		}

		if (curr_task == 10) // done - celebrateion!
		{
			task10();
		}
	}
}
