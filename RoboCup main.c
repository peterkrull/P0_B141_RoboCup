#pragma config(Sensor, 	S1, 	ultrasense, sensorEV3_Ultrasonic)
#pragma config(Sensor, 	S2, 	calbutton, 	sensorEV3_Touch)
#pragma config(Sensor, 	S3, 	colorsense, sensorEV3_Color)
#pragma config(Sensor, 	S4, 	homesense, 	sensorEV3_Touch)
#pragma config(Motor, 	motorA, motorL, 	tmotorEV3_Large, 	PIDControl, encoder)
#pragma config(Motor, 	motorB, motorR, 	tmotorEV3_Large, 	PIDControl, encoder)
#pragma config(Motor, 	motorC, klomotor, 	tmotorEV3_Medium, 	PIDControl, encoder)

// Sættes til true når task9 er gennemført
bool racedone = false;
// Bruges til at holde værdien af den nuværende opgave
int curr_task = 4;
// Bruges til at holde styr på antallet af krydsede sorte linjer
int black_counter = 0;
// variabel til at holde information om den kalibrerede linje
float perfect_line;
// Holder styr på om kloen er kalibreret
bool klo_kalibreret = false;

// Variabel til værdien af den hvide del af banen
float white_val = 60;
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
void driveSpeed(int Left, int Right)
{
	setMotorSpeed(motorL, -Left);	// sætter venstre motorhastighed
	setMotorSpeed(motorR, -Right);	// sætter højre motorhastighed
}

int ultrafilter(int count = 50)
{
	int sum = 0; // int til at holde værdien af summen
	for (int i = 0; i < count ; i++) // følgende kode gentages *count* antal gange
	{
		sum = sum + getUSDistance(ultrasense); // Ultrasonisk sensor aflæses og summeres
	}
	return sum / count; // summen divideret med antal målinger; gennemsnit
}

// Bruges til at følge en linje ved hjælp af et PID udregninger
void Linefollow_PID(float speed = 30, bool followright = true)
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
	int dir;

	if (followright == true)
		{dir = -1;}
	if (followright == false)
		{dir = 1;}

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

	// Udregn korrigering på baggrund af PID
	turn = (errors * Kp) + (error_sum * Ki) + (deltaErr * Kd);

	// Set motor speed
	setMotorSpeed(motorL, -speed - ((turn * speed) / 10));
	setMotorSpeed(motorR, -speed + ((turn * speed) / 10));
}

// Bruges til at køre en bestemt distance med PID lonefollowing
void PID_distance(float cm, float speed = 30)
{
	float maal = (360 / (5.5 * PI)) * cm;	  		//Formel for at beregne hvor mange "ticks" den skal køre for en hvis l?ngde(der er indsat 10cm)
	resetMotorEncoder(motorL);				   		//resetter venstre motors encoder
	resetMotorEncoder(motorR);				   		//resetter højre motors encoder
	float distanceR = getMotorEncoder(motorL); 		//giver værdien for venstre encoder
	float distanceL = getMotorEncoder(motorR);		//giver værdien for højre encoder
	float distance = (distanceR + distanceL) / 2; 	//gennemsnit for de to tick v?rdier

	while (distance < maal)
	{ // while loop der stoppe n?r robotten har k?rt en x l?ngde
		distanceR = -getMotorEncoder(motorR);
		distanceL = -getMotorEncoder(motorL);
		distance = (distanceR + distanceL) / 2; // gennemsnittet af de to encoders distance udregnes
		Linefollow_PID(speed); //linefollow
	}
	//delay(200);
}

// Funktion som stopper begge motorer
void driveStop()
{
	setMotorSpeed(motorL, 0);
	setMotorSpeed(motorR, 0);
}

//resetter motorencoderne på begge motorer
void resetME()
{
    resetMotorEncoder(motorL);
    resetMotorEncoder(motorR);
}

// Bruges til at dreje(x) antal grader
void dreje(float turn_degrees)
{
	driveStop();
	float hjul_om = 5.5;												   //hjulets omkreds i cm
	float sporvidde = 13.4;												   //sporvidde på bilen
	float correction = 1.032;											   //float til at lave små corrections på mængden bilen drejer
	float calc_turn = correction * (sporvidde * (turn_degrees / hjul_om)); //udregning af antal grader motoren skal dreje
	resetMotorEncoder(motorL);
	resetMotorEncoder(motorR);

	while (abs(getMotorEncoder(motorL)) < (abs(calc_turn) - 6)) // imens motorencoderen ikke endnu ikke er nået den ønskede værdi
	{
		if (turn_degrees > 0) // hvis positivt fortegn
		{
			driveSpeed(10,-10);  // drej mod høre
		}
		if (turn_degrees < 0) // hvis negativt fortegn
		{
			driveSpeed(-10,10); // drej mod venstre
		}
	}
	driveStop();
}

// Bruges til at drive(x) antal centimeter
void drive(float CM, int speedX = 40)
{
	driveStop();
	float forwardT = (360 / (5.5 * PI)) * CM; //udregning af rotation i grader motoren skal køre
	resetMotorEncoder(motorL);				  //5,5 er hjulstørrelsen
	resetMotorEncoder(motorR);

	while (abs(getMotorEncoder(motorL)) < (abs(forwardT) - 6)) // imens motorencoderen endnu ikke er nået den ønskede værdi
	{
		if (CM < 0) // hvis negativt fortegn
		{
			driveSpeed(-speedX,-speedX); // kør med negativ hastighed
		}
		if (CM > 0) // hvis positivt fortegn
		{
			driveSpeed(speedX,speedX); // kør med positiv hastighed
		}
	}
	driveStop();
}

//scanner til x grader venstre og y grader hoejre. I V3 scanner den twice og finder position ud fra det
void scan(float venstre_scan = 45, float hojre_scan = 45, float min_dist = 7)
{
    float old_scan_dist0 = 256.0; //old_scan_dist og scan_direction gemmer distancen af tingen der er tættest paa
    float old_scan_dist1 = 256.0; // og gemmer hjulenes position ved den retning
    int scan_directionL0;
    int scan_directionR0;
    int scan_directionL1;
    int scan_directionR1;
    float hjul_omA = 5.5;                                                         //hjulets omkreds i cm
    float sporviddeA = 13.4;                                                      //sporvidde paa robot
    float correctionA = 1;                                                        //float til at lave smaa corrections in case vi ændrer robottens konstruktion
    float first_turn = correctionA * (sporviddeA * ((-venstre_scan) / hjul_omA)); //udregning af antal grader motoren skal dreje foerste gange
    resetME();
    while (abs(getMotorEncoder(motorL)) < (abs(first_turn) - 6)) //sving til x antal grader venstre før scan
    {                                                            //vi lader i disse typer while loops
        driveSpeed(-25, 25);                                     //motorerne koere indtil de har naaet oensket pos
    }                                                            //de -6 er en buffer incase motoren ikke rammer den praecise pos
    driveStop();
    float second_turn = correctionA * (sporviddeA * (((hojre_scan + venstre_scan)) / hjul_omA)); //udregning af antal grader motoren skal dreje anden gange
    resetME();
    while (abs(getMotorEncoder(motorL)) < (abs(second_turn) - 6))
    {
        driveSpeed(5, -5);
        if (ultrafilter() < old_scan_dist0 && ultrafilter() > min_dist) //scanner for objekter hoejre om og gemmer motorpos 0
        {
            playTone(50, 5); //lyd til at verify den finder flasken. Not necessary
            scan_directionL0 = getMotorEncoder(motorL);
            scan_directionR0 = getMotorEncoder(motorR);
            old_scan_dist0 = ultrafilter();
        }
    }
    driveStop();
    while (abs(getMotorEncoder(motorL)) > 6) //scanner for objekter venstre om og gemmer motorpos 1
    {
        driveSpeed(-5, 5);
        if (ultrafilter() < old_scan_dist1 && ultrafilter() > min_dist)
        {
            playTone(50, 5); //lyd til at verify den finder flasken. Not necessary
            scan_directionL1 = getMotorEncoder(motorL);
            scan_directionR1 = getMotorEncoder(motorR);
            old_scan_dist1 = ultrafilter();
        }
    }
    driveStop();
    float scanGNS = (scan_directionL0 + scan_directionL1) / 2; //gennemsnit af de to motorpos
    while (abs(getMotorEncoder(motorL)) < (abs(scanGNS)+3))  //drejer til gennemsnitpos
    {
        driveSpeed(10, -10);
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

	int notes[][] = {
		{B_4, Whole},
		{E_5, 74},
	};

	for (int i = 0; i < 19; i++) //change the "62" to the new number of notes in the piece
	{
		playTone(notes[i][0], notes[i][1]);
		while (bSoundActive)
		wait1Msec(20);
	}
}

// Tæller hvor mange sorte linjer robotten kører over.
void black_line_counter() //timer2
{
	// Hvis det er mere end 3 sekunder siden at timer 2 er blevet resat
	// og sensorens aflæste værdi er under den sorte værdi
	// og knappen på robotten ikke bliver holdt inde...
	if (time1[T2] > 3000 && SensorValue(colorsense) < black_val && SensorValue(calbutton) == 0)
	{
		// ... optæl en sort streg og reset timeren
		black_counter++;
		clearTimer(T2);
	}
}

// Bruges til at kalibrere kloens placering
void klo_cal(int klo_pos = klo_loeft) //timer4
{
	clearTimer(timer4); 
	while (klo_kalibreret == false) // Holder kalibreringen i gang
	{
		while (time1[timer4] < 500)
		{
			setMotorSpeed(klomotor, 20); //for at sikre switchen ikke er trykket fra starten
		}
		setMotorSpeed(klomotor, -70);
		if (getTouchValue(homesense) == 1)
		{ //inds?t switchnavn p? "a"'s plads
			setMotorSpeed(klomotor, 0);
			resetMotorEncoder(klomotor);
			setMotorTarget(klomotor, klo_pos, 100);
			while (getMotorEncoder(klomotor) < klo_pos - 4)
			{
			}
			klo_kalibreret = true; // afslutter kalibrering. bruges af opgave 2 og 5
		}
	}
}

// Holder stille mens kloen åbnes
void aaben_klo()
{
    while (getMotorEncoder(klomotor) < (klo_aaben - 6)) // ??bner kloen
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
		if (calstate == 0) // Introskærm til farvekalibrering
		{
			displayCenteredBigTextLine(2, "Calibrating");
			displayCenteredBigTextLine(4, "color sensor");
			sleep(2000);
			eraseDisplay();
			calstate++;
		}
		else if (calstate == 1) // den grå farve kalibreres
		{
			char gray_val_LCD[15]; // char array til at holde en linje tekst
			int sens = SensorValue[colorsense]; // farvesensorens værdi indsættes i en int
			sprintf(gray_val_LCD, "Gray val: %3d", sens); // sprintf bruges til at formatere tekst og tal ind i vores char array
			displayCenteredBigTextLine(2, gray_val_LCD); // vores char array udskrives til skærmen
			displayCenteredTextLine(4, ""); // blank linje
			displayCenteredTextLine(5, "press button to calibrate"); // tekst til bruger
			if (SensorValue[calbutton] == 1) // hvis knappen bliver trykket
			{
				gray_val = SensorValue[colorsense]; // overskriv den forrige værdi for grå
				eraseDisplay(); // fjern forrige tekst fra displayet
				displayCenteredBigTextLine(2, "Gray calibrated"); // bekræftigelse om udført kalibrering
				setLEDColor(ledGreen); // sæt LED grøn
				sleep(1500); // vent 1500ms
				eraseDisplay(); // clear displayet
				setLEDColor(ledOff); // sluk LED
				calstate++; // fortsæt til næste kalibreringsstadie
			}
		}
		else if (calstate == 2) // den hvide farve kalibreres ... kommentarer fra forrige er gældende her
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
		else if (calstate == 3) // afsluttende stadie af kalibrering
		{
			perfect_line = gray_val + ((white_val - gray_val) / 2); // perfect line udregnes
			displayCenteredBigTextLine(2, "Calibration"); // tekst til bruger om succes
			displayCenteredBigTextLine(4, "done...");
			sleep(1000);
			displayCenteredBigTextLine(8, "Place robot"); // gør robotten klar til at køre
			displayCenteredBigTextLine(10, "on the line");
			sleep(2000);
			eraseDisplay(); // clear displayet
			color_cal = false; // sæt color_cal til false, for at komme ud af while-loopet
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
		Linefollow_PID();
	}
	if (black_counter == 1)
	{
		for (int i; i < 1; i++)
		{
			dreje(+45);
			drive(30);
			dreje(-45);
		}
		Linefollow_PID();
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
		
		black_counter = 2;
	}
	if (black_counter == 2)
	{
		Linefollow_PID();
	}
	if (black_counter == 3)
	{	
		for (int i; i < 1; i++)
		{
			setMotorTarget(klomotor, klo_aaben, 100);
			if (klo_kalibreret == false)
			{
				driveStop();
				klo_cal(klo_aaben);
			}
		}
		int flaskevej = 0;
		while (flaskevej == 0) // på langsiden
		{
			drive(20, 50);
			dreje(90);
			PID_distance(8);
			scan(30,30);
			flaskevej++;
		}

		while (flaskevej == 1) // på vej
		{
			if (getUSDistance(ultrasense) >= 20)
			{
				driveSpeed(15,15);
			}

			if (getUSDistance(ultrasense) >= 8.5 && getUSDistance(ultrasense) < 20 || getUSDistance(ultrasense) < 7)
			{
				driveSpeed(8,8);
			}
			if (getUSDistance(ultrasense) < 8.5 && getUSDistance(ultrasense) >= 7)
			{
				flaskevej++;
			}
		}
		while (flaskevej == 2) // løfter flasken.
		{
			driveStop();
			//delay(500);
			int motorlcode = getMotorEncoder(motorL);
			int motorrcode = getMotorEncoder(motorR);
			setMotorTarget(motorL, motorlcode - 100, 3);
			setMotorTarget(motorR, motorrcode - 100, 3);
			loeft_klo(); // Løft klo mens robotten står stille
			setLEDColor(ledRed);
			//delay(3000);
			flaskevej++;
		}

		while (flaskevej == 3) // k6rer til den sorte streg og lægger flasken
		{
			drive(22, 30);
			//black_counter = 4;
			//delay(2000);
			driveStop();
			aaben_klo();
			setLEDColor(ledGreen);
			//delay(2000);
			flaskevej++;
		}

		while (flaskevej == 4) // kører baglæns.
		{
			drive(-22, 30);
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
			drive(10);
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
			setMotorTarget(klomotor, klo_loeft, 100);
			drive(10);
			dreje(-60);
			drive(25);
			dreje(-34);
		}
		Linefollow_PID(20); // følg linjen igen
	}
	if (black_counter == 6)
	{
		for (int i; i < 1; i++)
		{
			drive(50, 75);		   // Kør HURTIGT op over rampen
			PID_distance(20);	  // Følg rampen med PID i 40 cm
			PID_distance(30,15);
			while (SensorValue(colorsense) > (gray_val - 5)) // Indtil sensoren ser mørk, følg linjen.
			{
				Linefollow_PID(20);
			}
			driveStop();
			delay(1000);
			drive(20);			   // Kør 20 cm ned over rampen
			PID_distance(30);	  // Tænd for PID over 32 CM
			drive(18);
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
		PID_distance(40);
		dreje(-45);
		drive(26); // stod på 40 før
		while (SensorValue(colorsense) > perfect_line)
		{
			driveSpeed(-20,-20);
		}
		drive(8);
		dreje(+40);
		curr_task++;
	}
}
void task5()
{
	if (black_counter < 7)
	{
		black_counter = 7;
	}
	if (black_counter == 7)
	{
		Linefollow_PID(30); // Følg linjen
	}
	if (black_counter == 8)
	{
		for (int i; i < 1; i++)
		{
			if (klo_kalibreret == false)
			{
				driveStop();
				klo_cal(klo_aaben);
			}
			PID_distance(22);						 // Følg linjen i 20 cm
			dreje(-90);								 // Drej 90 grader mod den nye linje
			setMotorTarget(klomotor, klo_aaben, 60); // ??ben kloen
		}
		Linefollow_PID(20); // følg linjen
	}
	if (black_counter == 9)
	{
		for (int i; i < 1; i++)
		{
			drive(60, 20); //Kør frem til midten uden PID											   // reset motorencoder
			dreje(-35);
			delay(2000);
			scan(35, 35 ,25);
			resetME();										 // scan efter flaske, og peg på den
			while (ultrafilter(20) > 8.5 || ultrafilter(20) < 7) // Imens ultrasense er mellem 7.8 og 70 cm
			{
				driveSpeed(20, 20);
			}
			int motorlcode = getMotorEncoder(motorL);
			int motorrcode = getMotorEncoder(motorR);
			setMotorTarget(motorL, motorlcode - 100, 4);
			setMotorTarget(motorR, motorrcode - 100, 4);
			loeft_klo(); // Kloen lukker og løfter flasken

			while (getMotorEncoder(motorL) < 0) // Imens motorencoderen er over nulpunktet
			{
				driveSpeed(-20, -20);
			}
			drive(-22);  // Kør yderligere 20 cm tilbage
			aaben_klo(); // Kloen åbnes og flasken stilles
			drive(-50);  // Kør yderligere 20 cm tilbage
			dreje(-100);
			setMotorTarget(klomotor, klo_loeft, 100);	  // drej tilbage mod banen									    // Kør ud af skydeskive
			while (SensorValue(colorsense) > perfect_line)	 // Imens sensoren læser hvid
			{
				driveSpeed(20, 20);
			}
			drive(18);  // Kør yderligere 20 frem
			dreje(-55); // dreje tilbage på banen
			PID_distance(20);
			drive(22);
			dreje(-90);
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
		Linefollow_PID();
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
		Linefollow_PID();
	}
	if (black_counter == 11)
	{
		for (int i; i < 1; i++)
		{
			drive(42);
			dreje(-35);
			drive(16);
			clearTimer(timer3);
			while (time1[timer3] < 3000)
			{
				driveSpeed(38,22);
			}
			dreje(-50);
			drive(24);
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
		Linefollow_PID();
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

	perfect_line = gray_val + ((white_val - gray_val) / 2);     // Udregner den perfekte linje én gang i starten

	while (racedone == false)								// Main loop til at køre når race endnu ikke er færdig
	{
		black_line_counter(); // Den sorte tæller kører altid, bortset fra når anden opgave udføres

		if (curr_task == 0) // done
		{
			klo_cal();   // Kloen kalibreres som det første, så vi ved hvor den er
			color_cal(); // farvekalibrering kører i starten
			introSong(); // intro sang
			curr_task++; // fortsætter til første opgave
		}

		if (curr_task == 1) // done
		{
			task1();
		}

		if (curr_task == 2) // done
		{
			task2();
		}

		if (curr_task == 3) // done
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

		if (curr_task == 7) // done
		{
			task7();
		}

		if (curr_task == 9) // done
		{
			task9();
		}

		if (curr_task == 10) // done - celebrateion!
		{
			task10();
		}
	}
}