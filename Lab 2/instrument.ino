const short int PIN = D1;
const short int b1 = D3;
const short int b2 = D2;
const short int b3 = D0;
const int c = 523;
const int cs = 554;
const int d = 587;
const int ds = 622;
const int e = 659;
const int f = 698;
const int fs = 740;
const int g = 783;
const int gs = 831;
const int a = 880;
const int as = 932;
const int b = 988;

int song[23] = {g,a,g,f,e,f,g,d,e,f,e,f,g,g,a,f,e,f,g,d,g,e,c};
int button1 = 0;
int button2 = 0;
int button3 = 0;

void setup() {
Serial.begin(115200);
pinMode(PIN,OUTPUT);
pinMode(b1,INPUT);
pinMode(b2,INPUT);
pinMode(b3,INPUT);
pinMode(A0,INPUT);
}

int val = 0;
int x = 0;
int y = 0;
int octave1 = 0;
int octave2 = 0;

void loop() {
val = analogRead(A0);
button1 = digitalRead(b1);
button2 = digitalRead(b2);
button3 = digitalRead(b3);
double v = val*0.0049;
Serial.println(v);

/*
if (button3 == 0 && button2 == 1 && button1 == 0)
{
  octave1 = fs;
  octave2 = b;
}
else if (button3 == 1 && button2 == 0 && button1 == 0)
{
  octave1 = f;
  octave2 = as;
}
else if (button3 == 1 && button2 == 1 && button1 == 0)
{
  octave1 = e;
  octave2 = a;
}
else if (button3 == 0 && button2 == 1 && button1 == 1)
{
  octave1 = ds;
  octave2 = gs;
}
else if (button3 == 0 && button2 == 0 && button1 == 0)
{
  octave1 = c;
  octave2 = g;
}
else if (button3 == 1 && button2 == 1 && button1 == 1)
{
  octave1 = cs;
  octave2 = 0;
}

else if (button3 == 1 && button2 == 0 && button1 == 1)
{
  octave1 = d;
  octave2 = 0;
}
else
{
  octave1 = 0;
  octave2 = 0;
}
*/

if(v < 0.5)
{
  if(x == 0)
  {
    tone(PIN,f);
    x = 1;
  }
}
else if (x == 1)
{
  noTone(PIN);
  x = 0;
}

if(v >= 0.5)
{
  if(x == 0)
  {
  tone(PIN,c);
  x = 2;
  }
}
else if (x == 2)
{
  noTone(PIN);
  x = 0;
}


/*
// play preprogrammed song array
int i;
for(i = 0;i < sizeof(song);i++)
{
tone(PIN,song[i]);
delay(100);
noTone(PIN);
delay(100);
}
*/

}
