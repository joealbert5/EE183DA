const short int PIN = D1;
const short int b1 = D3;
const short int b2 = D2;
const short int b3 = D0;

int c = 65;
int cs = 554;
int d = 587;
int ds = 622;
int e = 659;
int f = 698;
int fs = 740;
int g = 783;
int gs = 831;
int a = 880;
int as = 932;
int b = 988;
int cc = 1046;

int song[23] = {g, a, g, f, e, f, g, d, e, f, e, f, g, g, a, f, e, f, g, d, g, e, c};
int val = 0;
int x = 0;
int y = 0;
int octave1 = 0;
int octave2 = 0;
int octave3 = 0;
int button1 = 0;
int button2 = 0;
int button3 = 0;
int buttonState = 0;
int prevState = 0;

void brassMode(int x)
{
  if (x == 0) // tuba mode (low notes, low frequencies)
  {
    c = 131;
    cs = 139;
    d = 147;
    ds = 156;
    e = 165;
    f = 175;
    fs = 185;
    g = 196;
    gs = 208;
    a = 220;
    as = 233;
    b = 247;
    cc = 262;
  }
  if (x == 1) // trumpet mode (high notes, high frequencies)
  {
    c = 523;
    cs = 554;
    d = 587;
    ds = 622;
    e = 659;
    f = 698;
    fs = 740;
    g = 783;
    gs = 831;
    a = 880;
    as = 932;
    b = 988;
    cc = 1046;
  }
}

bool buttonChange()
{ // determines if the button fingering has changed (a note change)
  if (buttonState == prevState)
    return false;
  else
    return true;
}

int buttonDecode(int three, int two, int one)
{ // decodes what note is being played based on the button fingering
  if (three == 0 && two == 1 && one == 0)
  {
    octave1 = fs;
    octave2 = b;
    return 1;
  }
  else if (three == 1 && two == 0 && one == 0)
  {
    octave1 = f;
    octave2 = as;
    return 2;
  }
  else if (three == 1 && two == 1 && one == 0)
  {
    octave1 = e;
    octave2 = a;
    return 3;
  }
  else if (three == 0 && two == 1 && one == 1)
  {
    octave1 = ds;
    octave2 = gs;
    return 4;
  }
  else if (three == 0 && two == 0 && one == 0)
  {
    octave1 = c;
    octave2 = g;
    octave3 = cc;
    return 5;
  }
  else if (three == 1 && two == 1 && one == 1)
  {
    octave1 = cs;
    octave2 = 0;
    return 6;
  }

  else if (three == 1 && two == 0 && one == 1)
  {
    octave1 = d;
    octave2 = 0;
    return 7;
  }
  else
  {
    octave1 = 0;
    octave2 = 0;
    return 8;
  }

}

void setup() {
  Serial.begin(115200);
  pinMode(PIN, OUTPUT);
  pinMode(b1, INPUT);
  pinMode(b2, INPUT);
  pinMode(b3, INPUT);
  pinMode(A0, INPUT);
  brassMode(0);
}

void loop() {
  val = analogRead(A0);
  button1 = digitalRead(b1);
  button2 = digitalRead(b2);
  button3 = digitalRead(b3);
  double v = val * 0.0049;
  Serial.println(v);
  
  // makes sure the speaker changes notes when button presses change
  prevState = buttonState;
  buttonState = buttonDecode(button3, button2, button1);
  
  // determines the embouchure of the trumpet (based on analog readings from phototransistor)
  if (v < 0.5 && v >= 0.1)
  {
    if (x == 0)
    {
      tone(PIN, octave1);
      x = 1;
    }
  }
  else if (x == 1 || buttonChange())
  {
    noTone(PIN);
    x = 0;
  }

  if (v >= 0.5 && v < 1)
  {
    if (x == 0)
    {
      tone(PIN, octave2);
      x = 2;
    }
  }
  else if (x == 2 || buttonChange())
  {
    noTone(PIN);
    x = 0;
  }

  if (v >= 1)
  {
    if (x == 0)
    {
      tone(PIN, octave3);
      x = 3;
    }
  }
  else if (x == 3 || buttonChange())
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
