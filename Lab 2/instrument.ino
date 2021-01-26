const short int PIN = D1;
const short int b1 = D3;
const short int b2 = D2;
const short int b3 = D0;

int c = 523;
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

int song[192] = {b, b, b, b, b, c, 0, d,
                 0, 0, 0, g, b, c, 0, b,
                 0, g, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0,
                 b, b, b, b, b, c, 0, d,
                 0, 0, 0, g, b, c, 0, b,
                 0, g, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0,
                 f, f, f, f, f, 0, 0, 0,
                 f, f, f, f, f, 0, 0, 0,
                 f, g, g, g, g, g, g, g,
                 0, 0, 0, 0, 0, 0, 0, 0,
                 b, b, b, b, b, c, 0, d,
                 0, 0, 0, g, b, c, 0, b,
                 0, g, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0,
                 b, b, b, b, b, c, 0, d,
                 0, 0, 0, g, b, c, 0, b,
                 0, g, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0,
                 f, f, f, f, f, 0, 0, 0,
                 f, f, f, f, f, 0, 0, 0,
                 f, g, g, g, g, g, g, g,
                 0, 0, 0, 0, 0, 0, 0, 0
                };
int val = 0;
int x = 0;
int y = 0;
int octave1 = 0;
int octave2 = 0;
int octave3 = 0;
int button1 = 0;
int button2 = 0;
int button3 = 0;
int brassState = 0;
int buttonState = 0;
int prevState = 0;
int start = 0;

void brassMode(int x)
{
  if (x == 1)
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
  if (x == 0)
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
{
  if (buttonState == prevState)
    return false;
  else
    return true;
}

int buttonDecode(int three, int two, int one)
{

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
  pinMode(D4, INPUT);
  pinMode(D5, INPUT);
}

void loop() {
  val = analogRead(A0);
  button1 = digitalRead(b1);
  button2 = digitalRead(b2);
  button3 = digitalRead(b3);
  brassState = digitalRead(D4);
  double v = val * 0.0049;

  start = digitalRead(D5);

  if (start == 0) // user control mode
  {
    brassMode(brassState);

    prevState = buttonState;
    buttonState = buttonDecode(button3, button2, button1);

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

    if (v >= 0.5 && v < 0.9)
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

    if (v >= 0.9)
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
  }


  if (start == 1) // song mode
  {
    int tempo = 114;
    // play preprogrammed song array
    int i;
    for (i = 0; i < sizeof(song); i++)
    {
      tone(PIN, song[i]);
      delay(tempo);
      noTone(PIN);
      delay(tempo);
    }
  }

}
