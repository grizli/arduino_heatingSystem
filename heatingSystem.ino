#include<OneWire.h>
#include<LiquidCrystal.h>

class Output {
  // General IO pin class
  public:
  Output(int pin);
  void on();
  void off();
  bool get_state();

  private:
  int pin;
  bool is_on;
};

Output::Output(int pin) {
  pinMode(pin, OUTPUT);
  this->pin = pin;
}

void Output::on() {
  digitalWrite(this->pin, HIGH);
  this->is_on = true;
}

void Output::off() {
  digitalWrite(this->pin, LOW);
  this->is_on = false;
}

bool Output::get_state() {
  return this->is_on;
}

class Pump : public Output {
  public:
  Pump(int pin) : Output(pin) {this->off();};
};

class Boiler : public Output {
  public:
  Boiler(int pin) : Output(pin) {this->off();};
};

class StateError : public Output {
  public:
  StateError(int pin) : Output(pin) {};
};

class DS {
  public:
  DS(int pin);
  float get_temperature();
  bool is_error();

  private:
  int bottom;
  int upper;
  byte addr[8];
  void* sensor;
  bool error = false;

};

DS::DS (int pin) {
  OneWire ds1((uint8_t)pin);
  ds1.search(this->addr);
  if (OneWire::crc8(addr, 7) != addr[7]) {
    this->error = true;
  }
  this->sensor = &ds1;
};

bool DS::is_error() {
  return this->error;
}

/*
 * This method bare parts are taken from Arduino example:
 * // OneWire DS18S20, DS18B20, DS1822 Temperature Example
 * //
 * // http://www.pjrc.com/teensy/td_libs_OneWire.html
 * //
 * // The DallasTemperature library can do all this work for you!
 * // http://milesburton.com/Dallas_Temperature_Control_Library
 */
float DS::get_temperature() {
  byte data[12];
  byte type_s;
  float celsius;

  OneWire ds1 = static_cast<OneWire>(this->sensor);

  // start conversion
  ds1.reset();
  ds1.select(this->addr);
  ds1.write(0x44, 1); 

  // wait conversion;  hint: WATCHDOG!!!
  delay(1000);
  
  // the first ROM byte indicates which chip
  switch (this->addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
     // Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      //Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  
  ds1.reset();
  ds1.select(addr);
  ds1.write(0xBE);
  for (int i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds1.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;

  if (raw == 0) {
    this->error = true;
  }

  return celsius;
}

class Pots {
  public:
  Pots (int pin);
  int get_value();
  int get_temperature();
  void set_upper(int a);
  void set_bottom(int a);

  private:
  int upper;
  int bottom;
  int pin;
};

Pots::Pots (int pin) {
  this->pin = pin;
};

void Pots::set_bottom(int a) {
  this->bottom = a;
}

void Pots::set_upper(int a) {
  this->upper = a;
}

int Pots::get_value() {
  return analogRead(this->pin);
}

int Pots::get_temperature() {
  float steps = 255.0/(this->upper-this->bottom);
  return this->bottom + steps * this->get_value();
}

class PotHeating : public Pots {
  public:
  PotHeating (int pin) : Pots(pin){};
};

class PotBoiler : public Pots {
  public:
  PotBoiler (int pin) : Pots(pin){};
};

class Display {
  public:
  Display(int rs, int enable, int d4, int d5, int d6, int d7);
  set_heating_temp(float temp);
  set_heating_wanted(float temp);
  set_boiler_temp(float temp);
  set_boiler_wanted(float tmep);
  set_error(byte this_case);

  private:
  void* ptr;
};

Display::Display(int rs, int enable, int d4, int d5, int d6, int d7) {
  LiquidCrystal lcd((uint8_t)rs, (uint8_t)enable, (uint8_t)d4, (uint8_t)d5, (uint8_t)d6, (uint8_t)d7);
  lcd.begin(16,2);
  lcd.print("Starting up....");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Gr. --,- (--,-)");
  lcd.print("To. --,- (--,-)");
  this->ptr = &lcd;
}

int get_int(float a) {
  return (int)a;
}

int get_dec1(float a) {
  return (int)(a - get_int(a))*10;
}

Display::set_heating_temp(float temp) {
  LiquidCrystal lcd = *(LiquidCrystal *)this->ptr;
  lcd.setCursor(0, 4);
  lcd.print(get_int(temp));
  lcd.setCursor(0, 7);
  lcd.print(get_dec1(temp));
}
Display::set_heating_wanted(float temp) {
  LiquidCrystal lcd = *(LiquidCrystal *)this->ptr;
  lcd.setCursor(0, 10);
  lcd.print(get_int(temp));
  lcd.setCursor(0, 13);
  lcd.print(get_dec1(temp));
}

Display::set_boiler_temp(float temp) {
  LiquidCrystal lcd = *(LiquidCrystal *)this->ptr;
  lcd.setCursor(1, 4);
  lcd.print(get_int(temp));
  lcd.setCursor(1, 7);
  lcd.print(get_dec1(temp));
}

Display::set_boiler_wanted(float temp) {
  LiquidCrystal lcd = *(LiquidCrystal *)this->ptr;
  lcd.setCursor(1, 10);
  lcd.print(get_int(temp));
  lcd.setCursor(1, 13);
  lcd.print(get_dec1(temp));
}

Display::set_error(byte code) {
  LiquidCrystal lcd = *(LiquidCrystal *)this->ptr;
  lcd.clear();
  lcd.print("# E R R O R #");
  lcd.setCursor(1, 0);
  lcd.print("code: ");
  lcd.print(code);
}


void setup() {
  // put your setup code here, to run once:
  // 10 -> led & relay pump
  // 11 -> led & relay boiler
  // 12 -> led ERROR
  // A0 -> kotao temp
  // A1 -> boiler temp
  // 4 - 9 -> lcd pins

  Pump pump(10);
  Boiler boiler(11);
  StateError stateError(12);
  PotHeating potHeating(A0);
  PotBoiler potBoiler(A1);
  Display lcdDisplay(4, 5, 6, 7, 8, 9);

  float boiler_temperature = 0.0;
  float heating_temperature = 0.0;
  float boiler_ptemp = 0.0;
  float heating_ptemp = 0.0;
  const int boiler_hist = 3; //this MAY be changed
  const int heating_hist = 5; //recomended

  while(true){
    // todo add error handling!
    heating_ptemp = potHeating.get_temperature();
    boiler_ptemp = potBoiler.get_temperature();

    // todo add error handling!
    heating_temperature = potHeating.get_temperature();
    boiler_temperature = potBoiler.get_temperature();

    // todo if for error handling!
    if (true){
    }
    
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  // actually we never get here :)
}
