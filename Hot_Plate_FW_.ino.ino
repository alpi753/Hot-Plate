#include <Wire.h>
#include <LiquidCrystal.h>
  
// PINS (kartına göre uyarlayabilirsin)
#define PIN_DB4    PA0
#define PIN_E      PA1
#define PIN_RS     PA2
#define PIN_DB5    PA3 
#define PIN_DB6    PA4 
#define PIN_DB7    PA5
#define PIN_SSR    PA6
#define PIN_RTD    PA7
#define PIN_OUTA   PB0 // encoder A (interrupt)
#define PIN_OUTB   PB1 // encoder B (interrupt)
#define PIN_BUTTON PB10 // encoder push button (input pullup)
#define PIN_FAN    PB11 // fan digital output (PWM tercih edilebilir)

// PARAMETERS
#define PWM_FREQUENCY     200000  // [us]  200000us = 5 Hz (kullanılmıyor, referans)
#define MIN_TEMP               0  // [C]
#define MAX_TEMP             400  // [C]
#define REF_STEP            0.05  // [C]
#define MAX_DUTY            1023  // [%]
#define MIN_DUTY               0  // [%]
#define SAFETY_PERIOD      60000  // [ms]
#define SAFETY_THRESHOLD       1  // [C]
#define TURNOFF_TIME       15000  // [ms]
#define COOL_TEMPERATURE      25  // [C]

// RTD lookup (kopyalandı, küçük bir hatalı satır olabilir, istersen düzelt)
// RTD_TABLE_SIZE computed automatically
typedef struct {
    float temp;   // sıcaklık (°C)
    float r;      // direnç (ohm)
} RTD_LUT;

const RTD_LUT rtd_table[] = {
    {-50.0, 80.31},
    {-40.0, 84.27},
    {-30.0, 88.22},
    {-20.0, 92.16},
    {-10.0, 96.09},
    {  0.0, 100.00},
    { 10.0, 103.90},
    { 20.0, 107.79},
    { 30.0, 111.67},
    { 40.0, 115.54},
    { 50.0, 119.40},
    { 60.0, 123.24},
    { 70.0, 127.08},
    { 80.0, 130.89},
    { 90.0, 134.70},
    {100.0, 138.50},
    {110.0, 142.29},
    {120.0, 146.07},
    {130.0, 149.83},
    {140.0, 153.58},
    {150.0, 157.33},
    {160.0, 161.05},
    {170.0, 164.77},
    {180.0, 168.48},
    {190.0, 172.17},
    {200.0, 175.86},
    {210.0, 179.53},
    {220.0, 183.19},
    {230.0, 186.84},
    {240.0, 190.47},
    {250.0, 194.10},
    {260.0, 197.71},
    {270.0, 201.31},
    {280.0, 204.90},
    {290.0, 208.48},
    {300.0, 212.05},
    {310.0, 215.61},
    {320.0, 219.15},
    {330.0, 222.68},
    {340.0, 115.54}, // <-- orijinal girdi; burayı kontrol etmek isteyebilirsin
    {350.0, 226.21},
    {360.0, 233.21},
    {370.0, 236.70},
    {380.0, 240.18},
    {390.0, 243.64},
    {400.0, 247.09},
    {410.0, 250.53},
    {420.0, 253.96},
    {430.0, 257.38},
    {440.0, 260.78},
    {450.0, 264.18},
    {460.0, 267.56},
    {470.0, 270.93},
    {480.0, 274.29},
    {490.0, 277.64},
    {500.0, 280.98}
};
const int RTD_TABLE_SIZE = sizeof(rtd_table) / sizeof(RTD_LUT);

// PID/PI params
float duty = 0.00;
float kp = 1.5;
float ki = 0.001;
volatile float acc = 0.00;
volatile float error = 0.00;

// Temperature Variables
volatile int    set_temp            = 40;      // hedef (kullanıcı arayüzü)
volatile float  reference_temp      = 40.0;
volatile float  measured_temp       = 0.00;
volatile float  measured_resistance = 0.00;
volatile int    adc_raw             = 0;
volatile float  voltage             = 0;
volatile int    temp_reading        = 0;
volatile int    last_measured_temp  = 0;
volatile int    last_turnoff_check  = 0;

// Heater Variables
bool heater = false;
unsigned long heater_turned_on = 0;
bool finished = false;

// Safety Variables
float last_safety_temp  = 0.00;
unsigned long last_safety_check = 0;
bool error_state = false;

// Turnoff variables
bool counter_flag = true;
bool closing_sekans = false;

//LCD nesnesi tanımlanması
LiquidCrystal lcd ( PIN_RS, PIN_E, PIN_DB4, PIN_DB5, PIN_DB6, PIN_DB7 );
bool flag_set_temp = false; // for blinking "set" word

// Enkoder variables
volatile long encoderCount = 0; // raw tick sayısı (interrupt içinde güncellenir)
volatile int encoderPositionDelta = 0; // loop'ta kullanılmak üzere
volatile int lastEncoded = 0;

// Button (debounce & press detection)
bool inSetMode = false;                // uzun basma ile girilen ayar modu
bool buttonPressedDebounced = false;
unsigned long buttonPressTime = 0;
unsigned long buttonReleaseTime = 0;
const unsigned long longPressDuration = 1000; // uzun basma için ms (1s)
const unsigned long debounceDelay = 30; // ms
bool lastButtonLogical = false;
unsigned long lastDebounceTime = 0;
bool longPressTriggered = false;

// Forward declarations
int RTD_GetTemperature();
void ToggleHeater();
bool RegulatorHandler();
void Safety_Check ();
void Serial_Write();
void lcd_Display();
void lcd_Display_set_temp();
void TurnOff_Check();
void cooling_down_check();
void readButtonAndHandle();
void enterSetMode();
void exitSetMode();
void updateSetTempFromEncoder();
void encoderA_ISR();
void encoderB_ISR();


// --------------------- RTD hesaplama ---------------------
int RTD_GetTemperature()
{
    // ADC okuma (kartına göre analogRead çözünürlüğü 12-bit varsayılıyor)
    adc_raw = analogRead(PIN_RTD);
    voltage = (adc_raw * 3.3) / 4095.0 ; 
    measured_resistance = (voltage * 4400.0) / (33.0 - voltage);
   
    if (measured_resistance <= rtd_table[0].r)
        return int(rtd_table[0].temp);
    if (measured_resistance >= rtd_table[RTD_TABLE_SIZE - 1].r)
        return int(rtd_table[RTD_TABLE_SIZE - 1].temp);

    for (int i = 0; i < RTD_TABLE_SIZE - 1; i++) {
        if (measured_resistance >= rtd_table[i].r && measured_resistance <= rtd_table[i + 1].r) {
            float t1 = rtd_table[i].temp;
            float t2 = rtd_table[i + 1].temp;
            float r1 = rtd_table[i].r;
            float r2 = rtd_table[i + 1].r;
            measured_temp = t1 + (measured_resistance - r1) * (t2 - t1) / (r2 - r1);
            return int (measured_temp); // lineer interpolasyon
        }
    }
    return 0; // güvenlik için
}


// --------------------- Isıtıcı on/off ---------------------
void ToggleHeater()
{
  if (!heater)
  {
    Serial.println("Heater turned ON");
    heater_turned_on = millis();
    last_safety_check = millis();
    heater = true;
    counter_flag = true;
    finished = false;
    closing_sekans = false;
  }
  else
  {
    Serial.println("Heater turned OFF");
    heater = false;
    // Kapanış sekansını başlat
    last_turnoff_check = millis(); // istersen burada kaydet
  }
  return;
}


// --------------------- Regulator (PI) ---------------------
bool RegulatorHandler()
{
  // Measure the temperature
  temp_reading = RTD_GetTemperature();
  measured_temp = temp_reading;

  if (heater == true && error_state == false )
  {
    // Updating the reference temperature kademeli artırma / düşürme
    if (set_temp > reference_temp) reference_temp += REF_STEP;
    if (set_temp < reference_temp) reference_temp -= REF_STEP;
    if (abs(set_temp - reference_temp) < REF_STEP) reference_temp = set_temp;

    // PI controller
    error = reference_temp - measured_temp;
    acc += error;

    // Calculate the duty cycle for the PWM (ölçeklendirme orijinalden alındı)
    duty = kp * error + ki * acc;

    // Reset the safety check timer if the duty cycle is at 0%
    if (duty <= MIN_DUTY) last_safety_check = millis();

    // duty normalizasyon. scaling sabitini istersen ayarla
    duty = duty * 51.15;

    // Overshoot regulation
    if (duty > MAX_DUTY)
    {
      duty = MAX_DUTY;
      acc -= error;
    }

    // Undershoot regulation
    if (duty < MIN_DUTY)
    {
      duty = MIN_DUTY;
      acc -= error;
    }

    // Update the PWM (kartına göre PWM fonksiyonunu aç)
     analogWrite(PIN_SSR, map((int)duty, 0, MAX_DUTY, 0, 255)); // örnek
  }
  else if (error_state == false)
  {
    // Idle
    acc = 0;
    duty = 0;
    last_safety_check = millis();
  }
  return true;
}

// --------------------- Safety check ---------------------
void Safety_Check (){
    if (millis() - last_safety_check > SAFETY_PERIOD)
    {
      last_safety_check = millis();

      if (measured_temp - last_safety_temp < SAFETY_THRESHOLD)
      {
        // sıcaklık artışı yok -> hata
        error_state = true;
        duty = 0.00;
        acc = 0;
      }
      last_safety_temp = measured_temp;
    }
}

// --------------------- Serial debug ---------------------
void Serial_Write(){

  Serial.print("ADC: ");
  Serial.print(adc_raw);
  Serial.print("  V: ");
  Serial.print(voltage, 3);
  Serial.print("  Rrtd: ");
  Serial.print(measured_resistance, 4);
  Serial.print("  Temp: ");
  Serial.print(temp_reading);
  Serial.print("  Set: ");
  Serial.print(set_temp);
  Serial.print("  Ref: ");
  Serial.print(reference_temp,2);
  Serial.print("  Duty: ");
  Serial.print(duty, 2);
  Serial.print("  ERR: ");
  Serial.print(error, 4);
  if (error_state)
    Serial.print("  RTD ERROR");
  Serial.print("  ENCdelta:");
  Serial.println(encoderPositionDelta);
}

// --------------------- LCD display (non-blocking friendly) ---------------------
unsigned long lastLcdToggle = 0;
void lcd_Display(){
  // Basit, bloklamayan bir LCD güncellemesi
  lcd.clear();
  delay(1);
  lcd.setCursor(0, 0);
  delay(1);
  lcd.print("Sic:");
  delay(1);
  if (temp_reading < 100){
    lcd.setCursor(6,0);
  } else {
    lcd.setCursor(5,0);
  }
  lcd.print(temp_reading);
  delay(1);
  lcd.print((char)223); // derece sembolü (LCD karakter setine göre değişebilir)
  delay(1);
  lcd.setCursor(9, 0); 
  lcd.print("Set:");
  delay(1);
  if(set_temp < 100) lcd.setCursor(14,0); else lcd.setCursor(13,0);
  lcd.print(set_temp);
  delay(1);

  lcd.setCursor(0, 1);
  lcd.print("MODE:");
  delay(1);
  lcd.setCursor(6, 1);


  if(error_state == true && closing_sekans == false){
      lcd.print("RTD ERROR!");
      delay(1);
  } 
  else if (heater == true){
      lcd.print("HEAT");
      delay(1);
  }
  else if( heater == false && closing_sekans == true && finished == false ){
      lcd.print("COOLING!");
      delay(1);
  }
  else if(heater == false && finished == true){
      lcd.print("WAIT");
      delay(1);
  }
}

// Blinking set gösterimi için
void lcd_Display_set_temp(){
  // toggle flag her çağrıda yapılabilir veya zamanla değiştirilebilir
  flag_set_temp = !flag_set_temp;

  //lcd.clear();
  delay(1);
  lcd.setCursor(0, 0);
  lcd.print("Sic:");
  delay(1);
  if (temp_reading < 100) lcd.setCursor(6,0); else lcd.setCursor(5,0);
  lcd.print(temp_reading);
  delay(1);
  lcd.print((char)223);
  delay(1);

  lcd.setCursor(9, 0);
  delay(1);
  lcd.print("Set:");
  delay(1);
  if(flag_set_temp){
    if(set_temp < 100) lcd.setCursor(14,0); else lcd.setCursor(13,0);
    delay(1);
    lcd.print(set_temp);
    delay(70);
  } else {
    if(set_temp < 100) lcd.setCursor(14,0); else lcd.setCursor(13,0);
    lcd.print("   "); // boşluklarla sil
    delay(70);
  }

  lcd.setCursor(0, 1);
  lcd.print("MODE:");
  delay(1);
  lcd.setCursor(6, 1);
  if(error_state == true && closing_sekans == false){
      lcd.print("RTD ERROR!");
      delay(1);
  } 
  else if (heater == true){
      lcd.print("HEAT");
      delay(1);
  }
  else if( heater == false && closing_sekans == true && finished == false ){
      lcd.print("COOLING!");
      delay(1);
  }
  else if(heater == false && finished == true){
      lcd.print("WAIT");
      delay(1);
  }
}

// --------------------- TurnOff_Check ---------------------
void TurnOff_Check(){
  // starting counting the time for turnoff
  if ( (set_temp - temp_reading < 2) && counter_flag ) {
    last_turnoff_check = millis();        // starting counting the time for turnoff
    counter_flag = false;                 // bir kez başlat
    heater = false;                       // Isıtıcıyı kapat
    analogWrite(PIN_SSR, 0);              // Sogumaya geciste ısıtıcıyı sıfırla  
          
  }

  // turn off the heater after TURNOFF_TIME
  if( !counter_flag && (millis() - last_turnoff_check > TURNOFF_TIME) ) {
    heater = false;
    closing_sekans = true;
    counter_flag = true;
    // last_turnoff_check = millis(); // isteğe bağlı
  }
}

// --------------------- Set temperature mode ---------------------
void enterSetMode(){
  inSetMode = true;
  // sıfırla encoder delta
  noInterrupts();
  encoderPositionDelta = 0;
  interrupts();
  Serial.println(">>> Entering SET MODE");
}

void exitSetMode(){
  inSetMode = false;
  Serial.println(">>> Exiting SET MODE");
  // reference temp'i hedefle eşitle (kademeli artış yine referans ile yapılacak)
  // set_temp zaten ayarlandı
}

// Encoder'den gelen delta'yı set_temp'e uygula (loop içinde çağrılacak)
void updateSetTempFromEncoder(){
  // atomik okuma
  noInterrupts();
  int delta = encoderPositionDelta;
  encoderPositionDelta = 0;
  interrupts();

  if (delta != 0) {
    set_temp += delta; // birim olarak 1 derece / tick
    if (set_temp < MIN_TEMP) set_temp = MIN_TEMP;
    if (set_temp > MAX_TEMP) set_temp = MAX_TEMP;
    Serial.print("SET TEMP updated: ");
    Serial.println(set_temp);
  }
}

// --------------------- Encoder ISRs ---------------------
// Basit quadrature encoder okuma
void encoderA_ISR(){
  int MSB = digitalRead(PIN_OUTA); // A
  int LSB = digitalRead(PIN_OUTB); // B

  int encoded = (MSB << 1) | LSB;
  int sum  = (lastEncoded << 2) | encoded;

  // quadrature tablosu
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    // +1
    encoderPositionDelta++;
    encoderCount++;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    // -1
    encoderPositionDelta--;
    encoderCount--;
  }
  lastEncoded = encoded;
}

void encoderB_ISR(){
  // sadece A ISR yeterli ama B de tetiklensin, aynı mantık
  encoderA_ISR();
}

// --------------------- Cooling / fan check ---------------------
void cooling_down_check(){
  if( closing_sekans == true){
    if(COOL_TEMPERATURE < temp_reading ){
      digitalWrite(PIN_FAN, HIGH);
    } 
    else{
      digitalWrite(PIN_FAN, LOW);
      finished = true;
      closing_sekans = false;
    } 
  }
}

// --------------------- Buton okuma & short/long press logic (debounced) ---------------------
void readButtonAndHandle() {
  // INPUT_PULLUP => pressed when LOW
  bool raw = !digitalRead(PIN_BUTTON);  // lojik: true = basılı
  unsigned long now = millis();

  // --- Debounce kontrolü ---
  if (raw != lastButtonLogical) {
    lastDebounceTime = now;
  }

  if ((now - lastDebounceTime) > debounceDelay) {
    // Stabil durum
    // Değişiklik olduysa
    if (raw != buttonPressedDebounced) {
      buttonPressedDebounced = raw;

      if (buttonPressedDebounced) {
        // ----------------- BUTON BASILDI -----------------
        buttonPressTime = now;
        longPressTriggered = false;   // uzun basmayı yeniden aktif edilebilir yap
      }
      else {
        // ----------------- BUTON BIRAKILDI -----------------
        unsigned long pressedDuration = now - buttonPressTime;

        // Eğer uzun basma ZATEN tetiklendiyse (basılıyken), 
        // kısa basma veya yeniden uzun basma yapılmaz.
        if (longPressTriggered) {
          // Hiçbir şey yapma: uzun basma zaten aktif edilmişti
        }
        else {
          // Uzun basma süresi dolmadıysa → kısa basma
          if (pressedDuration < longPressDuration) {
            ToggleHeater();
          }
          // Süre dolmuş ama longPressTriggered == false ise (nadiren), yine set mode aç
          else {
            if (!inSetMode) enterSetMode();
            else exitSetMode();
          }
        }
      }
    } 
    else {
      // ----------------- BUTON HALA BASILI -----------------
      if (buttonPressedDebounced && !longPressTriggered) {
        if (now - buttonPressTime >= longPressDuration) {
          // Uzun basma SÜRE DOLUNCA anında gerçekleşsin
          longPressTriggered = true;

          if (!inSetMode) enterSetMode();
          else exitSetMode();
        }
      }
    }
  }

  lastButtonLogical = raw;
}

// --------------------- Setup / Loop ---------------------
void setup() {
 
  Serial.begin(115200);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_OUTB, INPUT_PULLUP);
  pinMode(PIN_OUTA, INPUT_PULLUP);
  pinMode(PIN_RTD, INPUT);
  pinMode(PIN_SSR, OUTPUT); 
  pinMode(PIN_DB4, OUTPUT);
  pinMode(PIN_DB5, OUTPUT);
  pinMode(PIN_DB6, OUTPUT);
  pinMode(PIN_DB7, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);

  set_temp = 40;
  reference_temp = 40;
  delay(1);
  lcd.begin(16,2);
  delay(1);
  lcd.setCursor(0,0);
  delay(1);
  lcd.print("Omkon Teknoloji");
  delay(1);
  lcd.setCursor(0,1);
  delay(1);
  lcd.print("PCB Heater v1.0");
  delay(3000);
  lcd.clear();
  delay(1);

  // Encoder interrupts (kartına uygun pin olmalı)
  attachInterrupt(digitalPinToInterrupt(PIN_OUTA), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_OUTB), encoderB_ISR, CHANGE);

  // Başlangıç zamanlayıcılar
  last_safety_check = millis();
  last_turnoff_check = millis();
  lastLcdToggle = millis();
  
}

void loop() {

  // RTD ölçümü/safety/regulator
  if( abs(reference_temp - measured_temp)  > 10 ){
    Safety_Check();
  }

  TurnOff_Check();

  RegulatorHandler();

  Serial_Write();

  // Buton işleme (debounce + short / long press)
  readButtonAndHandle();

  // Eğer set mode aktifse, encoder ile ayar yap ve blinkli lcd gösterimi
  if (inSetMode) {
    updateSetTempFromEncoder();
    lcd_Display_set_temp();
  } else {
    // Normal lcd gösterimi
    lcd_Display();
  }

  // Soğutma sekansı kontrolü
  cooling_down_check();

  // Döngü sonunda kısa bekle (uygun bir değer)
  delay(100);
}




