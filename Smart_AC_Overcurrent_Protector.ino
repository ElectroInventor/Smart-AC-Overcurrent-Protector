#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// --- OLED ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- Encoder ---
#define CLK 2
#define DT 3
#define SW 4

volatile int encoderValue = 0;
unsigned long lastButtonPress = 0;
unsigned long lastThrBlink=0;
bool showThr=false;
bool buttonState = HIGH;

// --- Relay ---
#define RELAY_PIN 10

// --- Buzzer ---
#define BUZZER_PIN 5

// --- Biến ---
float current = 0.0;          // Dòng đo được
float threshold = 5.0;        // Ngưỡng mặc định
float thresholdWat = threshold*220;        // Ngưỡng mặc định
float defaultThreshold = 5.0;
bool trip = false;
bool setupMode = false;

// --- ADC ---
#define SENSOR_PIN A0
#define SAMPED 10
const int samplesMax = 192;
const float Vref = 5.0;    // điện áp tham chiếu ADC
const int ADCmax = 1023;   // độ phân giải ADC 10-bit
const double Nturn = 1000;
const double Voffset = 0.005f;
int samping=0;
int samples = 0;
double sumAdc=0;
int datAdc[192];
double Vrms = 0;
double Irms = 0;
double Prms = 0;

//  UI
// long press (2s) to change mode
// shortPress (<500ms) to change paramaterIdx
volatile uint8_t mode;      // 0:normal  1:set thresholdCurrnet  2:set threshold Power
volatile uint8_t paraIdx=0; // thresholdCurrent => 0:interger part  1:decima digit    / thresholdPower => 0:hàng đơn vị   1:hàng chục   2:hàng trăm   3:hàng nghìn  


unsigned long tikBuzz=0;


void setupADC() {
  ADMUX = 0b01000000;  // Thiết lập kênh A0,căn phải, tham chiếu VCC 
  ADCSRA = (1 << ADEN)  | // Bật ADC
           (1 << ADATE) | // Auto trigger (Free Running)
           (1 << ADIE)  | // Bật ngắt ADC
           (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler = 128 (tốc độ ~9.6kHz)
  ADCSRB = 0x00;     // Free running mode
  sei();              // Bật global interrupt
  ADCSRA |= (1 << ADSC); // Bắt đầu chuyển đổi đầu tiên
}

// Ngắt ADC 
ISR(ADC_vect) {
  int low = ADCL;
  int high = ADCH;
  int valAdc = (high << 8) | low;
  if(samples<samplesMax)
  {
    datAdc[samples] = valAdc;
    sumAdc+=valAdc;
  }
 samples++;
}

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT);
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  pinMode(SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CLK), readEncoder, CHANGE);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED error"));
    while (1);
  }

   // Đọc ngưỡng từ EEPROM
  EEPROM.get(0, threshold);
  thresholdWat = threshold*220;

  display.clearDisplay();
  display.display();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  delay(1000);
  digitalWrite(RELAY_PIN, HIGH);
  setupADC();
}

  
void loop() {
  if(samples >= samplesMax)
  {
    // Tính toán RMS
    double Vmean = sumAdc / samplesMax;
    double valSqrt =0;

    for(int i=0;i<samplesMax;i++)
    {
      double range = datAdc[i] - Vmean;
      valSqrt += range*range;
    }
    double Vrms_adc = valSqrt/samplesMax;
    Vrms_adc = sqrt(Vrms_adc);
    double Vrmsc = (Vrms_adc / ADCmax) * Vref;
    Vrms += Vrmsc;

    samping++;
    if(samping==SAMPED)
    {
      samping=0;
      Vrms = Vrms/SAMPED;
      Vrms = Vrms - Voffset;
      if(Vrms<0) Vrms = 0;

      Irms = (Vrms*Nturn) / 100;  
      Prms = 220*Irms;
    }
    sumAdc=0;
    samples=0;
  }
  // Kiểm tra quá dòng 
  if (Irms > threshold) 
  {
    trip = true;
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(BUZZER_PIN,HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
  } 
  if(trip==true)
  {
    if(millis() - tikBuzz > 1000){
      tikBuzz = millis();
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
    }
  }

  handleButton();
  showDisplay();
}

// --- Hàm ngắt ---
void readEncoder() {
  static uint8_t lastState = HIGH;
  uint8_t state = digitalRead(CLK);
  int adj=0;

  if (state != lastState && state == HIGH) {
    // đọc DT để xác định hướng xoay
    delay(1);
    if (digitalRead(DT) == LOW) {
      encoderValue++;
      adj=1;
    } else {
      encoderValue--;
      adj=-1;
    }
    if(mode ==1)  // threshld Current Mode
    {
      if(paraIdx==0){
        threshold+= adj;
      }else if(paraIdx==1){
        if(adj>0){
          threshold+=0.1f;
        }else{
          threshold-=0.1f;
        }
      }
      if(threshold<=0){
        threshold = 0.1;
      }else if(threshold > 9){
        threshold = 9;
      }
      thresholdWat = threshold*220;
    }else if(mode==2) // threshold Power
    {
      if(paraIdx==0){
        thresholdWat+=adj;
      }else if(paraIdx==1)
      {
        thresholdWat+=(adj*10);
      }else if(paraIdx==2){
        thresholdWat+=(adj*100);
      }else if(paraIdx==3){
        thresholdWat+=(adj*1000);
      }
      if(thresholdWat <22){
        thresholdWat =22;
      }else if(thresholdWat > 6000)
      {
        thresholdWat = 6000;
      }
      threshold = thresholdWat/220;
    }
  }
  lastState = state;
}


// --- Xử lý nút nhấn ---
uint16_t countPress =0;

void handleButton() {
  if (digitalRead(SW) == LOW) {
    countPress++;
  }else if(countPress>0){
    if(countPress >= 10) // is Long Press
    {
      Serial.println("Longpress");
      if(trip==true){
        trip=false; // gỡ bỏ cảnh báo quá dòng và bật lại relay 
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(RELAY_PIN,HIGH);
      }else{
        mode++;
        if(mode > 2){
          mode = 0;
          EEPROM.put(0, threshold); // lưu lại ngưỡng vào eeprom 
        }
        paraIdx=0;
      }
      
    }else                   // is Short Press
    {
      if(mode==1)
      {
        paraIdx++;
        if(paraIdx > 1){
          paraIdx = 0;
        }
      }else if(mode==2)
      {
        paraIdx++;
        if(paraIdx >3){
          paraIdx = 0;
        }
      }
      Serial.print("Short press:");
    }
    countPress=0;
  }
}




// --- Hiển thị OLED ---
void showDisplay() {

  display.clearDisplay();
  display.setCursor(0,0);
  display.print("I: ");
  display.print(Irms,2);
  display.print(" A");
  display.setCursor(64,0);
  display.print("P:");
  display.print(Prms,0);
  display.println(" W");
  display.setCursor(0,11);
  display.print("Thr:");
  display.setCursor(24,12);
  display.print(threshold,1);
  display.print("A / ");
  display.print(thresholdWat,0);
  display.println(" W");

  display.setCursor(0, 22);
  if(mode==0)
  {
    if (trip)
      display.println("Status: TRIP!");
    else
      display.println("Status: OK");
  }else if(mode ==1)
  {
    display.print("Set ThrCurrent...");
  }else if(mode ==2){
    display.print("Set Thr_Power...");
  }
  
  
  if(millis() - lastThrBlink >500)
  {
    lastThrBlink = millis();
    if(setupMode==true)
    {
      showThr = !showThr;
    }
    
  }
  display.display();

}
