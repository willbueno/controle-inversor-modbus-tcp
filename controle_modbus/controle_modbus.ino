#define WIZNET_W5100 1

#include <Ethernet.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

IPAddress ModbusDeviceIP(192, 168, 0, 244);  // Put IP Address of PLC here
IPAddress moduleIPAddress(192, 168, 0, 243);  // Assign Anything other than the PLC IP Address
                                              
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xE1 };


#include <ModbusTCP.h>

ModbusTCP node(255);                            // Unit Identifier.

int pinBotao_liga = 21; //Declara pino do botao liga
int pinBotao_emergencia = 20; //Declara pino do botao emergencia
int pinPotenciometro = A15; //Declara pino potenciometro
bool ligado = false;
float kp = 31.2;
float ki = 0.167;
float kd = 0;
double loopTime;
double lastProcess;


unsigned int f_2uint_int1(float float_number) {             // split the float and return first unsigned integer

  union f_2uint {
    float f;
    uint16_t i[2];
  };

  union f_2uint f_number;
  f_number.f = float_number;

  return f_number.i[0];
}

unsigned int f_2uint_int2(float float_number) {            // split the float and return second unsigned integer

  union f_2uint {
    float f;
    uint16_t i[2];
  };

  union f_2uint f_number;
  f_number.f = float_number;

  return f_number.i[1];
}

float f_2uint_float(unsigned int uint1, unsigned int uint2) {    // reconstruct the float from 2 unsigned integers

  union f_2uint {
    float f;
    uint16_t i[2];
  };

  union f_2uint f_number;
  f_number.i[0] = uint1;
  f_number.i[1] = uint2;

  return f_number.f;

}

void setup()
{
  Serial.begin(9600);
  
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MF OFF");
  
  pinMode(pinPotenciometro, INPUT); //potenciometro
  pinMode(pinBotao_liga,INPUT); //botao digital
  pinMode(pinBotao_emergencia,INPUT); //botao digital
  
  delay(1000);
  
  Ethernet.begin(mac, moduleIPAddress);
  node.setServerIPAddress(ModbusDeviceIP);
  
  delay(1000);                                // To provide sufficient time to initialize.
}


void loop()
{
  int botao_liga = digitalRead(pinBotao_liga);
  int botao_emergencia = digitalRead(pinBotao_emergencia);
  float potenciometro = analogRead(pinPotenciometro);

  float setPoint = map(potenciometro, 0, 1023, 0, 100);
  //Serial.println(setPoint);
    
  if(botao_liga == 0)
  {
    //uint8_t result;
    node.writeSingleRegister(9100, 1);                  // Write single register
    //Serial.println(result, HEX);
    ligado = true;
    //delay(500);
    lcd.setCursor(3,0);
    lcd.print("ON ");
  }
  
  if(botao_emergencia == 0)
  {
    //uint8_t result;
    node.writeSingleRegister(9100, 2);                  // Write single register
    //Serial.println(result, HEX);
    ligado = false;
    //delay(500);
    lcd.setCursor(3,0);
    lcd.print("OFF");
  }
  
  float nivel;
  float pid;
  
  if(ligado == true)
  {
      uint8_t result;
      result = node.readHoldingRegisters(52000, 2);    // Read Holding Registers
      unsigned int aux_nivel1,aux_nivel2;
      aux_nivel1 = node.getResponseBuffer(0);
      aux_nivel2 = node.getResponseBuffer(1);
      nivel = f_2uint_float(aux_nivel1, aux_nivel2);
      node.clearResponseBuffer();

      unsigned int aux_setPoint1,aux_setPoint2;
      aux_setPoint1 = f_2uint_int1(setPoint);
      aux_setPoint2 = f_2uint_int2(setPoint);
      node.writeSingleRegister(52402, aux_setPoint1);                  // Write single register
      node.writeSingleRegister(52403, aux_setPoint2);                  // Write single register

      //PID
      float error;
      error = setPoint - nivel;
      loopTime = millis();
      float deltaTime = (loopTime-lastProcess)/1000.0;
      lastProcess = millis();

      float P, I;
    
      //Proporcional
      P = error*kp;

      //Integral
      I = I+(error*ki)*deltaTime;


      pid = P+I;

      if(pid >= 100)
      {
        pid = 100;
      }
      else if (pid <= 0)
      {
        pid = 0;
      }
       
      unsigned int aux_pid1,aux_pid2;
      aux_pid1 = f_2uint_int1(pid);
      aux_pid2 = f_2uint_int2(pid);
      node.writeSingleRegister(52400, aux_pid1);                  // Write single register
      node.writeSingleRegister(52401, aux_pid2);                  // Write single register

      
      
  
    }

    lcd.setCursor(8, 0);
    lcd.print("SP:");
    lcd.print(setPoint);

    lcd.setCursor(0, 1);
    lcd.print("PV:");
    lcd.print(nivel); 
      
    lcd.setCursor(7, 1);
    lcd.print(" MV:");
    lcd.print(pid);  

}
