/*
  System Start Motorcycle ESP32 (BLE) by Tsim lis
  --Control Relay 3v
    pin 3.3v, Gnd, 1, 3, 22, 23
  --Notification buzzer pin 21, Gnd
  --Voltage Meter system
     pin 36, 39
  --Meter TPs
     pin 27, pin out DAC 25 -> Amp
     -Touch select Reset ESP32
       pin 0
  --O2 Meter
     pin 34
  --Remote control off & pin touch on notification
     pin 3.3v, 35 ; 33, 32
  --Original TPs Pin 15
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


// Digital pin
//int RELAY_PINS[] = {23, 22, 1, 3, 21, 17, 0};
#define ReOpen 23
#define ReStart 22
#define ReClose 1
#define ReBuzzer  3
#define Buzzer 21
#define ResetESP 17
#define EspOffBuzzer 0
#define Tps_Ori 15

bool Pinmode () {
  pinMode (ReOpen, OUTPUT);
  pinMode (ReStart, OUTPUT);
  pinMode (ReClose, OUTPUT);
  pinMode (ReBuzzer, OUTPUT);
  pinMode (Buzzer, OUTPUT);
  pinMode (ResetESP, OUTPUT);
  pinMode (EspOffBuzzer, OUTPUT);
  pinMode (Tps_Ori, OUTPUT);
}

// Analog pin
//int Analog_Pin[] = {36, 39, 34, 35, 27, 4};
#define VoutSys 36
#define VinSys 39
#define O2 34
#define RemoteOffBuzzer 35
#define TPSin 27
#define DAC1 25
#define RPMpin 16
#define CheckDac25 4
#define TouchPin1 32
#define TouchPin2 33


// UUID
BLECharacteristic *pCharacteristic;

bool deviceConnected = false;
String filterString;
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID            "4FAFC201-1FB5-459E-8FCC-C5C9C331914B" // UART service UUID
#define CHARACTERISTIC_UUID     "BEB5483E-36E1-4688-B7F5-EA07361B26A8" //Use this characteristic for all JG

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Client Connected");
      digitalWrite(Buzzer, 1);
      Serial.print(" 1,"); //
      delay(250);
      digitalWrite(Buzzer, 0);
      Serial.print(" 0,"); //
      delay(250);
      digitalWrite(Buzzer, 1);
      Serial.println(" 1,"); //
      delay(250);
    }
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Client Disconnected");
      digitalWrite(Buzzer, 1);
      Serial.print(" 1,"); //
      delay(1000);
      digitalWrite(Buzzer, 0);
      Serial.print(" 0,"); //
      delay(500);
      digitalWrite(Buzzer, 1);
      Serial.print(" 1,"); //
      delay(250);
      digitalWrite(Buzzer, 0);
      Serial.print(" 0,"); //
      delay(250);
      digitalWrite(Buzzer, 1);
      Serial.println(" 1,"); //
      delay(250);
    }

};

char modeTps;
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      String BLEvalue = rxValue.c_str();

      if (BLEvalue == "rOpen_1") {
        digitalWrite(ReOpen, 1);
        Serial.println("ระบบไฟ-ติด");
      }
      else if (BLEvalue == "rOpen_0") {
        digitalWrite(ReOpen, 0);
        Serial.println("ระบบไฟ-ดับ");
      }
      else if (BLEvalue == "rStart_1") {
        digitalWrite(ReStart, 1);
        Serial.println("ไดร์สตาร์ท-ทำงาน");
      }
      else if (BLEvalue == "rStart_0") {
        digitalWrite(ReStart, 0);
        Serial.println("ไดร์สตาร์ท-หยุด");
      }
      else if (BLEvalue == "rClose_1") {
        digitalWrite(ReClose, 1);
        Serial.println("ปิดระบบ");
      }
      else if (BLEvalue == "rClose_0") {
        digitalWrite(ReClose, 0);
        Serial.println("ปิดระบบสำเร็จ");
      }
      else if (BLEvalue == "rAlarm_1") {
        Serial.println("เปิด-สัญญาณกันขโมย");
      }
      else if (BLEvalue == "rAlarm_0") {
        digitalWrite(EspOffBuzzer, 1);
        digitalWrite(ReBuzzer, 0);
        Serial.println("ปิด-สัญญาณกันขโมย");
      }
      else if (BLEvalue == "rReset_1") {
        digitalWrite(ResetESP, 1);
        Serial.println("รีเซต-เปิด");
      }
      else if (BLEvalue == "rReset_0") {
        digitalWrite(ResetESP, 0);
        Serial.println("รีเซต-ปิด");
      }
      else if (BLEvalue == "A") {
        modeTps = 'A';
        digitalWrite(Tps_Ori , LOW);
        Serial.println("Mode_A");
      }
      else if (BLEvalue == "B") {
        modeTps = 'B';
        digitalWrite(Tps_Ori , LOW);
        Serial.println("Mode_B");
      }
      else if (BLEvalue == "C") {
        modeTps = 'C';
        digitalWrite(Tps_Ori , LOW);
        Serial.println("Mode_C");
      }
      else if (BLEvalue == "N") {
        modeTps = 'N';
        digitalWrite(Tps_Ori , LOW);
        Serial.println("O2_Mode_On");
      }
      else if (BLEvalue == "F") {
        modeTps = 'F';
        digitalWrite(Tps_Ori , LOW);
        Serial.println("O2_Mode_off");
      }
      else if (BLEvalue == "O") {
        modeTps = 'O';
        digitalWrite(Tps_Ori , HIGH);
        Serial.println("TPS_Original");
      }
    }
};


/////////////////////single UUID---------function Create BLEserver//////////////
bool initBLE() {
  //---- -Setup Buletooth---- - /
  // Create the BLE Device
  BLEDevice::init("ESP32_Tsim");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
  ///
  //BLEAdvertising *pAdvertising = pServer->getAdvertising();
  //pAdvertising->start();
  ///
  return true;
}


//------------------------------------------------------------//


///////////////////////////////////  Voltage system I/O //
int VinSys_value = 0, VoutSys_value = 0;

String Volt_Sys() {
  float Vsys_In = 0, Vsys_Out = 0;

  VinSys_value = analogRead(VinSys);            // Volt max 15v. // Over limit 3.3 input eap32
  Vsys_In = (VinSys_value * 15.0) / 4095;   // analogRead have 12 bit
  VoutSys_value = analogRead(VoutSys);           // (+) -> R1->Vuse->R2-> (-)
  Vsys_Out = (VoutSys_value * 15.0) / 4095; // R1=3500 ohm, R2=1000 ohm

  Serial.print("Volt In = ");
  Serial.print(Vsys_In);
  Serial.print("\tOut = ");
  Serial.print(Vsys_Out);
  Serial.print("\t\tValue_out : ");
  Serial.println(VoutSys_value);

  String mixSys = String(Vsys_In) + "," + String(Vsys_Out);
  return mixSys;
}


//////////////////////////// Voltage meter Oxygen sensor //
float Vin_O2 = 0.0;

String Volt_O2() {
  int value_O2 = 0;
  value_O2 = analogRead(O2);        // 5v from O2 in Over limit 3,3v input eap32
  Vin_O2 = (value_O2 * 6.0) / 4095; // R1=1000 ohm, R2=1000 ohm

  // AFR point (y = mx + b)
  float AF = 0, b = 10;
  AF = (2 * Vin_O2) + b;

  Serial.print("Volt O2 = ");
  Serial.print(Vin_O2);
  Serial.print("\tAF = ");
  Serial.print(AF);
  Serial.print("\t\tValue_O2 : ");
  Serial.println(value_O2);

  String mixO2 = String(Vin_O2) + "," + String(AF);
  return mixO2;
}


//////////////////////////////// Voltage meter pin TPS //
float setO2_value = 1.00;
//const int TouchPinA =  2;
//int val_TouchA = touchRead(TouchPinA);

String Volt_TP() {
  // 5v from TPS in Over limit 3,3v input eap32 //
  // R1=1000 ohm, R2=1000 ohm   //

  int level;
  int  valueRead = analogRead(TPSin);
  float Vin_TP = (valueRead * 6.0) / 4095;
  float Per_in = valueRead / 40.95; // (4095 / 100=40.95)

  Serial.print("Vin TPS = ");
  Serial.print(Vin_TP);
  Serial.print("\tOut = ");
  Serial.print(level);
  Serial.print("\t\tValue:  ");
  Serial.print(valueRead);
  Serial.print("\t%:  ");
  Serial.println(Per_in);


  float Per_out;
  float convert = valueRead * 0.0635; // 255/4095=0.06227 + error
  //  float convert2 = map(analogRead(27), 0, 4095, 0, 255);


  /*
    /// manuad Mode TPs from remote  ///
    int select = select;
    if (touchRead(TouchPinA) < 50) {
    select = select + 1;
    if(select == 0){
     select = select + 4;
    }
    }return select;
    Serial.println("select : " + String(select));

    int Remote;
    unsigned long last1, last2, last3;
    if (analogRead(35) > 3000  && analogRead(36) > 3000) {
    //unsigned long Time_1 = millis();
    if (millis() - last1 >= 5000) {
    last1 = millis();
    Remote = 1;
    digitalWrite(21, 1);
    Serial.print(" 1,"); //
    delay(300);
    }
    else if (millis() - last2 >= 8000) {
    last2 = millis();
    Remote = 2;
    digitalWrite(21, 1);
    Serial.print(" 1,"); //
    delay(300);
    digitalWrite(21, 0);
    Serial.print(" 0,"); //
    delay(300);
    digitalWrite(21, 1);
    Serial.print(" 1,"); //
    delay(300);
    }
    else if (millis() - last3 >= 10000) {
    last3 = millis();
    Remote = 3;
    digitalWrite(21, 1);
    Serial.print(" 1,"); //
    delay(300);
    digitalWrite(21, 0);
    Serial.print(" 0,"); //
    delay(300);
    digitalWrite(21, 1);
    Serial.print(" 1,"); //
    delay(300);
    digitalWrite(21, 0);
    Serial.print(" 0,"); //
    delay(300);
    digitalWrite(21, 1);
    Serial.println(" 1,"); //
    delay(300);
    }
    }
  */

  std::string rxValue = pCharacteristic->getValue();
  String BLEvalue = rxValue.c_str();

  // Convert String to Intiger
  float set_O2 = BLEvalue.toFloat();  //// ค่าที่กำหนดให้รักษาระดับ 12.7 หรือหน่วยเป็นโวลต์
  if (set_O2 > 0 && set_O2 < 5) {
    setO2_value = set_O2;
    Serial.print("<=> Set O2 : ");  // solution 2
    Serial.println(setO2_value);
  }

  //  if (!isdigit(set_O2)) {
  //    Serial.print("isdigit, Set O2 : "); // solution 1
  //    Serial.println(set_O2);
  //  }


  char search;
  int a = rxValue.length(); // จำนวนตัวอักษรที่ส่งเข้าบอร์ด
  a = a - 1;

  int valuePin = analogRead(CheckDac25);  //เช็คโวลต์ DAC25
  float chekDac25 = (valuePin * 3.3) / 4095;

  switch (modeTps) {
    case 'O': {
        Serial.print("*****Ori*******TPS : Percent ");
        Serial.println(Per_in);
        break;
      }
    case 'A': {
        level = convert * pow(2, (convert - 255) / 255);
        Per_out = level / 2.55; // 255/100=2.55
        dacWrite(DAC1, level);
        Serial.print("*****A********TPS down : ");
        Serial.print(level);
        Serial.print("\t%:  ");
        Serial.println(Per_out);
        break;
      }
    //    case 'B': {
    //        level = convert;
    //        Per_out = level / 2.55; // 255/100=2.55
    //        dacWrite(DAC1, level);
    //        Serial.print("*****B********TPS normal : ");
    //        Serial.print(level);
    //        Serial.print("\t%:  ");
    //        Serial.println(Per_out);
    //        break;
    //      }
    case 'C': {
        level = (convert * (510 - convert)) / 255;
        Per_out = level / 2.55; // 255/100=2.55
        dacWrite(DAC1, level);
        Serial.print("*****C********TPS up : ");
        Serial.print(level);
        Serial.print("\t%:  ");
        Serial.println(Per_out);
        break;
      }

    case 'N': {
        Serial.print("############ Mode(ai) O2old");

        if (Vin_O2 < setO2_value) {
          level = convert + 5.1;  // up TPs 2%  (255/100 = 2.55 = 1%)
          Per_out = level / 2.55; // 255/100=2.55
          dacWrite(DAC1, level);
          Serial.print(" < ");
        }
        else if (Vin_O2 == setO2_value) {
          level = convert;
          Per_out = level / 2.55; // 255/100=2.55
          dacWrite(DAC1, level);
          Serial.print(" = ");
        }
        else if (Vin_O2 > setO2_value) {
          level = convert - 5.1;  //  Down TPs 2%
          Per_out = level / 2.55; // 255/100=2.55
          dacWrite(DAC1, level);
          Serial.print(" > ");
        }
        else if (setO2_value = 0) {
          level = convert ;
          Per_out = level / 2.55; // 255/100=2.55
          dacWrite(DAC1, level);
          Serial.print(" 0..");
        }

        Serial.print(setO2_value);
        Serial.print(" : ");
        Serial.println(Vin_O2 - setO2_value);
        Serial.print("TPS in : ");
        Serial.print(level);
        Serial.print("\t%:  ");
        Serial.println(Per_out);
        Serial.print("******O2*******TPS realtime value : ");
        Serial.print(level);
        Serial.print("\t%:  ");
        Serial.println(Per_out);
        break;
      }
    default: {
        level = convert;
        Per_out = level / 2.55;              // 255/100=2.55
        dacWrite(DAC1, level);
        Serial.print("*****B0********TPS normal : ");
        Serial.print(level);
        Serial.print("\t%:  ");
        Serial.println(Per_out);

        break;
      }
  }

  Serial.print("modeTps : ");
  Serial.print(modeTps);
  Serial.print(", BLE send float : ");
  Serial.println(setO2_value);
  Serial.print("length a : ");
  Serial.print(a);
  Serial.print(", DAC 25 : ");
  Serial.println(chekDac25);

  float Vout_TP = (level * 5.0) / 255;
  String mixTP = String(Vin_TP) + "," + String(Vout_TP);
  return mixTP;
}


///////////////////////////////////// Temprature CPU
extern "C" {
  uint8_t temprature_sens_read();
}

String TempCPU() {
  // fareha to celsius degrees
  float Temp = (temprature_sens_read() - 32) / 1.8;
  Serial.print("Temp CPU : ");
  Serial.print(Temp);
  Serial.println(" C'");

  String Cpu = String(Temp);
  return Cpu;
}


/////////////////////////////////// Notification & Touch Pin
//const int TouchPin1 =  32;
//const int TouchPin2 =  33;

void Alarm_Touch() {
  int val_Touch1 = touchRead(TouchPin1);
  int val_Touch2 = touchRead(TouchPin2);

  if (VoutSys_value > 3000) {
    digitalWrite(Buzzer, 0);
    Serial.println("System Open");
  }
  else if (VoutSys_value < 3000 && analogRead(RemoteOffBuzzer) < 3000) {
    Serial.print("Value touch pin (32, 33) : ");
    Serial.print(val_Touch1);
    Serial.print (",  ");
    Serial.print(val_Touch2);

    if (val_Touch1 < 10 || VoutSys_value < 1000) {
      Serial.println("\t-เปิด");

      //int time1 = millis();
      //if ( time1 <= 10000) {
      for (int i = 0; i < 15; i++) {
        digitalWrite(Buzzer, 1);
        Serial.print(" 1,"); //
        delay(500);
        if (analogRead(RemoteOffBuzzer) > 3000 || VoutSys_value > 3000) {
          Serial.println();
          digitalWrite(Buzzer, 0);
          Serial.print(" 0,"); //
          delay(500);
          break;
        }
        if (analogRead(RemoteOffBuzzer) > 3000 || VoutSys_value > 3000) {
          Serial.println();
          break;
        }
      }
    } else {
      digitalWrite(Buzzer, 0);
      Serial.println("\t-ปิด Sensor sleep ..zzz");
    }
  }
  else if (VoutSys_value < 3000 && analogRead(RemoteOffBuzzer) > 3000) {
    digitalWrite(Buzzer, 0);
    Serial.println("\tRemote Puch OFF");
  }
}


///////////////////////////////// Send-> Status on App////////////////////////
// Ligth Sys
bool swing = false;
String StatusLigth() {
  String St1;
  if (VoutSys_value > 3000 && swing == false) {
    St1 = "L1";
    swing = true;
  }
  else if (VoutSys_value < 3000 && swing == true) {
    St1 = "L0";
    swing = false;
  }
  Serial.print("St1 statusligth: ");
  Serial.println(St1);
  return St1;
}

// Ligth Alarm
String StatusAlarm() {
  int Alarm = digitalRead(Buzzer);
  String St2;
  if (Alarm == 1) {
    St2 = "A1";
  }
  else {
    St2 = "A0";
  }
  Serial.print("St2 statusAlarm: ");
  Serial.println(St2);
  return St2;
}


/*
   ////////// Remote select mode Tps
  const int TouchPin3 =  2;

  void b(){
  int val_Touch3 = touchRead(TouchPin3);
  }

  void touch_ModeTPs() {
  int val_Touch3 = touchRead(TouchPin3);
  int round_mode ;
  if(val_Touch3 < 10){
  round_mode = round_mode + 1;
  if(round_mode == 4){
    round_mode = round_mode - 4;
  }
  }
  Serial.print("value touch : ");
  Serial.print(val_Touch3);
  Serial.print("\tround : ");
  Serial.println(round_mode);

  switch (round_mode){
  case '0': digitalWrite(9, 0);
            digitalWrite(10, 1);
            digitalWrite(11, 0);
            break;
  case '1': digitalWrite(9, 1);
            digitalWrite(10, 0);
            digitalWrite(11, 0);
            break;
  case '2': digitalWrite(9, 0);
            digitalWrite(10, 1);
            digitalWrite(11, 0);
            break;
  case '3': digitalWrite(9, 0);
            digitalWrite(10, 0);
            digitalWrite(11, 1);
            break;
  }

  }
*/

////////////////////////////////////////////////////////////////////// RSSi
/*
  static BLEAddress *pServerAddress;
  BLEScan* pBLEScan;
  BLEClient*  pClient;
  bool deviceFound = false;

  String knownAddresses[] = { "5a:fd:95:9f:98:4a"};
  unsigned long entry;

  // สร้างฟังก์ชั่น Callback เมื่ออุปกรณ์เชื่อมต่อ(จับสัญญาณได้)
  static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  Serial.print("Notify callback for characteristic ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
  }

  // สร้าง Class Callback เพื่อตรวจจับระยะของอุปกรณ์
  class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice Device) {
      Serial.print("BLE Advertised Device found: ");
      Serial.println(Device.toString().c_str());
      pServerAddress = new BLEAddress(Device.getAddress());
      // ให้ known = false
      bool known = false;
      // ถ้าจับสัญญาณของอุปกรณ์ที่เรากำหนดไว้ได้ ให้ known = true
      for (int i = 0; i < (sizeof(knownAddresses) / sizeof(knownAddresses[0])); i++) {
        if (strcmp(pServerAddress->toString().c_str(), knownAddresses[i].c_str()) == 0)
          known = true;
      }
      // เมื่อ known = true ทำการเช็คค่า RSSI ถ้าถึงค่าที่กำหนด DeviceFound = true ถ้ายังไม่ถึง(เข้าใกล้ไม่พอ) DeviceFound = false
      if (known) {
        Serial.print("Device found: ");
        Serial.println(Device.getRSSI());
        // สามารถเปลี่ยนค่า RSSI ตรงนี้ได้ตามต้องการ
        if (Device.getRSSI() > -55) {
          deviceFound = true;
        }
        else {
          deviceFound = false;
        }
        Device.getScan()->stop();
        delay(100);
      }
    }
  };

  // ฟังก์ชั่นการทำงาน ถ้า DeviceFound = true ให้ LED เปิด และถ้า DeviceFound = false ให้ LED ปิด
  void Working() {
  Serial.println();
  Serial.println("BLE Scan restarted.....");
  deviceFound = false;
  BLEScanResults scanResults = pBLEScan->start(5);
  if (deviceFound) {
    digitalWrite(23, HIGH);
  }
  else {
    digitalWrite(23, LOW);
  }
  }
*/

///////----------------------------------------------------------------///////
////////////////////////////////////// RPM chanel
int num;
void RPM() {
  num++;
}

///////////////////////////////////////////////////////////////////
void setup() {

  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  initBLE();
  Pinmode();

  attachInterrupt(RPMpin, RPM, FALLING);

}

////////////////////////////////////////////////////////////////////
void loop() {

  //Alarm_Touch();
  TempCPU();
  StatusLigth();
  StatusAlarm();
  Volt_Sys();
  Volt_O2();
  Volt_TP();
  //Working();

  long se;
  if (millis() - se >= 1000) {
    se = millis();
    int rpm = num * 60;
    Serial.print("num : ");
    Serial.print(num);
    Serial.print(",   Rpm : ");
    Serial.println(rpm);
    num = 0;

    ///
//    String ligth =  StatusLigth() + "," + Volt_TP();
//    pCharacteristic->setValue(ligth.c_str());
//    pCharacteristic->notify();
//    Serial.println(ligth);
  }


  Serial.println();
  Serial.println();

  delay(1000);  // ลบเมื่อใช้งานจริง
}
