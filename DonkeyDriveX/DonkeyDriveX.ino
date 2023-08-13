// #include <esp_now.h>
// #include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// #define DEBUG  // Uncomment to enable debugging output

#define CH1_PIN 4  //接收机pwm输入CH1通道为，GPIO4
#define CH2_PIN 5  //接收机pwm输入CH2通道为，GPIO5
#define CH3_PIN 6  //接收机pwm输入CH3通道为，GPIO6
#define CH4_PIN 7  //接收机pwm输入CH4通道为，GPIO7

#define STEERING_PIN 8  //PIN of Servo
#define THROTTLE_PIN 9  //PIN of ESC

#define CH_STEERING 0 //index of pwm_value[]
#define CH_THROTTLE 1 //index of pwm_value[]
#define CH_PARK 2 //index of pwm_value[]
#define CH_MODE 3 //index of pwm_value[]

#define CAR_MODE_MANUAL 0 //0为遥控模式
#define CAR_MODE_SEMI_AUTO 1 //1为自动方向和手动油门模式
#define CAR_MODE_FULL_AUTO 2 //2为自动驾驶模式

#define MIN_MICROS      1050 
#define MAX_MICROS      1950

volatile int pwm_value[4] = {0, 0, 0, 0}; // value of CH1, CH2, CH3, CH4
volatile unsigned long rise_time[4] = {0, 0, 0, 0}; // time of rising edge of CH1, CH2, CH3, CH4

void IRAM_ATTR handle_interrupt(int channel) { // interrupt handler
  static int pin_state[4] = {0, 0, 0, 0};
  pin_state[channel] = digitalRead(channel + CH1_PIN);
  if (pin_state[channel] == HIGH) {
    rise_time[channel] = micros();
  } else {
    pwm_value[channel] = micros() - rise_time[channel];
  }
}

void IRAM_ATTR CH1_interrupt() { handle_interrupt(0); } // interrupt handler
void IRAM_ATTR CH2_interrupt() { handle_interrupt(1); }
void IRAM_ATTR CH3_interrupt() { handle_interrupt(2); }
void IRAM_ATTR CH4_interrupt() { handle_interrupt(3); }

void (*isr_functions[4])() = {CH1_interrupt, CH2_interrupt, CH3_interrupt, CH4_interrupt};  // array of function pointers

// Define a data structure
struct struct_message {
  int throttle; // 油门值
  int steering; // 转向值
  int mode; //驾驶模式，0为遥控模式，1为自动方向和手动油门模式，2为自动驾驶模式
  bool park;  //停车状态，0为停车，1为起步
} ;

struct struct_message esp_now_data = {0, 0, 0, false}; // Initialize the structure at declaration
struct struct_message rc_data = {0, 0, 0, false}; // Initialize the structure at declaration
struct struct_message pilot_data = {0, 0, 0, false}; // Initialize the structure at declaration
struct struct_message car_output = {0, 0, 0, false}; // Initialize the structure at declaration

const int PWM_MIN = 819; // 'minimum' pulse length count (out of 4096)
const int PWM_MAX = 1638; // 'maximum' pulse length count (out of 4096)
const int MOTOR_MID = 1229; // 需要实际测试
const int MOTOR_RANGE = 390; // Pulse range for Motor Throttle # change from 60 to 90 
const int SERVO_MID = 1270; //  需要实际测试
const int SERVO_RANGE = 390; // Pulse range for Motor Throttle # change from 60 to 90 
const int MOTOR_OFFSET = 0;
const int SERVO_OFFSET = 0;

// Create a structure object
// struct_message* myData;

// callback function executed when data is received.
// void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
// 	myData = (struct_message*)incomingData;

//   esp_now_data = *myData; //将接收到的数据赋值给esp_now_data

// }

// int adj(int v, int s) // v: value, s: step
// {
//     v = v + s;
//     if(v > 4095) v = 4095;
//     if(v < 0) v = 0;

//     return v;
// }

void park_change()
{
  if(pwm_value[CH_PARK] > 1500){
    rc_data.park  = false;
  }
  else{
    rc_data.park  = true;
  }
  car_output.park = rc_data.park;
}

void mode_change()  //根据遥控器的mode值，切换驾驶模式
{
   rc_data.mode = pwm_value[CH_MODE];
  if(rc_data.mode <= 1400)
  {
    car_output.mode = CAR_MODE_MANUAL;  //0为遥控模式
  }
  else if(rc_data.mode > 1400 && rc_data.mode < 1600 )
  {
    car_output.mode = CAR_MODE_SEMI_AUTO; //1为自动方向和手动油门模式
  }
  else
  {
    car_output.mode = CAR_MODE_FULL_AUTO; //2为自动驾驶模式
  }
}


bool I2CRead(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t *Data)
{
    bool ret = true;

    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    if(Wire.endTransmission()) 
    {
      ret = false;
      Serial.println("I2C Read Errro");      
    }

    // Read Nbytes
    Wire.requestFrom(Address, Nbytes);
    uint8_t index = 0;
    while (Wire.available())
    {
        Data[index++] = Wire.read();
    }

    return ret;
}

uint16_t I2CReadValue(uint8_t addr, uint8_t reg)
{
  uint16_t ret = -1;

  uint8_t data[2];      
  if(I2CRead(addr, reg, 2, data))
  {
    ret = (uint16_t)data[0] << 8 | data[1];                   
  }  
  
  return ret;
}

void I2CWriteValue(uint8_t Address, uint8_t Register, uint16_t Data)
{
    uint8_t* pData = (uint8_t*)&Data;         

    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(pData[1]);
    Wire.write(pData[0]);
    if(Wire.endTransmission()) 
    {
      Serial.println("I2C Write Error");      
    }
}

void read_ina219(){
  Serial.printf("%d: %f mA\r\n", 1, I2CReadValue(0x41, 1) / 100.0 / 0.01);
  Serial.printf("%d: %f V\r\n", 2, I2CReadValue(0x41, 2) / 2 / 1000.0);
}

void read_mpu6050(){
/* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
}

void setup() {
	Serial.begin(115200);
  Wire.begin(2, 3, 400000L); // SDA, SCL, 400kHz

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer Range
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  // set Gyro Range
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
  // Set filter bandwidth
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  // Set the RC receiver pins as inputs and attach the interrupts
  for (int i = 0; i < 4; i++) {
    pinMode(CH1_PIN + i, INPUT);
    attachInterrupt(digitalPinToInterrupt(CH1_PIN + i), isr_functions[i], CHANGE);
  }

  ledcSetup(CH_STEERING, 50, 14); // 定义通道0（steering）
  ledcSetup(CH_THROTTLE, 50, 14); // 定义通道1（Throttle）PWM输出为pin1，50Hz，14-bit resolution
	// 分配PWM输出通道到管脚	
  ledcAttachPin(STEERING_PIN, CH_STEERING);
  ledcAttachPin(THROTTLE_PIN, CH_THROTTLE);

	// WiFi.mode(WIFI_STA); 
  // for(int i = 0; i < 10; i++)
  //   Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  
  // if (esp_now_init() != esp_OK) {
	//   Serial.println("Error initializing ESP-NOW");
	//   return;
	// }

	// esp_now_register_recv_cb(OnDataRecv);
  
  //SETUP ENDS
}


void loop() {
  //  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  read_ina219();
  read_mpu6050();

  if (Serial.available()){
      String CMD = Serial.readStringUntil('\n');
      String CMD_steering = CMD;
      String CMD_throttle = CMD;

      int CMD_gap = CMD.indexOf(':');
      int CMD_len = CMD.length();

      CMD_steering.remove(CMD_gap,-1);
      CMD_throttle.remove(0,CMD_gap+1);
    
      pilot_data.steering = CMD_steering.toInt();
      pilot_data.throttle = CMD_throttle.toInt();
    }

  
  rc_data.steering = pwm_value[CH_STEERING];
  rc_data.throttle = pwm_value[CH_THROTTLE];
  park_change();  //pwm_value[CH_PARK]
  mode_change();  //pwm_value[CH_MODE]


  if(car_output.mode== CAR_MODE_FULL_AUTO) {
    // Controlled by Pilot
    car_output.throttle = pilot_data.throttle;
    car_output.steering = pilot_data.steering;
  } 
  else if(car_output.mode == CAR_MODE_SEMI_AUTO){
    // Controlled by both RC and Pilot
    // car_output.throttle = rc_data.throttle;
    car_output.throttle = map(rc_data.throttle-1493,888-1493,2149-1493,-100,100);
    car_output.steering = pilot_data.steering;
  }
  else { // car_output.mode = CAR_MODE_MANUAL
    // Controlled by RC Controller

    // RC => CAR
    car_output.steering = map(rc_data.steering-1488,872-1488,2113-1488,-100,100);
    car_output.throttle = map(rc_data.throttle-1493,888-1493,2149-1493,-100,100);

    // CAR => Pilot
    Serial.printf("T%dS%d\n",car_output.throttle,car_output.steering);
  }

  if(car_output.park == 1){
    car_output.throttle = 0;
  }
  // CAR => PWM
  int pwm_steering = map(car_output.steering,-100,100,SERVO_MID-SERVO_RANGE,SERVO_MID+SERVO_RANGE);
  int pwm_throttle = map(car_output.throttle,-100,100,MOTOR_MID-MOTOR_RANGE,MOTOR_MID+MOTOR_RANGE);

  pwm_steering = min(max(pwm_steering,PWM_MIN),PWM_MAX);
  pwm_throttle = min(max(pwm_throttle,PWM_MIN),PWM_MAX);

  
  #ifdef DEBUG // Print the values for debugging
  // Read the RC receiver values
  for (int i = 0; i < 4; i++) {
    Serial.print(" CH"); Serial.print(i + 1); Serial.print(": "); Serial.print(pwm_value[i]);
  }
  Serial.printf("  %d, %d, %d, %d, %d\n", car_output.steering, pwm_steering, car_output.throttle, pwm_throttle, car_output.park );
  #endif
  
  ledcWrite(CH_STEERING,pwm_steering);
  ledcWrite(CH_THROTTLE,pwm_throttle);

  delay(10);
}