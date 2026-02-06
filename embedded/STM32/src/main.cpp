// Code for CAN implementation
#include <SimpleFOC.h>
#include <encoders/mt6835/MagneticSensorMT6835.h>

#include "Drivers/FDCAN_STM32.h"
#include "Drivers/SysClock.h"
#include "Drivers/TIM.h"

#define LED_PIN PC6
#define phaseA PB6
#define phaseB PB7
#define phaseC PB9
#define MOT_EN PB1

// HAL definition
#define phaseA_HAL GPIO_PIN_6
#define phaseB_HAL GPIO_PIN_7
#define phaseC_HAL GPIO_PIN_9
#define MOT_EN_HAL GPIO_PIN_1

#define SPI_CLCK PA5
#define SPI_MISO PA6
#define SPI_MOSI PA7
#define CS PA0

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(phaseA, phaseB, phaseC);
SPISettings encoderSettings = SPISettings(1e6, MSBFIRST, SPI_MODE3);
MagneticSensorMT6835 encoder = MagneticSensorMT6835(CS, encoderSettings);

float target_velocity = 5;
const float max_target_velocity = 25;
float prevTime = 0.0;
float currTime;

uint8_t TxData_C2_To_C3[64];
uint8_t RxData_C3[8];
volatile int txDone = 0;
const static char motor_id = 'M';

Commander command = Commander(Serial);
void doTarget(char *cmd) { command.scalar(&target_velocity, cmd); }
void doMotor(char *cmd) { command.motor(&motor, cmd); }
void doLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup() {
  HAL_Init();
  SystemClock_Config();

  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("STM32 Serial OK!");

  /* Configure the system clock */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_FDCAN2_Init();
  MX_TIM4_Init();

  // MT6835_Init(&hspi1, MT_CS_GPIO, MT_CS_PIN);

  /* USER CODE BEGIN 2 */
  TxHeader.Identifier = 0x123; // Standard ID
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x123;
  sFilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
  }

  HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2, 38, 0);
  HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2);

  HAL_FDCAN_Start(&hfdcan2);

  // SIMPLEFOC
  digitalWrite(MOT_EN, LOW);
  delay(250);
  currTime = millis();
  SimpleFOCDebug::enable(&Serial);
  delay(500);

  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 25000;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 8;

  encoder.init();

  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  // link the motor and the driver

  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 3; // [V]
  motor.foc_modulation = FOCModulationType::SinePWM;

  motor.initFOC();

  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  pinMode(LED_PIN, OUTPUT);

  // init motor hardware
  if (!motor.init()) {
    Serial.println("Motor init failed!");
    return;
  }
  motor.useMonitoring(Serial);

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");
  command.add(motor_id, doMotor, "motor");
  motor.monitor_start_char =
      motor_id; // the same latter as the motor id in the commander
  motor.monitor_end_char =
      motor_id; // the same latter as the motor id in the commander

  command.verbose = VerboseMode::machine_readable; // can be set using the
                                                   // webcontroller - optional

  Serial.println("Motor ready!!!");
  Serial.println("Set target velocity [rad/s]");
}

void loop() {
  // // put your main code here, to run repeatedly:
  TxData_C2_To_C3[0] = '6';
  TxData_C2_To_C3[1] = '7';
  TxData_C2_To_C3[2] = '8';
  TxData_C2_To_C3[3] = '9';
  TxData_C2_To_C3[4] = '0';

  currTime = millis();

  if (currTime - prevTime > 100) {
    if (digitalRead(LED_PIN) == HIGH) {
      digitalWrite(LED_PIN, LOW);
    } else {
      digitalWrite(LED_PIN, HIGH);
    }
    prevTime = currTime;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData_C2_To_C3) !=
        HAL_OK) {
      Error_Handler();
    }

    while (HAL_FDCAN_IsTxBufferMessagePending(&hfdcan2, FDCAN_TX_BUFFER0)) {
      FDCAN_ProtocolStatusTypeDef status;
      HAL_FDCAN_GetProtocolStatus(&hfdcan2, &status);
      // Serial.printf("Hello from USB CDC! Tick: %lu\r\n", HAL_GetTick());
      // Serial.printf("BusOff: %d, Activity: %d, LastErrorCode: %d, RX FIFO
      // Fill: %lu\n",
      //   status.BusOff, status.Activity, status.LastErrorCode,
      //   HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0));
    };

    if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader,
                               RxData_C3) == HAL_OK) {
      target_velocity =
          max_target_velocity *
          (float)((RxData_C3[0] - '0') * 10 + (RxData_C3[1] - '0')) / 100.0f;
    }
  }

  motor.move(target_velocity);
  motor.loopFOC();

  motor.monitor();

  // Serial.println("ENCODER:");
  encoder.update();
  Serial.println(encoder.getAngle());

  // uint32_t mdeg = {1};
  // TxData_C2_To_C3[0] = (mdeg >> 24) & 0xFF;
  // TxData_C2_To_C3[1] = (mdeg >> 16) & 0xFF;
  // TxData_C2_To_C3[2] = (mdeg >> 8) & 0xFF;
  // TxData_C2_To_C3[3] = mdeg & 0xFF;
  // uint8_t burst[6] = { 0xA0, 0x03, 0, 0, 0, 0 };
}
