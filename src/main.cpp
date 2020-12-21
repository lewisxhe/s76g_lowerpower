#include "STM32LowPower.h"
#include <LoRa.h>
#include <U8g2lib.h>
#include "utilities.h"

HardwareSerial  SerialGPS(GPS_RX, GPS_TX);

U8G2_SSD1306_64X32_1F_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ PA8);


/*************************************************
*  _            ______
* | |           | ___ \
* | |      ___  | |_/ /  __ _
* | |     / _ \ |    /  / _` |
* | |____| (_) || |\ \ | (_| |
* \_____/ \___/ \_| \_| \__,_|
*
************************************************/

bool isrReceived = false;

void setRadioDirection(bool rx)
{
    isrReceived = rx;
    digitalWrite(RADIO_ANT_SWITCH_RXTX, rx ? HIGH : LOW);
}

#define REG_LR_TCXO               0x4B
#define RFLR_TCXO_TCXOINPUT_MASK  0xEF
#define RFLR_TCXO_TCXOINPUT_ON    0x10
#define RFLR_TCXO_TCXOINPUT_OFF   0x00  // Default

bool setupLoRa(void)
{
    SPI.setMISO(RADIO_MISO);
    SPI.setMOSI(RADIO_MOSI);
    SPI.setSCLK(RADIO_SCLK);
    SPI.begin();
    LoRa.setSPI(SPI);
    LoRa.setPins(RADIO_NSS, RADIO_RESET, RADIO_DIO_0);// set CS, reset, IRQ pin
    if (!LoRa.begin(LORA_BAND)) { // initialize ratio at 915 MHz
        LOG("setupLoRa FAIL");
        return false;
    }
    //! Initialize Radio ant switch pin
    pinMode(RADIO_ANT_SWITCH_RXTX, OUTPUT);
    //! Lora ANT Switch 1:Rx, 0:Tx
    setRadioDirection(true);

    pinMode(PC1, INPUT);
    digitalWrite(PC1, HIGH);
    if ( digitalRead(PC1) == 0 ) {
        uint8_t tcxo =  LoRa.readRegister( REG_LR_TCXO ) & RFLR_TCXO_TCXOINPUT_MASK;
        LoRa.writeRegister(REG_LR_TCXO, tcxo | RFLR_TCXO_TCXOINPUT_OFF);
    }
    return true;
}

void sleepLoRa()
{
    LoRa.end();
}

void powerSave()
{
    // /* Enable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_Struct;
    //relase all pins but PB_2 (GPS_RST_X)
    GPIO_Struct.Pin = GPIO_PIN_All;
    GPIO_Struct.Mode = GPIO_MODE_ANALOG;
    GPIO_Struct.Pull = GPIO_NOPULL;
    GPIO_Struct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOA, &GPIO_Struct);
    HAL_GPIO_Init(GPIOD, &GPIO_Struct);
    HAL_GPIO_Init(GPIOE, &GPIO_Struct);
    HAL_GPIO_Init(GPIOH, &GPIO_Struct);

    GPIO_Struct.Pin &= (~GPIO_PIN_2);
    HAL_GPIO_Init(GPIOB, &GPIO_Struct);

    //GPS UART4 TX & RX still not change
    //Level Shifter OE pin still not change, PC6
    GPIO_Struct.Pin = GPIO_PIN_All;
    GPIO_Struct.Pin &= (~GPIO_PIN_10);
    GPIO_Struct.Pin &= (~GPIO_PIN_11);
    GPIO_Struct.Pin &= (~GPIO_PIN_6);   ///GPS enable
    HAL_GPIO_Init(GPIOC, &GPIO_Struct);

    // for LoRa sx1276 TCXO OE Pin
    GPIO_Struct.Pin = GPIO_PIN_7;
    GPIO_Struct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Struct.Pull = GPIO_NOPULL;
    GPIO_Struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_Struct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

    //Sx1276 SPI PIN can be not analog PIN
    GPIO_Struct.Pin   = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_Struct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_Struct.Pull  = GPIO_PULLUP;
    GPIO_Struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( GPIOB, &GPIO_Struct );
    //Sx1276 SPI power saving setting
    HAL_GPIO_WritePin( GPIOB, GPIO_PIN_15, GPIO_PIN_SET );
    HAL_GPIO_WritePin( GPIOB, GPIO_PIN_14, GPIO_PIN_SET );
    HAL_GPIO_WritePin( GPIOB, GPIO_PIN_13, GPIO_PIN_SET );
    HAL_GPIO_WritePin( GPIOB, GPIO_PIN_12, GPIO_PIN_SET );

    /* Disable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_DISABLE();
    // __HAL_RCC_GPIOB_CLK_DISABLE();
    // __HAL_RCC_GPIOC_CLK_DISABLE();
    // __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();
    __HAL_RCC_GPIOE_CLK_DISABLE();

}

/**********************************************
*  _____ ______  _____
* |  __ \| ___ \/  ___|
* | |  \/| |_/ /\ `--.
* | | __ |  __/  `--. \
* | |_\ \| |    /\__/ /
*  \____/\_|    \____/
*
************************************************/


void waitAck()
{
    uint32_t smap = millis() + 2000;
    while (millis() < smap) {
        while (SerialGPS.available() > 0) {
            LOG_WRITE(SerialGPS.read());
        }
    }
}


void sleepGPS(void)
{
    SerialGPS.println("@GSTP");
    waitAck();
    SerialGPS.println("@BUP");
    waitAck();
    SerialGPS.println("@SLP 2");
    waitAck();
    delay(5);
    SerialGPS.end();
}

void setupGPS()
{
    SerialGPS.begin(GPS_BAUD_RATE);

    //! Added 1pps intput
    pinMode(GPS_1PPS_INTPUT, INPUT);

    pinMode(GPS_LEVEL_SHIFTER_EN, OUTPUT);
    digitalWrite(GPS_LEVEL_SHIFTER_EN, HIGH);

    pinMode(GPS_RST, OUTPUT);
    //Set  Reset Pin as 0
    digitalWrite(GPS_RST, LOW);
    //Scope shows 1.12s (Low Period)
    delay(200);
    //Set  Reset Pin as 1
    digitalWrite(GPS_RST, HIGH);
    delay(100);

    //! Start GPS connamd
    SerialGPS.println("@GSR");
}

/*********************************************
*   _____  _      _____ ______
* |  _  || |    |  ___||  _  \
* | | | || |    | |__  | | | |
* | | | || |    |  __| | | | |
* \ \_/ /| |____| |___ | |/ /
*  \___/ \_____/\____/ |___/
************************************************/
bool oledFind = false;

void sleepOLED()
{
    u8g2.sleepOn();
    Wire.end();
}

void wakeupOLED()
{
    u8g2.sleepOff();
}

bool setupOLED(void)
{
    Wire.setSCL(I2C_SCL);
    Wire.setSDA(I2C_SDA);
    Wire.begin();
    Wire.beginTransmission(SSD1306_ADDRRESS);
    if (Wire.endTransmission() == 0) {
        u8g2.begin();
        u8g2.clearBuffer();
        u8g2.setFontMode(1);
        u8g2.setFont(u8g2_font_cu12_tr);
        u8g2.setCursor(10, 15);
        u8g2.print(F("SoftRF"));
        u8g2.setCursor(10, 30);
        u8g2.print(F("LilyGo"));
        u8g2.sendBuffer();
        Serial.println("Device FIND");
        oledFind = true;
    } else {
        Serial.println("Device FAILED");
        oledFind = false;
    }
    return oledFind;
}


/**********************************************
* ___  ___  ___   _____  _   _
* |  \/  | / _ \ |_   _|| \ | |
* | .  . |/ /_\ \  | |  |  \| |
* | |\/| ||  _  |  | |  | . ` |
* | |  | || | | | _| |_ | |\  |
* \_|  |_/\_| |_/ \___/ \_| \_/
*
************************************************/

#ifdef LED_BUILTIN
#undef LED_BUILTIN
#define LED_BUILTIN     PA3
#endif

void repetitionsIncrease()
{
    // This function will be called once on device wakeup
    // You can do some little operations here (like changing variables which will be used in the loop)
    // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
}


void setup()
{
    Serial.setRx(UART_RX);
    Serial.setTx(UART_TX);
    Serial.begin(9600);

    LowPower.begin();

    LowPower.attachInterruptWakeup(TTP223_TP_PIN, repetitionsIncrease, RISING);

    Serial.println("LowPower.begin()");
    setupOLED();
    setupLoRa();
    setupGPS();
    delay(3000);

    Serial.println("Press to sleep !");
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(TTP223_VDD_PIN, OUTPUT);
    pinMode(TTP223_TP_PIN, INPUT);

    digitalWrite(TTP223_VDD_PIN, HIGH);
    while (digitalRead(TTP223_TP_PIN) == LOW) {
        digitalToggle(LED_BUILTIN); delay(500);
    }

    Serial.println("sleep LoRa .");
    sleepGPS();
    Serial.println("sleep GPS .");
    sleepLoRa();
    delay(500);
    sleepOLED();
    Serial.println("sleep OLED .");
    delay(500);
    Serial.println("powerSave .");
    delay(500);
    Serial.end();

    /*Low power processing*/
    powerSave();
}

void loop()
{
    LowPower.deepSleep();
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.setRx(UART_RX);
    Serial.setTx(UART_TX);
    Serial.begin(9600);
    Serial.print("Wakeup:");
    Serial.println(millis());
    setupOLED();
    int i = 15;
    while (i--) {
        digitalWrite(LED_BUILTIN, HIGH); delay(500);
        digitalWrite(LED_BUILTIN, LOW);  delay(500);
    }
    sleepOLED();
    Serial.end();
    delay(1000);
}





