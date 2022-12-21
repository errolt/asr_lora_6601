/*!
 * \file      main.c
 *
 * \brief     LoRaMac classA device implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdio.h>
#include "utilities.h"
#include "LoRaMac.h"
#include "Commissioning.h"
#include "tremo_system.h"
#include "tremo_regs.h"
#include "tremo_adc.h"
#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "tremo_i2c.h"
#include <math.h>
#include "tremo_delay.h"
#include "tremo_pwr.h"
#include <sys/errno.h>
#include "bme280.h"

gpio_t *g_int_gpiox = GPIOA;
uint8_t g_int_pin = GPIO_PIN_6;

#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_CN470 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_CN470

#endif
int *__errno()
{
//return get_errno_ptr();
}

struct bme280_dev dev;
uint8_t dev_addr = BME280_I2C_ADDR_PRIM;

/*!
 * Defines the application data transmission duty cycle. 1h, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            3600000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2
   

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = 4;
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           16

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime = APP_TX_DUTYCYCLE;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * Device states
 */
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*gpio_t *g_test_gpiox = GPIOA;
uint8_t g_test_pin = GPIO_PIN_11;
volatile uint32_t g_gpio_interrupt_flag = 0;
void power_off(void)
{
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_PWR, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);

    gpio_init(g_test_gpiox, g_test_pin, GPIO_MODE_INPUT_PULL_UP);
    gpio_config_stop3_wakeup(g_test_gpiox, g_test_pin, true, GPIO_LEVEL_LOW);
    gpio_config_interrupt(g_test_gpiox, g_test_pin, GPIO_INTR_FALLING_EDGE);

    // NVIC config 
    NVIC_EnableIRQ(GPIO_IRQn);
    NVIC_SetPriority(GPIO_IRQn, 2);

    // Infinite loop 
    while (1) {

        pwr_deepsleep_wfi(PWR_LP_MODE_STOP3);

        if (g_gpio_interrupt_flag) {
            g_gpio_interrupt_flag = 0;
        }
        delay_ms(2000);
    }    
}
*/
uint8_t read_i2c(uint8_t chip_address,uint8_t address,uint8_t len,uint8_t* data)
{
   // start
    i2c_master_send_start(I2C0, chip_address, I2C_WRITE);
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);

    // write address
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    i2c_send_data(I2C0, address);
    while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);

    // restart
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    i2c_master_send_start(I2C0, chip_address, I2C_READ);
    while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);

    // read data
    i2c_clear_flag_status(I2C0, I2C_FLAG_RECV_FULL);
    for(int i=0;i<len;i++)
    {
        if(i==len-1)
        {
            i2c_set_receive_mode(I2C0, I2C_NAK);
        }
        else
        {
            i2c_set_receive_mode(I2C0, I2C_ACK);
        }
        while (i2c_get_flag_status(I2C0, I2C_FLAG_RECV_FULL) != SET);
        i2c_clear_flag_status(I2C0, I2C_FLAG_RECV_FULL);
        data[i] = i2c_receive_data(I2C0);
    }
    // stop
    i2c_master_send_stop(I2C0);

    return 0;
}
void write_i2c(uint8_t chip_address,uint8_t address, uint8_t data)
{
   // start
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    i2c_master_send_start(I2C0, chip_address, I2C_WRITE);
    while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);

    // write address
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    i2c_send_data(I2C0, address);
    while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);

    // write data
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    i2c_send_data(I2C0, data);
    while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);

    // stop
    i2c_master_send_stop(I2C0);

}

uint8_t write2_i2c(uint8_t chip_address,uint8_t address, uint8_t len, uint8_t *data)
{
   // start
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    i2c_master_send_start(I2C0, chip_address, I2C_WRITE);
    while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);

    // write address
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    i2c_send_data(I2C0, address);
    while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);

    // write data
    for(int i=0;i<len;i++)
    {
        i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
        i2c_send_data(I2C0, data[i]);
        while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);
    }

    // stop
    i2c_master_send_stop(I2C0);

}


/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    struct bme280_data comp_data;
    uint8_t rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
    delay_ms(10);
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);    
   
    AppDataSize = 8;
    AppData[0] = comp_data.temperature>>8;
    AppData[1] = comp_data.temperature;
    AppData[2] = comp_data.humidity>>16;
    AppData[3] = comp_data.humidity>>8;
    AppData[4] = comp_data.humidity;
    AppData[5] = comp_data.pressure>>16;
    AppData[6] = comp_data.pressure>>8;
    AppData[7] = comp_data.pressure;
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    
    
    return true;
}

static bool SendCustomFrame(uint8_t AppPortl)
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPortl;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPortl;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    
    
    return true;
}
/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            // Network not joined yet. Try to join again
            MlmeReq_t mlmeReq;
            mlmeReq.Type = MLME_JOIN;
            mlmeReq.Req.Join.DevEui = DevEui;
            mlmeReq.Req.Join.AppEui = AppEui;
            mlmeReq.Req.Join.AppKey = AppKey;

            if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
            {
                DeviceState = DEVICE_STATE_SLEEP;
            }
            else
            {
                DeviceState = DEVICE_STATE_CYCLE;
            }
        }
    }
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
    }
    NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    printf( "receive data: rssi = %d, snr = %d, datarate = %d, type = %d\r\n", mcpsIndication->Rssi, (int)mcpsIndication->Snr,
                 (int)mcpsIndication->RxDatarate, mcpsIndication->McpsIndication);
    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    if( mcpsIndication->FramePending == true )
    {
        // The server signals that it has pending data to be sent.
        // We schedule an uplink as soon as possible to flush the server.
        OnTxNextPacketTimerEvent( );
    }
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot
    if( mcpsIndication->RxData == true )
    {
        printf("Got data on port %i:",mcpsIndication->Port);
        for(int i=0;i<mcpsIndication->BufferSize;i++)
        {
            printf("%02X",mcpsIndication->Buffer[i]);
        }
        delay_ms(10);
    }
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                printf("joined\r\n");
                // Status is OK, node has joined the network
                DeviceState = DEVICE_STATE_SEND;
            }
            else
            {
                MlmeReq_t mlmeReq;
                
                printf("join failed\r\n");
                // Join was not successful. Try to join again
                mlmeReq.Type = MLME_JOIN;
                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.NbTrials = 8;

                if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                {
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                else
                {
                    DeviceState = DEVICE_STATE_CYCLE;
                }
            }
            break;
        }    gpio_config_stop3_wakeup(g_int_gpiox, g_int_pin, true, GPIO_LEVEL_LOW);

        case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
            }
            break;
        }
        default:
            break;
    }
    NextTx = true;
}

/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] mlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication( MlmeIndication_t *mlmeIndication )
{
    switch( mlmeIndication->MlmeIndication )
    {
        case MLME_SCHEDULE_UPLINK:
        {// The MAC signals that we shall provide an uplink as soon as possible
            OnTxNextPacketTimerEvent( );
            break;
        }
        default:
            break;
    }
}

static void lwan_dev_params_update( void )
{
    MibRequestConfirm_t mibReq;
    uint16_t channelsMaskTemp[6];
    channelsMaskTemp[0] = 0x00FF;
    channelsMaskTemp[1] = 0x0000;
    channelsMaskTemp[2] = 0x0000;
    channelsMaskTemp[3] = 0x0000;
    channelsMaskTemp[4] = 0x0000;
    channelsMaskTemp[5] = 0x0000;

    mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
    mibReq.Param.ChannelsDefaultMask = channelsMaskTemp;
    LoRaMacMibSetRequestConfirm(&mibReq);
    mibReq.Type = MIB_CHANNELS_MASK;
    mibReq.Param.ChannelsMask = channelsMaskTemp;
    LoRaMacMibSetRequestConfirm(&mibReq);
}

uint8_t BoardGetBatteryLevel( void )
{

    float gain_value;
    float dco_value;
    float battery;
    rcc_set_adc_clk_source(RCC_ADC_CLK_SOURCE_RCO48M);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_ADC, true);
    gpio_init(GPIOA, GPIO_PIN_8, GPIO_MODE_ANALOG);
    gpio_init(GPIOA, GPIO_PIN_11, GPIO_MODE_OUTPUT_PP_LOW);


    adc_get_calibration_value(false, &gain_value, &dco_value);

    adc_init();

    adc_config_clock_division(20); //sample frequence 150K

    adc_config_sample_sequence(0, 2);
    adc_config_sample_sequence(1, 2);
   
    adc_config_conv_mode(ADC_CONV_MODE_SINGLE);
    adc_enable(true);
    adc_start(true);
    while(!adc_get_interrupt_status(ADC_ISR_EOC));
    battery = adc_get_data();

    adc_start(false);
    adc_enable(false);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_ADC, false);

    gpio_init(GPIOA, GPIO_PIN_11, GPIO_MODE_INPUT_FLOATING);

    battery = ((1.2/4096) * battery - dco_value) / gain_value;
    battery =battery * 4.1333;

    printf("Battery V: %fV\r\n",battery);
    if(battery > 3.25)
    {//PSU
        return 0;//Mains powered
    }

    battery= 1 + ((((battery -2.5)/1)*255))*2;
    if(battery>254)battery=254;//Even if higher than 3V, but less than 3.25, report 100%.
    printf("Battery: %f\r\n",battery);
    return battery;
}


/**
 * Main application entry point.	printf("arch:ASR6601,%02X%02X%02X%02X%02X%02X%02X%02X\r\n",buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

 */

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
//    printf("Read from 0x%02X, address 0x%02X, len %i.\r\n",((uint8_t*)intf_ptr)[0],reg_addr,len);
    read_i2c(((uint8_t*)intf_ptr)[0],reg_addr,len,reg_data);
/*    printf("Done. Result ",reg_data[0]);
    for(int i=0;i<len;i++)
    {
        printf("0x%02X,",reg_data[i]);
    }
    printf("\r\n");
    */
    return 0;
}

int8_t user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    write2_i2c(((uint8_t*)intf_ptr)[0],reg_addr,len,reg_data);
    return 0;
}
int app_start( void )
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;
    i2c_config_t config;
    delay_init();

    
    DeviceState = DEVICE_STATE_INIT;

    printf("ClassA app start. Region: %i\r\n",ACTIVE_REGION);

    /* NVIC config */
    NVIC_EnableIRQ(GPIO_IRQn);
    NVIC_SetPriority(GPIO_IRQn, 2);

    gpio_init(GPIOA, GPIO_PIN_11, GPIO_MODE_ANALOG);

    // set iomux
    gpio_set_iomux(GPIOA, GPIO_PIN_14, 3);
    gpio_set_iomux(GPIOA, GPIO_PIN_15, 3);

    // init
    i2c_config_init(&config);
    i2c_init(I2C0, &config);
    i2c_cmd(I2C0, true);


    // setup BME
    int8_t rslt = BME280_OK;

    dev.intf_ptr = &dev_addr;
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_us = delay_us;

	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_1X;
	dev.settings.osr_t = BME280_OVERSAMPLING_1X;
	dev.settings.filter = BME280_FILTER_COEFF_OFF;
	dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;
	uint8_t settings_sel;
	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;

    rslt = bme280_init(&dev);
	rslt = bme280_set_sensor_settings(settings_sel, &dev);

    while( 1 )
    {
        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, ACTIVE_REGION );

                TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );
                
                lwan_dev_params_update();
                
                DeviceState = DEVICE_STATE_JOIN;
                break;
            }
            case DEVICE_STATE_JOIN:
            {
#if( OVER_THE_AIR_ACTIVATION != 0 )
                MlmeReq_t mlmeReq;

                // Initialize LoRaMac device unique ID
                system_get_chip_id((uint32_t*)DevEui);
                printf("DevEui, %02X%02X%02X%02X%02X%02X%02X%02X\r\n",DevEui[0], DevEui[1], DevEui[2], DevEui[3], DevEui[4], DevEui[5], DevEui[6], DevEui[7]);
                printf("AppEui, %02X%02X%02X%02X%02X%02X%02X%02X\r\n",AppEui[0], AppEui[1], AppEui[2], AppEui[3], AppEui[4], AppEui[5], AppEui[6], AppEui[7]);
                printf("AppKey, %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\r\n",AppKey[0],AppKey[1],AppKey[2],AppKey[3],AppKey[4],AppKey[5],AppKey[6],AppKey[7],AppKey[8],AppKey[9],AppKey[10],AppKey[11],AppKey[12],AppKey[13],AppKey[14],AppKey[15]);
                //BoardGetUniqueId(DevEui);

                mlmeReq.Type = MLME_JOIN;

                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.NbTrials = 8;

                if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                {
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                else
                {
                    DeviceState = DEVICE_STATE_CYCLE;
                }
#else

                mibReq.Type = MIB_NET_ID;
                mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_DEV_ADDR;
                mibReq.Param.DevAddr = DevAddr;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NWK_SKEY;
                mibReq.Param.NwkSKey = NwkSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_APP_SKEY;
                mibReq.Param.AppSKey = AppSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NETWORK_JOINED;
                mibReq.Param.IsNetworkJoined = true;
                LoRaMacMibSetRequestConfirm( &mibReq );

                DeviceState = DEVICE_STATE_SEND;
#endif
                break;
            }
            case DEVICE_STATE_SEND:
            {
                printf("SEND\r\n");
                if( NextTx == true )
                {
                    PrepareTxFrame( AppPort );

                    NextTx = SendFrame( );
                }
                
                // Schedule next packet transmission
                TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( 0, APP_TX_DUTYCYCLE_RND );
                DeviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                printf("CYCLE\r\n");
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                TimerStart( &TxNextPacketTimer );
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                //printf("SLEEP\r\n");
                // Wake up through events
                TimerLowPowerHandler( );
                
                // Process Radio IRQ
                Radio.IrqProcess( );
                break;
            }
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
    }
}
