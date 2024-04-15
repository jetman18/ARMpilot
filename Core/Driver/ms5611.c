#include "ms5611.h"
#include "../lib/timer.h"
#include "../lib/maths.h"
#include"i2c.h"
#include "blackbox.h"

#define MS5611_ADDR            (0x77<<1)
#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8
#define SAMPLERATIO             0x00  //sampleRatio
#define _PTYPE                  0  //unit type pressure
#define _TTYPE                  0  //unit type T
#define _SEALEVELPRESS          101325

int16_t _POFFSET = 0; // Pressure offset value where 1000 = 1000 Pascals (10.00 millibars)
int16_t _TOFFSET = 0; // Temperature offset value where 2000 = 20.00°C
static void ms5611_reset(void);
static uint16_t ms5611_prom(int8_t coef_num);
static uint8_t ms5611_crc(uint16_t *prom);
static uint8_t ms5611_read_crc(void);
static int8_t ms5611_check_crc(void);
static uint32_t ms5611_read_adc(void);
static void send_cmd(uint8_t);
static void ms_5611_readout();
int32_t _pressure;
float TEMP;
int _altitude;

static uint16_t ms5611_c[PROM_NB];  // on-chip ROM

static I2C_HandleTypeDef *hi2c = NULL;
static uint32_t baro_timer;
static float seaLevelPress;
static uint8_t readStep;


//black_box_file_t alt_;
void ms5611_init(I2C_HandleTypeDef *hi2c2)
{
    hi2c = hi2c2;
    readStep = 0;
    baro_timer = millis();
	
	//black_box_create_file(&alt_,"alt_data.txt");
    // reset sensor
    ms5611_reset();
    // read all coefficients
    for (int i = 0; i < PROM_NB; i++)
        ms5611_c[i] = ms5611_prom(i);
}
static void ms5611_reset(void)
{
	uint8_t data = CMD_RESET;
    HAL_I2C_Master_Transmit(hi2c,MS5611_ADDR,&data,1,1);
    HAL_Delay(2);
}

static uint16_t ms5611_prom(int8_t coef_num)
{
    uint8_t rxbuf[2] = { 0, 0 };
    HAL_I2C_Mem_Read(hi2c,MS5611_ADDR,CMD_PROM_RD + coef_num * 2,1,rxbuf,2,1);
    return rxbuf[0] << 8 | rxbuf[1];
}
static void send_cmd(uint8_t c)
{
   HAL_I2C_Master_Transmit(hi2c,MS5611_ADDR,&c,1,10);
}

static uint8_t ms5611_crc(uint16_t *n_prom)
{
    uint8_t n_bit;
    int16_t cnt; // Simple counter
    uint16_t n_rem; // CRC remainder
    uint16_t crc_read; // Original value of the CRC
    n_rem = 0x00;
    crc_read = n_prom[7];	// Save original value of CRC
    n_prom[7] = (0xFF00 & (n_prom[7])); // CRC byte is replaced by 0
    for ( cnt = 0; cnt < 16; cnt++ )
    {
		// choose LSB or MSB
        if ( cnt % 2 == 1 ) n_rem ^= (uint8_t)((n_prom[cnt>>1]) & 0x00FF);
        else n_rem ^= (uint8_t)(n_prom[cnt>>1]>>8);

        for (n_bit=8; n_bit>0; n_bit--)
        {
            if (n_rem & (0x8000))
            {
            	n_rem=(n_rem<<1) ^ 0x3000;
            }
            else
            {
                n_rem=(n_rem<<1);
            }
        }
   }
		return 0; // not use
}
static uint8_t ms5611_read_crc(void)
{
    uint16_t crc_read = ( 0x000F & ( ms5611_c[7] ) );
    return ( crc_read );
}
static int8_t ms5611_check_crc(void)
{
    int8_t status = 0;
    if(ms5611_crc(ms5611_c) == ms5611_read_crc())
         status = 1;
    return status;
}

static uint32_t ms5611_read_adc(void)
{  
    uint32_t value = 0;
    uint8_t rxbuf[3];
    rxbuf[0] = CMD_ADC_READ;
    HAL_I2C_Master_Transmit(hi2c,MS5611_ADDR,&rxbuf[0],1,10);
    HAL_I2C_Master_Receive(hi2c,MS5611_ADDR,rxbuf,3,10);
    value =  (rxbuf[0] << 16) | (rxbuf[1] << 8) | (rxbuf[2]);
    return value;
}

uint32_t _D1 = 0; 
uint32_t _D2 = 0; 
/*
void ms5611_start()
{
    switch (readStep){
    case 0:
        send_cmd(CMD_ADC_CONV + CMD_ADC_D2 + CMD_ADC_256);
        _D1 = _D2 = 0;
        readStep ++;
        break;
    case 1:
        _D2 = ms5611_read_adc();
        send_cmd(CMD_ADC_CONV + CMD_ADC_D1 + CMD_ADC_256);
        readStep ++;
        break;
    case 2:
        _D1 = ms5611_read_adc();
        if (_D1 == 0 || _D2 == 0) { 
        	readStep = 0;
             break;

        }else{
            ms_5611_readout();
        }
        readStep = 0;
        break;
    }

}
*/
void ms5611_start()
{  
	
	
	static int count = 0;
	count ++;
	if(count == 20){
       _D2 = ms5611_read_adc();
        send_cmd(CMD_ADC_CONV + CMD_ADC_D1 + CMD_ADC_256);
        count = 0;
		return;
	}
	
    switch (readStep){
    case 0:
        send_cmd(CMD_ADC_CONV + CMD_ADC_D2 + CMD_ADC_256);
        _D1 = _D2 = 0;
        readStep ++;
        break;

    case 1:
        _D1 = ms5611_read_adc();
        if (_D1 == 0 || _D2 == 0) { 
        	readStep = 0;
             break;

        }else{
            ms_5611_readout();
        }
        readStep = 0;
        break;
    }

}


float offset_altitude = 0;
static void ms_5611_readout()
{
	// While sensor document states these variables should be int32 and int64 unless everything is
	// int64 calculation errors occur.  To ensure compatibility across multiple platforms, I've chosen to
	// make their types as doubles.
	double dT = 0; // Difference in actual vs reference temperature
	double OFF = 0; // Offset at actual temperature
	double SENS = 0; //Sensitivy at actual temperature
	
	// When Pressure and Temperature start off as doubles, it tried to calculate it beyond it's precision
	// keep it to int32_t and then multiply by 0.01 to get only the level of precision the sensor has.
	int32_t pressure = 0;
	int32_t temp = 0;

	// Calculate 1st order pressure and temperature based on sensor used.
	dT=(float)_D2-(float)ms5611_c[5]*256.0f; // _D2 - Tref = _D2 - C5 * 2^8
	

	OFF=(float)ms5611_c[2]*65536.0f+(dT*(float)ms5611_c[4])/128.0f; // OFFt1 + TCO * dT = C2 * 2^16 + (C4 * dT) / 2^7
	SENS=(float)ms5611_c[1]*32768.0f+(dT*(float)ms5611_c[3])/256.0f; // SENSt1 + TCS * dT = C1 * 2^15 + (C3*dT) / 2^8
	
	temp=(2000.0f+(dT*(float)ms5611_c[6])/8388608.0f)+(float)_TOFFSET; // 20C + dT * TEMPSENS = 2000 + dT * C6 / 2^23
	
	// Perform higher order corrections based on sensor used.
	double T2=0., OFF2=0., SENS2=0.;
	if(temp<2000) {
		T2=dT*dT/2147483648.0f; // dT^2 / 2^31

		OFF2=5.0f*((temp-2000.0f)*(temp-2000.0f))/2.0f; // 5 * (temp - 2000)^2 / 2
		SENS2=5.0f*((temp-2000.0f)*(temp-2000.0f))/4.0f; // 5 * (temp - 2000)^2 / 4
	  
	  if(temp<-1500.0f) {
		OFF2+=7.0f*((temp+1500.0f)*(temp+1500.0f)); // OFF2 + 7 * (temp + 1500)^2
		SENS2+=11.0f*((temp+1500.0f)*(temp+1500.0f))/2.0f; // SENS2 + 11 * (temp + 1500) ^2 /2
	    
	  }
	}
	// Apply any required offsets from higher order corrections
	temp -= T2;
	OFF  -= OFF2;
	SENS -= SENS2;
	
	// Calculate pressure
	 pressure =((_D1*SENS/2097152.0f-OFF)/32768.0f)+_POFFSET; // (((_D1*SENS)/pow(2,21)-OFF)/pow(2,15))
	// Convert temperature to Celcius
	TEMP = temp * 0.01f;
    // press
	static int8_t pr_start = 1;
	 if(pr_start){
	   _pressure = pressure;
		 pr_start = 0;
	 }
	 else{
       _pressure = 0.7*_pressure + 0.3* pressure;
	 }

    // Altitude 
    //static float offset_altitude = 0;
    static uint8_t init_offset = 0;
    float altitude_cal =  44330.0f* (1. - pow(_pressure/ (float)_SEALEVELPRESS, 0.19029495)); // cm
/*
	while(init_offset < 5){
	    offset_altitude += altitude_cal;
		if(init_offset == 4){
		   offset_altitude /= 5;
		}
	    init_offset  ++;
	}
    if(init_offset > 4){
	   alt = altitude_cal ;//- offset_altitude;  
	}
	else{
	    alt = 0.0f;
	}
	*/

}

static float ms5611_getSeaLevel(double altitude)
{	
	if (_pressure == 0) {
		return -1;
	}
	
   // seaLevelPress = press / pow((1.0 - (altitude / 44330.0)), 5.255);
	
	return seaLevelPress;
}
