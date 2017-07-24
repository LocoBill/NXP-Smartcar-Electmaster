/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ������������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_adc.h
 * @brief      ADC����
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-08-28
 */


#ifndef __MK60_ADC_H__
#define __MK60_ADC_H__ 1

typedef enum ADCn  //ADC�˿�
{
    ADC0,
    ADC1
} ADCn;

//ADCͨ��
//���ѣ��õ�ʱ�򣬿���ֱ���� DP0  ���� DAD0 ��������Ҳ���ƣ���Ϊ�и��궨�壺#define DP0       DAD0
typedef enum ADC_Ch
{
    //SC1n[DIFF]= 0
    //  ------ADC0------Ұ�𿪷���˿ӡ��----       ------ADC1------Ұ�𿪷���˿ӡ��----
    DAD0 = 0, //	    ADC0_DP0				                    ADC1_DP0
    DAD1 = 1, //	    ADC0_DP1				                    ADC1_DP1
    DAD2 = 2, //	    PGA0_DP2					            PGA1_DP2
    DAD3 = 3, //	    ADC0_DP3				                    ADC1_DP3

    //ADCx_CFG2[MUXSEL] λ���� ADCx_SEn ͨ��Ϊ a �� b.
    AD4a = 4, //	    ����					                    ADC1_SE4a   -- PTE0
    AD5a = 5, //	    ����					                    ADC1_SE5a   -- PTE1
    AD6a = 6, //	    ����					                    ADC1_SE6a   -- PTE2
    AD7a = 7, //	    ����					                    ADC1_SE7a   -- PTE3

    //Ҳ�� 4��5��6��7
    AD4b = AD4a, //	    ADC0_SE4b	-- PTC2			                ADC1_SE4b   -- PTC8
    AD5b = AD5a, //	    ADC0_SE5b	-- PTD1			                ADC1_SE5b   -- PTC9
    AD6b = AD6a, //	    ADC0_SE6b	-- PTD5			                ADC1_SE6b   -- PTC10
    AD7b = AD7a, //	    ADC0_SE7b	-- PTD6			                ADC1_SE7b   -- PTC11

    AD8 = 8,  //	    ADC0_SE8  	-- PTB0			                ADC1_SE8    -- PTB0
    AD9 = 9,  //	    ADC0_SE9  	-- PTB1			                ADC1_SE9    -- PTB1
    AD10 = 10, //	    ADC0_SE10 	-- PTA7			                ADC1_SE10   -- PTB4
    AD11 = 11, //	    ADC0_SE11 	-- PTA8			                ADC1_SE11   -- PTB5
    AD12 = 12, //	    ADC0_SE12 	-- PTB2			                ADC1_SE12   -- PTB6
    AD13 = 13, //	    ADC0_SE13 	-- PTB3			                ADC1_SE13   -- PTB7
    AD14 = 14, //	    ADC0_SE14 	-- PTC0			                ADC1_SE14   -- PTB10
    AD15 = 15, //	    ADC0_SE15 	-- PTC1			                ADC1_SE15   -- PTB11
    AD16 = 16, //	    ADC0_SE16				                    ADC1_SE16
    AD17 = 17, //	    ADC0_SE17 	-- PTE24		                ADC1_SE17   -- PTA17
    AD18 = 18, //	    ADC0_SE18 	-- PTE25		                VREF Output
    AD19 = 19, //	    ADC0_DM0				                    ADC1_DM0
    AD20 = 20, //	    ADC0_DM1				                    ADC1_DM1
    AD21 = 21, //				        	                        ����
    AD22 = 22, //
    AD23 = 23, //	    DAC0_OUT(12-bit) -- DAC0_OUT		        DAC1_OUT(12-bit)
    AD24 = 24, //	    ����					����
    AD25 = 25, //	    ����					����
    AD26 = 26, //	    Temperature Sensor (S.E)		            Temperature Sensor (S.E)
    AD27 = 27, //	    Bandgap (S.E)				                Bandgap (S.E)
    AD28 = 28, //	    ����					                    ����
    AD29 = 29, //	    VREFH (S.E)				                    VREFH (S.E)
    AD30 = 30, //	    VREFL					                    VREFL
    AD31 = 31 //	    ����ADC0				                    ����ADC1
} ADC_Ch;

//����λ��
typedef enum ADC_nbit
{
    ADC_8bit   = 0x00,
    ADC_10bit  = 0x02,
    ADC_12bit  = 0x01,
    ADC_16bit  = 0x03
} ADC_nbit;


//�ⲿ�����ӿ�����
extern void     adc_init  (ADCn, ADC_Ch);              //AD��ʼ��
extern uint16      ad_once   (ADCn, ADC_Ch, ADC_nbit);    //�ɼ�һ��һ·ģ������ADֵ

extern uint16      ad_mid    (ADCn, ADC_Ch, ADC_nbit);    //��ֵ�˲���Ľ��
extern uint16      ad_ave    (ADCn, ADC_Ch, ADC_nbit, uint8 N); //��ֵ�˲���Ľ��
extern uint16      ad_flt    (ADCn, ADC_Ch, ADC_nbit);    //��ǰ���β�����ֵ�˲�   �ٶȿ���ad_ave�ܶ࣬������ڲ���ʱ��Ƚ϶̣��������ȷ�� ad_once�����������϶̵���������ȿ���ʹ�����

extern void     adc_start (ADCn, ADC_Ch, ADC_nbit);    //��ʼadcת��
extern void     adc_stop  (ADCn);                      //ֹͣADCת��


//�ڲ���������



#define ADC0_irq_no 57
#define ADC1_irq_no 58


// Bit shifting of bitfiled is already taken into account so
// bitfiled values are always represented as relative to their position.

/************************* #Defines ******************************************/



#define A                 0x0
#define B                 0x1


/////// NOTE: the following defines relate to the ADC register definitions
/////// and the content follows the reference manual, using the same symbols.


//// ADCSC1 (register)

// Conversion Complete (COCO) mask
#define COCO_COMPLETE     ADC_SC1_COCO_MASK
#define COCO_NOT          0x00

// ADC interrupts: enabled, or disabled.
// ADC �ж�: ʹ�ܻ��߽�ֹ
#define AIEN_ON           ADC_SC1_AIEN_MASK
#define AIEN_OFF          0x00

// Differential or Single ended ADC input
//��ֻ��ߵ���ADC����
#define DIFF_SINGLE       0x00
#define DIFF_DIFFERENTIAL ADC_SC1_DIFF_MASK

//// ADCCFG1

// Power setting of ADC
//ADC��Դ����
#define ADLPC_LOW         ADC_CFG1_ADLPC_MASK
#define ADLPC_NORMAL      0x00

// Clock divisor
//ʱ�ӷ�Ƶ
#define ADIV_1            0x00
#define ADIV_2            0x01
#define ADIV_4            0x02
#define ADIV_8            0x03

// Long samle time, or Short sample time
// ������ʱ����߶̲���ʱ��
#define ADLSMP_LONG       ADC_CFG1_ADLSMP_MASK
#define ADLSMP_SHORT      0x00

// How many bits for the conversion?  8, 12, 10, or 16 (single ended).
// ת������ 8, 12, 10, ���� 16 (����).
#define MODE_8            0x00
#define MODE_12           0x01
#define MODE_10           0x02
#define MODE_16           0x03

// ADC Input Clock Source choice? Bus clock, Bus clock/2, "altclk", or the
//                                ADC's own asynchronous clock for less noise
//ADC����ʱ��Դѡ�� ���ߣ�����/2����altclk������ADC�����첽ʱ���Լ�������
#define ADICLK_BUS        0x00
#define ADICLK_BUS_2      0x01
#define ADICLK_ALTCLK     0x02
#define ADICLK_ADACK      0x03

//// ADCCFG2


// ѡ��ͨ��A��ͨ��B
#define MUXSEL_ADCB       ADC_CFG2_MUXSEL_MASK
#define MUXSEL_ADCA       0x00

// Ansync clock output enable: enable, or disable the output of it
// �첽ʱ�����ʹ�ܣ�ʹ�ܣ����߽�ֹ���
#define ADACKEN_ENABLED   ADC_CFG2_ADACKEN_MASK
#define ADACKEN_DISABLED  0x00

// High speed or low speed conversion mode
// ���ٵ���ת��ʱ��
#define ADHSC_HISPEED     ADC_CFG2_ADHSC_MASK
#define ADHSC_NORMAL      0x00

// Long Sample Time selector: 20, 12, 6, or 2 extra clocks for a longer sample time
// ������ʱ��ѡ��20,12,6����2�������ʱ�Ӷ��ڳ�����ʱ��
#define ADLSTS_20          0x00
#define ADLSTS_12          0x01
#define ADLSTS_6           0x02
#define ADLSTS_2           0x03

////ADCSC2

// Read-only status bit indicating conversion status
// ֻ��״̬λֻ��ת��״̬
#define ADACT_ACTIVE       ADC_SC2_ADACT_MASK
#define ADACT_INACTIVE     0x00

// Trigger for starting conversion: Hardware trigger, or software trigger.
// For using PDB, the Hardware trigger option is selected.
// ������ʼת��:Ӳ����������������
#define ADTRG_HW           ADC_SC2_ADTRG_MASK
#define ADTRG_SW           0x00

// ADC Compare Function Enable: Disabled, or Enabled.
//ADC�ȽϹ���ʹ�ܣ���ֹ����ʹ��
#define ACFE_DISABLED      0x00
#define ACFE_ENABLED       ADC_SC2_ACFE_MASK

// Compare Function Greater Than Enable: Greater, or Less.
// �ȽϹ��ܴ��ڱȽ�ʹ�ܣ����ڻ���С��
#define ACFGT_GREATER      ADC_SC2_ACFGT_MASK
#define ACFGT_LESS         0x00

// Compare Function Range Enable: Enabled or Disabled.
// �ȽϹ��ܷ�Χʹ�ܣ�ʹ�ܻ��߽�ֹ
#define ACREN_ENABLED      ADC_SC2_ACREN_MASK
#define ACREN_DISABLED     0x00

// DMA enable: enabled or disabled.
// DMAʹ�ܣ�ʹ�ܻ��߽�ֹ
#define DMAEN_ENABLED      ADC_SC2_DMAEN_MASK
#define DMAEN_DISABLED     0x00

// Voltage Reference selection for the ADC conversions
// (***not*** the PGA which uses VREFO only).
// VREFH and VREFL (0) , or VREFO (1).

//ADCת���ĵ�ѹ�ο�ѡ��
#define REFSEL_EXT         0x00
#define REFSEL_ALT         0x01
#define REFSEL_RES         0x02     /* reserved */
#define REFSEL_RES_EXT     0x03     /* reserved but defaults to Vref */

////ADCSC3

// Calibration begin or off
// У׼��ʼ���߹ر�
#define CAL_BEGIN          ADC_SC3_CAL_MASK
#define CAL_OFF            0x00

// Status indicating Calibration failed, or normal success
// ָʾУ׼ʧ�ܳɹ���״̬
#define CALF_FAIL          ADC_SC3_CALF_MASK
#define CALF_NORMAL        0x00

// ADC to continously convert, or do a sinle conversion
// ADC����ת������һ��ת��
#define ADCO_CONTINUOUS    ADC_SC3_ADCO_MASK
#define ADCO_SINGLE        0x00

// Averaging enabled in the ADC, or not.
// ƽ������ʹ�ܻ��߽�ֹ
#define AVGE_ENABLED       ADC_SC3_AVGE_MASK
#define AVGE_DISABLED      0x00

// How many to average prior to "interrupting" the MCU?  4, 8, 16, or 32
// MCU�����ж�ǰ��ƽ������4,8,16������32
#define AVGS_4             0x00
#define AVGS_8             0x01
#define AVGS_16            0x02
#define AVGS_32            0x03

///////////////////////////////PGA���֣����ݴ�ѧ��û�е�/////////////////////////////////////
////PGA

// PGA enabled or not?
#define PGAEN_ENABLED      ADC_PGA_PGAEN_MASK
#define PGAEN_DISABLED     0x00

// Chopper stabilization of the amplifier, or not.
#define PGACHP_CHOP        ADC_PGA_PGACHP_MASK
#define PGACHP_NOCHOP      0x00

// PGA in low power mode, or normal mode.
#define PGALP_LOW          ADC_PGA_PGALP_MASK
#define PGALP_NORMAL       0x00

// Gain of PGA.  Selectable from 1 to 64.

#define PGAG_1             0x00
#define PGAG_2             0x01
#define PGAG_4             0x02
#define PGAG_8             0x03
#define PGAG_16            0x04
#define PGAG_32            0x05
#define PGAG_64            0x06

/////////// The above values fit into the structure below to select ADC/PGA
/////////// configuration desired:

//ADת�������ýṹ��
typedef struct adc_cfg
{
    uint8_t  CONFIG1;         // ADC configuration register 1     ���üĴ���1
    uint8_t  CONFIG2;         // Configuration register 2         ���üĴ���2
    uint16_t COMPARE1;        // Compare value registers 1        �Ƚ�ֵ�Ĵ���1
    uint16_t COMPARE2;        // Compare value registers 2        �Ƚ�ֵ�Ĵ���2
    uint8_t  STATUS2;         // Status and control register 2    ״̬���ƼĴ���2
    uint8_t  STATUS3;         // Status and control register 3
    uint8_t  STATUS1A;        // ADC status and control registers 1 Aͨ��
    uint8_t  STATUS1B;        // ADC status and control registers 1 Bͨ��
    uint32_t PGA;             // ADC PGA register
} *tADC_ConfigPtr, tADC_Config ;


#define CAL_BLK_NUMREC 18

typedef struct adc_cal
{

    uint16_t  OFS;
    uint16_t  PG;
    uint16_t  MG;
    uint8_t   CLPD;
    uint8_t   CLPS;
    uint16_t  CLP4;
    uint16_t  CLP3;
    uint8_t   CLP2;
    uint8_t   CLP1;
    uint8_t   CLP0;
    uint8_t   dummy;
    uint8_t   CLMD;
    uint8_t   CLMS;
    uint16_t  CLM4;
    uint16_t  CLM3;
    uint8_t   CLM2;
    uint8_t   CLM1;
    uint8_t   CLM0;
} tADC_Cal_Blk ;




///////////////////////////����Ϊ���ݴ�ѧ���ӵ�////////////////////////////////////
#define ADC1_CHANA    26


#define ADC0_DLYA     0x2000                                // ADC0 ����A�ӳ�
#define ADC0_DLYB     0x4000                                // ADC0 ����B�ӳ�
#define ADC1_DLYA     0x6000                                // ADC1 ����A�ӳ�
#define ADC1_DLYB     0x7fff                                // ADC1 ����B�ӳ�


#define ADC0A_DONE   0x01
#define ADC0B_DONE   0x02
#define ADC1A_DONE   0x04
#define ADC1B_DONE   0x08

/////////////////////////����Ϊ���ݴ�ѧ���ӵ�////////////////////////////////

/*       ADC��DAC PGA ͨ���궨��        */
#define DP0       DAD0 			//	ADC0				ADC1
#define DP1       DAD1 			//	ADC0				ADC1
#define DP        DAD2 			//	PGA0				PGA1
#define DP3       DAD3 			//	ADC0				ADC1
#define SE4a      AD4a 			//					ADC1
#define SE5a      AD5a 			//					ADC1
#define SE6a      AD6a 			//					ADC1
#define SE7a      AD7a 			//					ADC1
#define SE4b      AD4b 			//	ADC0				ADC1
#define SE5b      AD5b 			//	ADC0				ADC1
#define SE6b      AD6b 			//	ADC0				ADC1
#define SE7b      AD7b 			//	ADC0				ADC1
#define SE8       AD8  			//	ADC0  				ADC1
#define SE9       AD9  			//	ADC0  				ADC1
#define SE10      AD10 			//	ADC0 				ADC1
#define SE11      AD11 			//	ADC0 				ADC1
#define SE12      AD12 			//	ADC0 				ADC1
#define SE13      AD13 			//	ADC0 				ADC1
#define SE14      AD14 			//	ADC0 				ADC1
#define SE15      AD15 			//	ADC0 				ADC1
#define SE16      AD16 			//	ADC0				ADC1
#define SE17      AD17 			//	ADC0				ADC1
#define SE18      AD18 			//	ADC0
#define DM0       AD19 			//	ADC0				ADC1
#define DM1       AD20 			//	ADC0				ADC1 			
#define OUT       AD23 			//	DAC0				DAC1
#define Temperaturensor  AD26 		//  �ڲ��¶ȴ�����                 �ڲ��¶ȴ�����
#define Bandgap   AD27 			//  ��϶��ѹ�ο�                    ��϶��ѹ�ο�
#define Module_disabled    AD31         //   ����ģ��

static void adc_config_alt(ADC_MemMapPtr adcmap, tADC_ConfigPtr ADC_CfgPtr);//��adc�Ĵ����ṹ�����ý�adc�Ĵ���

#endif /* __ADC16_H__ */