/* NRF24L01.c */
#include "Drv_nrf24l01.h"
#include "include.h"
#include "Drv_spi.h"
#include "stm32f4xx.h"
/* global buffer */
const u8 TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ
const u8 RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ

/*
//��ʼ��24L01��IO��
void NRF24L01_Init(void)
{
	RCC->APB2ENR|=1<<2;    //ʹ��PORTA��ʱ�� 
	RCC->APB2ENR|=1<<4;    //ʹ��PORTC��ʱ�� 
	GPIOA->CRL&=0XFFF000FF;//PA4���
	GPIOA->CRL|=0X00033300; 
	GPIOA->ODR|=7<<2;	   //PA2.3.4 ���1		 
	GPIOC->CRL&=0XFF00FFFF;//PC4��� PC5���
	GPIOC->CRL|=0X00830000; 
	GPIOC->ODR|=3<<4;	   //����	 
	SPIx_Init();    //��ʼ��SPI
	NRF24L01_CE=0; 	//ʹ��24L01
	NRF24L01_CSN=1;	//SPIƬѡȡ��		  		 		  
}
*/


/* 
	pin allocate scheme:
	NRF24L01 CE: 	GPIOD_Pin_1 
	NRF24L01 CSN: 	GPIOC_Pin 13 
	NRF24L01_IRQ : 	GPIOD_Pin 2
*/
void Drv_NRF24L01CSPin_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
	
	/* initilize GPIOC_Pin_13 */
    GPIO_InitStructure.GPIO_Pin = NRF24_CSN_PIN;			//GPIOC_Pin_13
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init ( NRF24_CS_GPIO, &GPIO_InitStructure );
	
    GPIO_SetBits ( NRF24_CS_GPIO, NRF24_CSN_PIN );		// GPIOC_Pin13
	
	/* initialize GPIOD_Pin_1 and GPIOD_Pin_2
	NRF24_CE=0; 	//ʹ��24L01
	NRF24_CSN=1;	//SPIƬѡȡ��
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;	//GPIOD_Pin_1 GPIOD_Pin_2 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOD, GPIO_Pin_2);	/* NRF24_IRQ:GPIOD_Pin_2 = 1 �ж����ų�ʼ�ߵ�ƽ*/
	GPIO_ResetBits(GPIOD, GPIO_Pin_1);	/* NRF24_CE:GPIOD_Pin_1 =0,ʹ��24L01 */
}
/* ѡ��NRF24L01 NRF24L01 CSN: 	GPIOC_Pin 13 */
static void nrf24_enable ( u8 ena )
{
    if ( ena )
        GPIO_ResetBits ( NRF24_CS_GPIO, NRF24_CSN_PIN );
    else
        GPIO_SetBits ( NRF24_CS_GPIO, NRF24_CSN_PIN );
}


//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;
	//SPIx_SetSpeed(SPI_SPEED_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   	 
	NRF24L01_Write_Buf(WRITE_REG_CMD+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//���24L01����	
	return 0;		 //��⵽24L01
}	 	 
//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
	
    //NRF24_CSN_PIN = 0;
	GPIO_ResetBits(GPIOD, NRF24_CSN_PIN);
  	status =Drv_SPI2_RW(reg);	//���ͼĴ����� 
  	Drv_SPI2_RW(value);      	//д��Ĵ�����ֵ
  	//NRF24_CSN_PIN = 1;
	GPIO_SetBits(GPIOD,NRF24_CSN_PIN);
  	return(status);       			//����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	GPIO_ResetBits(GPIOD, NRF24_CSN_PIN);		
  	Drv_SPI2_RW(reg);   //���ͼĴ�����
  	reg_val=Drv_SPI2_RW(0XFF);//��ȡ�Ĵ�������
  	GPIO_SetBits(GPIOD,NRF24_CSN_PIN);          //��ֹSPI����		    
  	return(reg_val);           //����״ֵ̬
}	
//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;	       
  	GPIO_ResetBits(GPIOD, NRF24_CSN_PIN);    //ʹ��SPI����
  	status=Drv_SPI2_RW(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
	pBuf[u8_ctr]=Drv_SPI2_RW(0);//��������
  	GPIO_SetBits(GPIOD,NRF24_CSN_PIN);       //�ر�SPI����
  	return status;        //���ض�����״ֵ̬
}

//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	GPIO_ResetBits(GPIOD, NRF24_CSN_PIN);           //ʹ��SPI����
  	status = Drv_SPI2_RW(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++) Drv_SPI2_RW(*pBuf++); //д������	 
  	GPIO_SetBits(GPIOD,NRF24_CSN_PIN);       //�ر�SPI����
  	return status;          //���ض�����״ֵ̬
}

//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 sta;
 	//SPIx_SetSpeed(SPI_SPEED_8);//spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	//NRF24_CE =0;
	GPIO_ResetBits(GPIOD, NRF24_CE);
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 	//NRF24_CE =1;//��������	
	GPIO_SetBits(GPIOD, NRF24_CE);
	while(NRF24_IRQ !=0);//�ȴ��������
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(WRITE_REG_CMD+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&MAX_TX)//�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
	if(sta&TX_OK)//�������
	{
		return TX_OK;
	}
	return 0xff;//����ԭ����ʧ��
}
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    							   
	//SPIx_SetSpeed(SPI_SPEED_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(WRITE_REG_CMD+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&RX_OK)//���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
		return 0; 
	}	   
	return 1;//û�յ��κ�����
}					    
//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
void RX_Mode(void)
{
	GPIO_ResetBits(GPIOD, NRF24_CE);  
  	NRF24L01_Write_Buf(WRITE_REG_CMD+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
	  
  	NRF24L01_Write_Reg(WRITE_REG_CMD+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(WRITE_REG_CMD+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  	 
  	NRF24L01_Write_Reg(WRITE_REG_CMD+RF_CH,40);	     //����RFͨ��Ƶ��		  
  	NRF24L01_Write_Reg(WRITE_REG_CMD+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
  	NRF24L01_Write_Reg(WRITE_REG_CMD+RF_SETUP,0x0f);//����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(WRITE_REG_CMD+CONFIG, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  	//NRF24_CE = 1; //CEΪ��,�������ģʽ 
	GPIO_SetBits(GPIOD, NRF24_CE);
}						 
//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.	 
void TX_Mode(void)
{														 
	//NRF24_CE = 0;	 
	GPIO_ResetBits(GPIOD, NRF24_CE); 
  	NRF24L01_Write_Buf(WRITE_REG_CMD+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  	NRF24L01_Write_Buf(WRITE_REG_CMD+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

  	NRF24L01_Write_Reg(WRITE_REG_CMD+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(WRITE_REG_CMD+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
  	NRF24L01_Write_Reg(WRITE_REG_CMD+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	NRF24L01_Write_Reg(WRITE_REG_CMD+RF_CH,40);       //����RFͨ��Ϊ40
  	NRF24L01_Write_Reg(WRITE_REG_CMD+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(WRITE_REG_CMD+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	//NRF24_CE = 1;//CEΪ��,10us����������
	GPIO_SetBits(GPIOD, NRF24_CE);
}		  





