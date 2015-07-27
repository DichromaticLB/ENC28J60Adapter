

#ifndef ENC28J60MAR
#define ENC28J60MAR
#include "USER_DEF_ENC28J60.h"


//CONSTANTS 

#define DefaultEthRXBufferEnd (uint16_t)0x0fff



//MACROS
#define ENC28J60Eth_RXON()  setBits(ECON1,0x04)
#define ENC28J60Eth_RXOFF() clearBits(ECON1,0x04)

#define ENC28J60_WaitTransmitEnd() 		while(readControlRegister(ECON1)&0x08){}
#define ENC28J60_WaitDMABusy() 		while(readControlRegister(ECON1)&0x20){}

#define ENC28J60_PHYLinkChanged() (readPhyReg(PHSTAT1)&0x4)
#define ENC28J60_JabberingHappened() (readPhyReg(PHSTAT1)&0x2)
#define ENC28J60_PHYLinkIsUp() (readPhyReg(PHSTAT2)&0x0400)
#define ENC28J60_PHYCollisionHappening() (readPhyReg(PHSTAT2)&0x0800)
#define ENC28J60_PHYTransmitinp() (readPhyReg(PHSTAT2)&0x2000)
#define ENC28J60_PHYReceiving() (readPhyReg(PHSTAT2)&0x1000)

//******* Iterrupt related macros
#define ENC28J60_IRQ_GlobalEnable()  		 setBits(EIE,0x80)
#define ENC28J60_IRQ_GlobalDissable() 		 clearBits(EIE,0x80)	
#define ENC28J60_IRQ_PendingInterrupt() (readControlRegister(ESTAT)&0x80)



//Enums

typedef enum 	      
{
EthBank0	= 0, 
EthBank1	= 1, 
EthBank2	= 2, 
EthBank3	= 3
}Eth_Bank;

typedef enum 	      
{

ERDPTL=0x0, 
ERDPTH=0x1, 
EWRPTL=0x2, 
EWRPTH=0x3,
ETXSTL=0x4,
ETXSTH=0x5,
ETXNDL=0x6,
ETXNDH=0x7,
ERXSTL=0x8,
ERXSTH=0x9,
ERXNDL=0xA,
ERXNDH=0xB,
ERXRDPTL=0xC,
ERXRDPTH=0xD,
ERXWRPTL=0xE,
ERXWRPTH=0xF,
EDMASTL=0x10,
EDMASTH=0x11,	
EDMANDL=0x12,
EDMANDH=0x13,
EDMADSTL=0x14,
EDMADSTH=0x15,
EDMACSL=0x16,
EDMACSH=0x17,
EIE=0x1B,
EIR=0x1C,
ESTAT=0x1D,
ECON2=0x1E,
ECON1=0x1F,
ETH0	= 0x10, 
ETH1= 0x21, 
ETH2= 0x22, 
ETH3= 0x23,
ETH4=0x24,
ETH5=0x25,
ETH6=0x26,
ETH7=0x27,
EPMM0=0x28,
EPMM1=0x29,
EPMM2=0x2A,
EPMM3=0x2B,
EPMM4=0x2C,
EPMM5=0x2D,
EPMM6=0x2E,
EPMM7=0x2F,
EPMCSL=0x30,
EPMCSH=0x31,
EPMMOL=0x34,
EPMMOH=0x35,
ERXFCON=0x38,
EPKTCNT=0x39,
MACON1=0x40,
MACON3=0x42,
MACON4=0x43,
MABBIPG=0x44,
MAIPGL=0x46,
MAIPGH=0x47,
MACLCON1=0x48,
MACLCON2=0x49,
MAMXFLL=0x4A,
MAMXFLH=0x4B,
MICMD=0x52,
MIREGADR=0x54,
MIWRL=0x56,
MIWRH=0x57,
MIRDL=0x58,
MIRDH=0x59,
MADDR5=0x80,
MADDR6=0x81,
MADDR3=0x82,
MADDR4=0x83,
MADDR1=0x84,
MADDR2=0x85,
EBSTSD=0x86,
EBSTCON=0x87,
EBSTCSL=0x88,
EBSTCSH=0x89,
MISTAT=0x8A,
EREVID=0x92,
ECOCON=0x95,
EFLOCON=0x97,
EPAUSL=0x98,
EPAUSH=0x99,
}Regs;


typedef enum 	      
{
PHCON1=0x0,
PHSTAT1=0x01,
PHID1=0x02,
PHID2=0x03,
PHCON2=0x10,
PHSTAT2=0x11,
PHIE=0x12,
PHIR=0x13,
PHLCON=0x14
}PRegs;


typedef enum 	      
{
	Eth_Filter_PROMISCUOUS=0x0,
	Eth_Filter_Unicast=0x80,
	Eth_Filter_AND=0x40,
	Eth_Filter_OR=0x00,
	Eth_Filter_CRC=0x20,
	Eth_Filter_Patter=0x10,
  Eth_Filter_magic_Packet=0x08,
	Eth_Filter_Hash=0x04,
	Eth_Filter_multicast=0x02,
	Eth_Filter_broadcast=0x01
}Eth_Filters;

typedef enum 	      
{
	Eth_IRQflag_EnablePin=0x80,
	Eth_IRQflag_ReceivePacketPending=0x40,
	Eth_IRQflag_DMA=0x20,
	Eth_IRQflag_LinkStatusChange=0x10,
	Eth_IRQflag_TransmitEnable=0x08,
	Eth_IRQflag_TransmitError=0x02,
  Eth_IRQflag_ReceiveError=0x01
}Eth_IRQflags;

typedef enum 	      
{
	Eth_STAT_IRQPending=0x80,
	Eth_STAT_BufferError=0x40,
	Eth_STAT_LateCollision=0x10,
	Eth_STAT_RXbusy=0x4,
	Eth_STAT_TransmissionAborted=0x2,
  Eth_STAT_CLKRDY=0x01
}Eth_STAT;

typedef enum 	      
{
	
  Eth_DMAFunctionCheckSum=0x01,
	Eth_DMAFunctionCopyData=0x0
}Eth_DMAFunction;

typedef enum 	      
{

  Eth_MACON1Conf_AllowPauseFramesTX=0x08,
	Eth_MACON1Conf_RespectPauseFrames=0x04,
	Eth_MACON1Conf_AcceptMACControlFrames=0x02,
	Eth_MACON1Conf_EnablePacketReception=0x01
}Eth_MACON1Conf;

typedef enum 	      
{
	Eth_MACON3Conf_PAD64B_CRC=0xE0,
	Eth_MACON3Conf_DetectVLANPad=0xA0,
	Eth_MACON3Conf_PAD60B_CRC=0x20,
  Eth_MACON3Conf_MACAppendCRC=0x10,
	Eth_MACON3Conf_JESUSCHRISTWATISTHISIDONTEVEN=0x08,
	Eth_MACON3Conf_AllowAnyFrameMaxSize=0x04,
	Eth_MACON3Conf_ReportInVectorTipe_LengthMissmatches=0x2,
	Eth_MACON3Conf_FullDupleOperation=0x1,
}Eth_MACON3Conf;

typedef enum 	      
{

  Eth_MACON4Conf_HalfDuplexWaitForMediumFree=0x40,
	Eth_MACON4Conf_HalfDuplexIfAbortMediumOcupied=0x0,
	Eth_MACON4Conf_InstantRetransmisionIfCollisionMINE=0x20,
	Eth_MACON4Conf_InstantRetransmisionIfCollisionANY=0x10
}Eth_MACON4Conf;
typedef enum 	      
{

  Eth_ClockPinOutputDivider_8=0x05,
	Eth_ClockPinOutputDivider_4=0x04,
	Eth_ClockPinOutputDivider_3=0x03,
	Eth_ClockPinOutputDivider_2=0x02,
	Eth_ClockPinOutputDivider_1=0x01,
	Eth_ClockPinOutputDivider_PINOFF=0
}Eth_ClockPinOutputDivider;

typedef enum 	      
{

  Eth_FlowControl_FullDupSendOnePauseFrameTime0=0x03,
	Eth_FlowControl_FullDupSendFramesPeriodically=0x02,
	Eth_FlowControl_FullDupSendOnePauseFrame=0x01,
	Eth_FlowControl_HalfDupFlowOn=0x01,
	Eth_FlowControl_FlowOff=0x0
}Eth_FlowControl;

typedef enum 	      
{

  Eth_PHCON2_ForceLinkup=0x4000,
	Eth_PHCON2_DissableTwistedPair=0x2000,
	Eth_PHCON2_DissableJabberCorrection=0x0400,
	Eth_PHCON2_SendDataDirectlyToTwistedPair=0x0100
}Eth_PHCON2;

typedef enum 	      
{

  Eth_PHCON1_LoopBackAllDataToMAC=0x4000,
	Eth_PHCON1_ShutDownPHY=0x0800,
	Eth_PHCON1_FullDuplexOperation=0x0100
}Eth_PHCON1;

typedef enum 	      
{

 Eth_LEDA_DisplayDuplexCollision=0x3E00,
 Eth_LEDA_DisplayLinkStatusTR=0x3D00,
 Eth_LEDA_DisplayLinkStatusR=0x3C00,
 Eth_LEDA_BlinkSlow=0x3B00,
 Eth_LEDA_BlinkFast=0x3A00,
 Eth_LEDA_Off=0x3900,
 Eth_LEDA_On=0x3800,
 Eth_LEDA_TR=0x3700,
 Eth_LEDA_DuplexStatus=0x3500,
 Eth_LEDA_LinkStatus=0x3400,
 Eth_LEDA_DisplayCollision=0x3300,
 Eth_LEDA_DisplayR=0x3200,
 Eth_LEDA_DisplayT=0x3100
}Eth_LEDA;

typedef enum 	      
{

 Eth_LEDB_DisplayDuplexCollision=0x30E0,
 Eth_LEDB_DisplayLinkStatusTR=0x30D0,
 Eth_LEDB_DisplayLinkStatusR=0x30C0,
 Eth_LEDB_BlinkSlow=0x30B0,
 Eth_LEDB_BlinkFast=0x30A0,
 Eth_LEDB_Off=0x3090,
 Eth_LEDB_On=0x3080,
 Eth_LEDB_TR=0x3070,
 Eth_LEDB_DuplexStatus=0x3050,
 Eth_LEDB_LinkStatus=0x3040,
 Eth_LEDB_DisplayCollision=0x3030,
 Eth_LEDB_DisplayR=0x3020,
 Eth_LEDB_DisplayT=0x3010
}Eth_LEDB;






//SPI Operation
void delayMicro(uint32_t time);
void writeControlRegister(Regs dir,uint8_t val);
uint8_t readControlRegister(Regs dir);
void clearBits(Regs dir,uint8_t mask);
void setBits(Regs dir,uint8_t mask);
void readBufferMemory(uint16_t,void* destination);
void writeBufferMemory(uint16_t,void* source);
void softReset(void);
uint16_t readPhyReg(PRegs  addr);
void writePhyReg(PRegs addr,uint16_t val);
void ethSetBank(Eth_Bank);
uint16_t readPair(Regs dir);
void writePair(Regs dir,uint16_t val);
void enterPHYScanMode(PRegs);
void exitPHYScanMode(void);
uint16_t readPhyRegScan(void);

//Initialization
void ethSetMACAddr(uint8_t* MACADDR);
void ethInit(Eth_Filters,Eth_IRQflags,Eth_DMAFunction,Eth_MACON1Conf,Eth_MACON3Conf,Eth_MACON4Conf,uint16_t MaxFrameLength,Eth_PHCON1,Eth_PHCON2);
void ethInitRXBufferSize(uint16_t bytesl);
void ethSetFilters(Eth_Filters);
void ethSetInterrupts(Eth_IRQflags);
Eth_IRQflags ethGetFlags(void);
void confLeds(Eth_LEDA,Eth_LEDB);

//Status
uint8_t ethGetStatus(void);

//Power Management
void ethPowerDown(void);
void ethPowerUp(void);
void ethClockPinOutput(Eth_ClockPinOutputDivider);

//DMA routines
void DMACopyMemory(uint16_t start,uint16_t end,uint16_t destination);
void DMACalculateChecksum(uint16_t start,uint16_t end);
uint16_t DMAGetCheckSum(uint16_t start,uint16_t end);

// Routines
uint8_t newPackets(void); 
void decreasePacketCount(void);
void setFlowControl(Eth_FlowControl);
uint16_t readPacket(struct RXstatusVector *res,void* destination);
void writePacket(uint16_t length,struct TXstatusVector *res,void* source);
void ethRXRestartRoutine(void);
void ethTXRestartRoutine(void);
#endif



