#include "ENC28J60.h"

static uint16_t EthRXBufferEnd=0;
static uint8_t ControlByte=0;

/*
*/
/* Function briefing: readPair: Read the 16 bit data distributed in two continuous 8 bit registers:THIS FUNCTION DOESNT TAKE IN ACCOUNT THE BANK YOU'RE ACTUALLY ON
*@rgIn: Enum:dir <-> Low register of the couple
*
*retval: short of with the data
*
*/
uint16_t readPair(Regs dir)
{
	
	return (uint16_t)readControlRegister(dir)|((uint16_t)readControlRegister((Regs)(dir+1)))<<8;
}
/*
*/
/* Function briefing writePair: Write 16 bits to two continuous 8 bit registers:THIS FUNCTION DOESNT TAKE IN ACCOUNT THE BANK YOU'RE ACTUALLY ON
*@rgIn: Enum:dir <-> Low register of the couple
*
*retval: void
*
*
*/
void writePair(Regs dir,uint16_t val)
{
	writeControlRegister(dir,((uint8_t)val));
	
	writeControlRegister((Regs)(dir+1),(uint8_t)(val>>8));

}
/*
*/
/* Function briefing: Write a byte to a register :THIS FUNCTION DOESNT TAKE IN ACCOUNT THE BANK YOU'RE ACTUALLY ON
*@rgIn: Enum:Register Direction
*
*retval: void
*
*/
void writeControlRegister(Regs dir,uint8_t val)
{
uint8_t Tbuff[2];
	
	Tbuff[0]=0x40|(dir&0x1f);
	Tbuff[1]=val;
	_ENC28J60_EnableEth();
  _ENC28J60_SPITransfer(1,Tbuff);
	 _ENC28J60_SPITransfer(1,Tbuff+1);
	_ENC28J60_DisableEth();
	
}
/*
*/
/* Function briefing: Read a byte from a register :THIS FUNCTION DOESNT TAKE IN ACCOUNT THE BANK YOU'RE ACTUALLY ON
*@rgIn: Enum:Register Direction
*
*retval:void
*
*/
uint8_t readControlRegister(Regs dir)
{
uint8_t Tbuff[1];
	uint8_t res;
	Tbuff[0]=0x1f&dir;
	_ENC28J60_EnableEth();
	_ENC28J60_SPITransfer(1,Tbuff);
	
	if((dir>0x39&&dir<0x86)||dir==0x8A)//MM MI dummy byte
		_ENC28J60_SPIReceive(1,&res);
	
	_ENC28J60_SPIReceive(1,&res);
	_ENC28J60_DisableEth();
	
	return res;
}
/*
*/
/* Function briefing: Clear bits from a register :THIS FUNCTION DOESNT TAKE IN ACCOUNT THE BANK YOU'RE ACTUALLY ON
*@rgIn:  Enum:Register Direction
*@rgIn: uint_8t mask: Bits which will be cleared on the register
*
*retval:void
*
*/
void clearBits(Regs dir,uint8_t mask)
{
	uint8_t Tbuff[2];
	Tbuff[0]=0xA0|(0x1f&dir);
	Tbuff[1]=mask;
	_ENC28J60_EnableEth();
  _ENC28J60_SPITransfer(1,Tbuff);
  _ENC28J60_SPITransfer(1,Tbuff+1);
	_ENC28J60_DisableEth();
	
}
/*
*/
/* Function briefing: Set bits from a register :THIS FUNCTION DOESNT TAKE IN ACCOUNT THE BANK YOU'RE ACTUALLY ON
*@rgIn:  Enum:Register Direction
*@rgIn: uint_8t mask: Bits which will be set on the register
*
*retval:void
*
*/
void setBits(Regs dir,uint8_t mask)
{
	uint8_t Tbuff[2];
	Tbuff[0]=0x80|(0x1f&dir);
	Tbuff[1]=mask;
	_ENC28J60_EnableEth();
	_ENC28J60_SPITransfer(1,Tbuff);
  _ENC28J60_SPITransfer(1,Tbuff+1);
	_ENC28J60_DisableEth();
	
}
/*
*/
/* Function briefing: Realize a software reset, for details see datasheet
*@rgIn:void
*
*
*retval: void
*
*/
void softReset()
{
	uint8_t Tbuff[1];
	Tbuff[0]=0xff;
	_ENC28J60_EnableEth();
	_ENC28J60_SPITransfer(1,Tbuff);
	_ENC28J60_DisableEth();
	
}
/*
*/
/* Function briefing: Read from the chip data buffer
*@rgIn: uint16_t bytes: Number of bytes you're gonna read
*@rgIn: destination: Pointer to the first byte where we are gona write
*
*retval: void
*
*/
void readBufferMemory(uint16_t bytes,void* destination)
{
	uint8_t *d2=destination;
	uint8_t Tbuff[1];
	Tbuff[0]=0x3A;
	_ENC28J60_EnableEth();
	_ENC28J60_SPITransfer(1,Tbuff);
	_ENC28J60_SPIReceive(bytes,d2);
	_ENC28J60_DisableEth();
}
/*
*/
/* Function briefing: write to the chip data buffer
*@rgIn: uint16_t bytes: Number of bytes you're gonna write
*@rgIn: destination: Pointer to the first byte where we are gona read from
*
*retval: void
*
*/
void writeBufferMemory(uint16_t bytes,void* source)
{

	uint8_t *s2=source;
	uint8_t Tbuff[1];
	Tbuff[0]=0x7A;
	_ENC28J60_EnableEth();
	_ENC28J60_SPITransfer(1,Tbuff);
	_ENC28J60_SPITransfer(bytes,s2);
	_ENC28J60_DisableEth();
}
/*
*/
/* Function briefing: change to another grup of registers
*@rgIn: Enum:bank The bank we're selecting
*
*retval:void
*
*/
void ethSetBank(Eth_Bank b)
{
clearBits(ECON1,0x3);
setBits(ECON1,b);
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void ethSetMACAddr(uint8_t* MediaAccessControlAddress)
{
	ethSetBank(EthBank3);
	writeControlRegister(MADDR5,MediaAccessControlAddress[0]); 
	writeControlRegister(MADDR6,MediaAccessControlAddress[1]); 
	writeControlRegister(MADDR3,MediaAccessControlAddress[2]); 
	writeControlRegister(MADDR4,MediaAccessControlAddress[3]); 
	writeControlRegister(MADDR1,MediaAccessControlAddress[4]); 
	writeControlRegister(MADDR2,MediaAccessControlAddress[5]); 
	
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void writePhyReg(PRegs addr,uint16_t val)
{
	ethSetBank(EthBank2);
	writeControlRegister(MIREGADR,addr);
	writeControlRegister(MIWRL,val);
	writeControlRegister(MIWRH,(val>>8));
	ethSetBank(EthBank3);
	while(readControlRegister(MISTAT)&0x1);
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
uint16_t readPhyReg(PRegs addr)
{
	ethSetBank(EthBank2);
	writeControlRegister(MIREGADR,addr);
	writeControlRegister(MICMD,0x1);
	ethSetBank(EthBank3);
	while(readControlRegister(MISTAT)&0x1)
	{};
	ethSetBank(EthBank2);
	clearBits(MICMD,0x1);

		
		return readPair(MIRDL);
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void enterPHYScanMode(PRegs regToScan)
{
	ethSetBank(EthBank2);
	writeControlRegister(MIREGADR,regToScan);
	writeControlRegister(MICMD,0x2);
	ethSetBank(EthBank3);
	while(readControlRegister(MISTAT)&4);
	
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
uint16_t readPhyRegScan()
{
	ethSetBank(EthBank2);
	return readPair(MIRDL);
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void exitPHYScanMode()
{
	writeControlRegister(MICMD,0x0);

}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void ethInit(Eth_Filters filt,Eth_IRQflags IRQFlags,Eth_DMAFunction DMAf,Eth_MACON1Conf MAC1,Eth_MACON3Conf MAC3,Eth_MACON4Conf MAC4,uint16_t MaxFrameLength,Eth_PHCON1 ph1c,Eth_PHCON2 ph2c)
{

	ethPowerUp();
	ethSetFilters(filt);
	ethSetInterrupts(IRQFlags);		
	ethInitRXBufferSize(EthRXBufferEnd); 
	setBits(ECON1,0x10*DMAf);
	ethSetBank(EthBank2);
	writePair(MAMXFLL,MaxFrameLength?MaxFrameLength:1518);
	setFlowControl(Eth_FlowControl_FlowOff);
	
	if(ph1c)
		writePhyReg(PHCON1,ph1c);
	
	writePhyReg(PHCON2,ph2c);
	ethSetBank(EthBank2); 
	
	writeControlRegister(MACON1,MAC1);//Set MACON1 
	writeControlRegister(MACON3,MAC3); //Set MACON3 Half Dup
	writeControlRegister(MACON4,MAC4); //Set MACON4-> defer
	
	writeControlRegister(MABBIPG,0x12+((MAC3&Eth_MACON3Conf_FullDupleOperation)?3:0)); //Set MAIPG
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void confLeds(Eth_LEDA la,Eth_LEDB lb)
{
	writePhyReg(PHLCON,la|lb);
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
uint8_t ethGetStatus()
{
	return readControlRegister(ESTAT);
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void ethPowerDown()
{
	ENC28J60Eth_RXOFF();
	while(ethGetStatus()&Eth_STAT_RXbusy);
	ENC28J60_WaitTransmitEnd();
	setBits(ECON2,0x08);
	setBits(ECON2,0x20);
	
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void ethPowerUp()
{
	clearBits(ECON2,0x20);
	delayMicro(50);
	while(!(ethGetStatus()&Eth_STAT_CLKRDY));
	clearBits(ECON2,0x8);
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void ethClockPinOutput(Eth_ClockPinOutputDivider div)
{
	ethSetBank(EthBank3);
	writeControlRegister(ECOCON,div);
}/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void setFlowControl(Eth_FlowControl fc) 
{
	ethSetBank(EthBank3);
	writeControlRegister(EFLOCON,fc);
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void DMACopyMemory(uint16_t start,uint16_t end,uint16_t destination)
{
	while(readControlRegister(ECON1)&0x20){};
	clearBits(ECON1,0x10);
	ethSetBank(EthBank0);
	writePair(EDMASTL,start);
	writePair(EDMANDL,end);
	writePair(EDMADSTL,destination);
	setBits(ECON1,0x20);
	
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void DMACalculateChecksum(uint16_t start,uint16_t end)
{
	ethSetBank(EthBank0);
	while(readControlRegister(ECON1)&0x20){};
	writePair(EDMASTL,start);
	writePair(EDMANDL,end);
	setBits(ECON1,0x10);
	setBits(ECON1,0x20);
	
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
uint16_t DMAGetCheckSum(uint16_t start,uint16_t end)
{
	ethSetBank(EthBank0);
return readPair(EDMACSL);
	
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void ethSetInterrupts(Eth_IRQflags InterruptFlags)
{
	ENC28J60_IRQ_GlobalDissable();
	if(InterruptFlags&Eth_IRQflag_LinkStatusChange)
	{
		writePhyReg(PHIE,0x0012);
	}
	writeControlRegister(EIE,InterruptFlags);
	
	
	
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void ethSetFilters(Eth_Filters f)
{
	ethSetBank(EthBank1);
	writeControlRegister(ERXFCON,f);
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
uint8_t newPackets(void)
{
	ethSetBank(EthBank1);
	return readControlRegister(EPKTCNT);
	
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void decreasePacketCount()
{
	setBits(ECON2,0x40);
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void  ethInitRXBufferSize(uint16_t bytesl)
{
	ENC28J60Eth_RXOFF();
	EthRXBufferEnd=bytesl;
	ethSetBank(EthBank0); //All addresses for Eth buff are here (Mostly i think :) )
	writePair(ERDPTL,0);
	writePair(ERXSTL,0);//Transmision Buffer start
	
	EthRXBufferEnd=EthRXBufferEnd?EthRXBufferEnd:DefaultEthRXBufferEnd;
	writePair(ERXNDL,EthRXBufferEnd);//Transmission Buffer end
	writePair(ERXRDPTL,EthRXBufferEnd+1);
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
uint16_t  readFrame(struct RXstatusVector *res,void* destination)
{
uint16_t nextPacketPointer,byteCount;
uint8_t Rbuff[4];
readBufferMemory(2,Rbuff);
nextPacketPointer=((uint16_t)Rbuff[0])|(((uint16_t)Rbuff[1])<<8);
readBufferMemory(4,Rbuff);
if(res)
{
	res->data[0]=Rbuff[0];
	res->data[1]=Rbuff[1];	
	res->data[2]=Rbuff[2];
	res->data[3]=Rbuff[3];
}

byteCount=*((uint16_t*)(Rbuff));
readBufferMemory(byteCount,destination);
ethSetBank(EthBank0);
writePair(ERDPTL,nextPacketPointer);
writePair(ERXRDPTL,nextPacketPointer-1);
return byteCount;
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void ethRXRestartRoutine()
{
	setBits(ECON1,0x40);
	clearBits(ECON1,0x40);
	ethInitRXBufferSize(EthRXBufferEnd);
	ENC28J60Eth_RXON();
}/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void ethTXRestartRoutine()
{
	setBits(ECON1,0x80);
	clearBits(ECON1,0x80);
}
/*
*/
/* Function briefing:
*@rgIn:
*
*
*
*
*
*
*
*retval:
*
*/
void writeFrame(uint16_t length,struct TXstatusVector *res,void* source)
{
	
	ethSetBank(EthBank0);
	while(readControlRegister(ECON1)&0x08){};
	writePair(EWRPTL,EthRXBufferEnd+1);
	writeBufferMemory(1,&ControlByte);
	writeBufferMemory(length,source);
	writePair(ETXSTL,EthRXBufferEnd+1);
	writePair(ETXNDL,EthRXBufferEnd+1+length);//write control byte
	setBits(ECON1,0x08);
	if(res)
	{
		uint16_t temporal=readPair(ERDPTL);
		writePair(ERDPTL,readPair(EWRPTL));
		while(readControlRegister(ECON1)&0x08){};
			
			readBufferMemory(7,res->data);
			writePair(ERDPTL,temporal);
	}
}
