#include "sd.h"			   

					   
u8 SD_Type = 0;	//SD卡的类型 

/************************SPI相关移植修改************************/
/**
  * SD卡SPI读写函数
  * data:要写入的数据
  * 返回值:读到的数据
  */
u8 SD_SPI_ReadWriteByte(u8 data)
{

	return SPI3_ReadWriteByte(data);
}

/**
  * 设置SD卡的SPI为低速模式 
  * SD卡初始化的时候,需要低速
  * 在卡初始化的时候,CLK时钟最大不能超过400KHz！！！
  */
void SD_SPI_SpeedLow(void)
{
 	SPI3_SetSpeed(SPI_BaudRatePrescaler_256);	//设置到低速模式(281.25KHz)	
}

/**
  * 设置SD卡的SPI为高速模式
  * SD卡正常工作的时候,可以高速了（18MHz）
  */
void SD_SPI_SpeedHigh(void)
{
 	SPI3_SetSpeed(SPI_BaudRatePrescaler_4);		//设置到高速模式(18MHz)	
}



/**
  * 取消片选函数
  * 取消片选后,多发8个CLK是为了使SD卡完成一些内部操作
  */
void SD_DisSelect(void)
{
	SD_CS_DISABLE();
 	SD_SPI_ReadWriteByte(0xFF);	//提供额外的8个时钟
}

/**
  * 等待SD卡准备好函数
  * 返回值:0,准备好了;
  *     其他,错误代码;
  */
u8 SD_WaitReady(void)
{
	u32 t = 0;
	
	do
	{
		/* SD卡在写完数据块以后,SD卡内部会拉低MISO线,直到内部编程结束之后才会释放MISO总线 */
		if(SD_SPI_ReadWriteByte(0xFF) == 0xFF)	
			return 0;		//OK
		t++;		  	
	}while(t < 0xFFFFFF);	//等待 
	
	return 1;	//ERROR
}

/**
  * 使能SD卡片选函数
  * 并且等待SD卡准备OK
  * 返回值:0,成功;
  *        1,失败;
  */
u8 SD_Select(void)
{
	SD_CS_ENABLE();
	
	if(SD_WaitReady() == 0)
		return 0;	//等待成功
	
	SD_DisSelect();
	return 1;		//等待失败
}

/**
  * 等待SD卡回应
  * Response:想要得到的回应值
  * 返回值:0,成功得到了该回应值
  *     其他,得到回应值失败
  */
u8 SD_GetResponse(u8 Response)
{
	u16 Count = 0xFFFF;	//等待次数	
	
	while((SD_SPI_ReadWriteByte(0xFF) != Response) && Count)	//等待得到对应的回应 
		Count--; 	  
	if(Count == 0)
		return MSD_RESPONSE_FAILURE;	//得到回应失败   
	else 
		return MSD_RESPONSE_NO_ERROR;	//得到了想要的回应
}

/** 
  * 从sd卡读取一个数据包的内容
  * buf:数据缓存区
  * len:要读取的数据长度.
  * 返回值:0,成功;
  *     其他,失败;	
  */
u8 SD_RecvData(u8*buf, u16 len)
{			  	  
	if(SD_GetResponse(0xFE))	//等待SD卡发回数据起始令牌0xFE
		return 1;				//没等到数据起始令牌0xFE
	
	/* 等到了数据起始令牌0xFE */
    while(len--)	//开始接收数据
    {
        *buf = SPI3_ReadWriteByte(0xFF);
        buf++;
    }
	
    /* 读取2个字节的CRC */
    SD_SPI_ReadWriteByte(0xFF);
    SD_SPI_ReadWriteByte(0xFF);	
	
    return 0;//读取成功
}

/**
  * 向sd卡写入一个数据包（512字节）的内容 
  * buf:数据缓存区
  * cmd:数据令牌
  * 返回值:0,成功;
  *     其他,失败;	
  */
u8 SD_SendBlock(u8* buf, u8 cmd)
{	
	u16 t;	
	
	if(SD_WaitReady())
		return 1;	//SD卡未准备好
	
	/* SD卡已经准备好 */
	SD_SPI_ReadWriteByte(cmd);	//发送令牌    
	if(cmd != 0XFD)	//不是结束指令
	{
		for(t = 0; t < 512; t++)
			SPI3_ReadWriteByte(buf[t]);	//提高速度,减少函数传参时间
		
	    SD_SPI_ReadWriteByte(0xFF);		//发送两个字节的伪crc
	    SD_SPI_ReadWriteByte(0xFF);
		
		t = SD_SPI_ReadWriteByte(0xFF);	//接收响应
		if((t & 0x1F) != 0x05)
			return 2;	//响应错误									  					    
	}
	
    return 0;//写入成功
}

/**
  * 向SD卡发送一条命令函数
  * 输入: u8 cmd   命令索引（最高两位固定为01） 
  *      u32 arg   命令参数（有些命令没有参数）
  *       u8 crc   crc校验值（最低位固定为1）	   
  * 返回值:0xFF,片选失效
  *        其它,SD卡返回的应答数据r1	
  */
u8 SD_SendCmd(u8 cmd, u32 arg, u8 crc)
{
    u8 r1,cmd1 = 0;	
	u8 retry = 0; 
	
	SD_DisSelect();		//取消上次片选
	if(SD_Select())
		return 0xFF;	//片选失效 

	/* 片选成功 */
	cmd1 = cmd;
	cmd1 &= 0x7F; 		//最高位固定为0
	cmd1 |= 0x40;		//次高位固定为1
    SD_SPI_ReadWriteByte(cmd1);			//写入命令索引
	
    SD_SPI_ReadWriteByte(arg >> 24);	//写入命令参数
    SD_SPI_ReadWriteByte(arg >> 16);
    SD_SPI_ReadWriteByte(arg >> 8);
    SD_SPI_ReadWriteByte(arg);	  
	
    SD_SPI_ReadWriteByte(crc | 0x01);	//写入CRC 
	
	if(cmd == CMD12)
		SD_SPI_ReadWriteByte(0xFF);		//在停止读取时,跳过一个字节
	
    /* 等待响应,或超时退出 */
	retry = 0x1F;
	do
	{
		r1 = SD_SPI_ReadWriteByte(0xFF);
	}while((r1 & 0x80) && (retry--));	 
	
	//返回应答数据r1
    return r1;
}	

/**
  * SD卡初始化函数
  * 返回值:0,成功;
  *     其它,失败;
  */
u8 SD_Initialize(void)
{
    u8 r1;      //存放SD卡的返回值
    u16 retry;  //用来进行超时计数
    u8 buf[4];  
	u16 i;
	
 	for(i = 0; i < 10; i++)			//这里发送了80个CLK
		SD_SPI_ReadWriteByte(0xFF);	//发送最少74个CLK（在上电初期,上升到SD卡正常工作电压需要64个CLK,其后的10个CLK是为了与SD同步）
	
	retry = 20;
	do
	{
		r1 = SD_SendCmd(CMD0, 0, 0x95);	//进入IDLE状态
	}while((r1 != 0x01) && (retry--));
	
 	SD_Type = 0;		//默认无卡
	if(r1 == 0x01)		//进入IDLE状态
	{
		/* CMD8指令用于发送SD卡接口条件信息,包括供电电压等;另外只有V2.0或以后的SD卡才会响应该指令,MMC卡和V1.x的卡,是不支持该命令的;故也可以用于区分SD卡版本 */
		if(SD_SendCmd(CMD8, 0x1AA, 0x87) == 1)		//SD V2.0 不需要设置块大小（固定为512字节）
		{
			for(i = 0; i < 4; i++)
				buf[i] = SD_SPI_ReadWriteByte(0xFF);	//Get trailing return value of R7 resp
			if((buf[2] == 0x01) && (buf[3] == 0XAA))	//卡是否支持2.7~3.6V(检查CMD8写入的指令参数和读出的应答是否一致)
			{
				retry = 0xFFFE;
				do
				{
					SD_SendCmd(CMD55, 0, 0X01);					//发送CMD55,告知SD卡下一条是APP CMD
					r1 = SD_SendCmd(CMD41, 0x40000000, 0x01);	//发送CMD41（ACMD指令）,设置HCS = 1,告诉SD卡主机可以支持高容量卡
				}while(r1 && retry--);		//等待SD卡退出空闲状态
				
				
				if(retry == 0)
					SD_Type = SD_TYPE_ERR;	//错误的卡
				else if(SD_SendCmd(CMD58, 0, 0x01) == 0)	//鉴别SD2.0卡版本开始
				{
					for(i = 0; i < 4; i++)
						buf[i] = SD_SPI_ReadWriteByte(0xFF);	//得到OCR值
					if(buf[0] & 0x40)							//检查CCS
						SD_Type = SD_TYPE_V2HC; 				//CCS = 1   	
					else 
						SD_Type = SD_TYPE_V2;					//CCS = 0   
				}
			}
		}
		else	//SD V1.x or MMC V3
		{
			SD_SendCmd(CMD55, 0, 0x01);				//发送CMD55,告知SD卡下一条是APP CMD
			r1 = SD_SendCmd(CMD41, 0, 0x01);		//发送CMD41（ACMD指令）,设置HCS = 0,告诉SD卡主机不支持高容量卡（SD V1.x or MMC V3没有高容量卡）
			if(r1 <= 1)
			{		
				SD_Type = SD_TYPE_V1;
				retry = 0xFFFE;
				do 
				{
					SD_SendCmd(CMD55, 0, 0x01);		//发送CMD55,告知SD卡下一条是APP CMD
					r1 = SD_SendCmd(CMD41, 0, 0x01);//发送CMD41（ACMD指令）,设置HCS = 0,告诉SD卡主机不支持高容量卡（SD V1.x or MMC V3没有高容量卡）
				}while(r1 && retry--);				//等待退出IDLE模式	
			}
			else	//MMC卡不支持CMD55+CMD41识别
			{
				SD_Type = SD_TYPE_MMC;	//MMC V3
				retry = 0xFFFE;
				do 
				{											    
					r1 = SD_SendCmd(CMD1, 0, 0x01);	//发送CMD1（等同于CMD55+CMD41的功能）
				}while(r1 && retry--);				//等待退出IDLE模式  
			}
			
			/* SD V1.x or MMC V3需要设置块大小 */
			if((retry == 0) || (SD_SendCmd(CMD16, 512, 0X01) != 0))
				SD_Type = SD_TYPE_ERR;	//错误的卡
		}
	}
	
	SD_DisSelect();		//取消片选
	SD_SPI_SpeedHigh();	//高速模式
	if(SD_Type)			
		return 0;		//SD卡初始化成功
	else if(r1)
		return r1; 		//错误	   
	else
		return 0xAA;	//其他错误
}

/**
  * 读SD卡函数
  * buf:数据缓存区
  * sector:起始扇区地址
  * cnt:扇区数
  * 返回值:其他,失败;
  *           0,成功;  
  */
u8 SD_ReadDisk(u8* buf, u32 sector, u8 cnt)
{
	u8 r1;
	
	if(SD_Type != SD_TYPE_V2HC)
		sector <<= 9;	//转换为字节地址（乘以512）
	if(cnt == 1)		//单块读取
	{
		r1 = SD_SendCmd(CMD17, sector, 0x01);	//读一块数据的命令
		if(r1 == 0)		//指令发送成功
		{
			r1 = SD_RecvData(buf, 512);			//接收512个字节	   
		}
	}
	else	//多块读取
	{
		r1 = SD_SendCmd(CMD18, sector, 0X01);	//读取多块数据的命令
		if(r1 == 0)		//指令发送成功
		{
			do
			{
				r1 = SD_RecvData(buf, 512);		//接收512个字节	 
				buf += 512;  
			}while((--cnt) && (r1 == 0));
			
			SD_SendCmd(CMD12, 0, 0x01);			//发送停止读命令
		}
	}  
	
	SD_DisSelect();	//取消片选
	
	return r1;		
}

/**
  * 写SD卡函数
  * buf:数据缓存区
  * sector:起始扇区地址
  * cnt:扇区数
  * 返回值:0,成功;
  *     其他,失败;
  */
u8 SD_WriteDisk(u8*buf,u32 sector,u8 cnt)
{
	u8 r1;
	
	if(SD_Type != SD_TYPE_V2HC)
		sector <<= 9;	//转换为字节地址（乘以512）
	if(cnt == 1)		//单块写
	{
		r1 = SD_SendCmd(CMD24, sector, 0x01);	//单块写命令
		if(r1 == 0)		//指令发送成功
		{
			r1 = SD_SendBlock(buf, 0xFE);		//写512个字节	    0xFE为写单块命令
		}
	}
	else				//多块写		
	{
		if(SD_Type != SD_TYPE_MMC)
		{
			SD_SendCmd(CMD55, 0, 0x01);	
			SD_SendCmd(CMD23, cnt, 0X01);		//发送ACMD预擦除数据块指令	
		}
		
 		r1 = SD_SendCmd(CMD25, sector, 0X01);	//连续读命令
		if(r1==0)
		{
			do
			{
				r1 = SD_SendBlock(buf, 0xFC);	//写512个字节	 
				buf += 512;  
			}while((--cnt) && (r1 == 0));
			
			r1 = SD_SendBlock(0,0xFD);			//发送多块写数据结束令牌:0XFD,结束多块写操作
		}
	} 
	
	SD_DisSelect();	//取消片选
	
	return r1;
}

/** 
  * 获取SD卡的总扇区数（扇区数）  
  * 每扇区的字节数必为512,因为如果不是512,则初始化不能通过.
  * 返回值:0,取容量出错 
  *     其他,SD卡的扇区数（每个扇区512字节）	
  */
u32 SD_GetSectorCount(void)
{
    u8 csd[16];
    u32 Capacity;  
    u8 n;
	u16 csize; 
	
	/* 取CSD信息,如果期间出错,返回0 */
    if(SD_GetCSD(csd) != 0) 
		return 0;
	
    /* 如果为SDHC卡,按照下面方式计算 */
    if((csd[0] & 0xC0) == 0x40)	 //V2.00的卡
    {	
		csize = csd[9] + ((u16)csd[8] << 8) + 1;
		Capacity = (u32)csize << 10;		//得到扇区数	 		   
    }
	else//V1.XX的卡
    {	
		n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
		csize = (csd[8] >> 6) + ((u16)csd[7] << 2) + ((u16)(csd[6] & 3) << 10) + 1;
		Capacity = (u32)csize << (n - 9);	//得到扇区数   
    }
	
    return Capacity;	//返回扇区数（计算实际容量需要*512）
}

/**
  * 获取SD卡的CID信息,包括制造商信息
  * 输入:u8* cid_data(存放CID的内存,至少16Byte）	  
  * 返回值:0,NO_ERR;
  * 	   1,错误;
 */
u8 SD_GetCID(u8* cid_data)
{
    u8 r1;
	
    /* 发CMD10命令,读CID */
    r1 = SD_SendCmd(CMD10, 0, 0x01);
    if(r1 == 0x00)
	{
		r1 = SD_RecvData(cid_data, 16);	//接收16个字节的数据	 
    }
	
	SD_DisSelect();	//取消片选
	
	if(r1)
		return 1;	//读取失败
	else 
		return 0;	//读取成功
}	

/**
  * 获取SD卡的CSD信息,包括容量和速度信息
  * 输入:u8* cid_data(存放CID,16Byte）	    
  * 返回值:0,NO_ERR
  * 	   1,错误
  */
u8 SD_GetCSD(u8* csd_data)
{
    u8 r1;
	
    r1 = SD_SendCmd(CMD9, 0, 0x01);		//发CMD9命令,读CSD
    if(r1 == 0)
	{
    	r1 = SD_RecvData(csd_data, 16);	//接收16个字节的数据 
    }
	
	SD_DisSelect();	//取消片选
	
	if(r1)
		return 1;	//读取失败
	else 
		return 0;	//读取成功
}  
