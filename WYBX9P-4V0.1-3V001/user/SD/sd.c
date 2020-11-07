#include "sd.h"			   

					   
u8 SD_Type = 0;	//SD�������� 

/************************SPI�����ֲ�޸�************************/
/**
  * SD��SPI��д����
  * data:Ҫд�������
  * ����ֵ:����������
  */
u8 SD_SPI_ReadWriteByte(u8 data)
{

	return SPI3_ReadWriteByte(data);
}

/**
  * ����SD����SPIΪ����ģʽ 
  * SD����ʼ����ʱ��,��Ҫ����
  * �ڿ���ʼ����ʱ��,CLKʱ������ܳ���400KHz������
  */
void SD_SPI_SpeedLow(void)
{
 	SPI3_SetSpeed(SPI_BaudRatePrescaler_256);	//���õ�����ģʽ(281.25KHz)	
}

/**
  * ����SD����SPIΪ����ģʽ
  * SD������������ʱ��,���Ը����ˣ�18MHz��
  */
void SD_SPI_SpeedHigh(void)
{
 	SPI3_SetSpeed(SPI_BaudRatePrescaler_4);		//���õ�����ģʽ(18MHz)	
}



/**
  * ȡ��Ƭѡ����
  * ȡ��Ƭѡ��,�෢8��CLK��Ϊ��ʹSD�����һЩ�ڲ�����
  */
void SD_DisSelect(void)
{
	SD_CS_DISABLE();
 	SD_SPI_ReadWriteByte(0xFF);	//�ṩ�����8��ʱ��
}

/**
  * �ȴ�SD��׼���ú���
  * ����ֵ:0,׼������;
  *     ����,�������;
  */
u8 SD_WaitReady(void)
{
	u32 t = 0;
	
	do
	{
		/* SD����д�����ݿ��Ժ�,SD���ڲ�������MISO��,ֱ���ڲ���̽���֮��Ż��ͷ�MISO���� */
		if(SD_SPI_ReadWriteByte(0xFF) == 0xFF)	
			return 0;		//OK
		t++;		  	
	}while(t < 0xFFFFFF);	//�ȴ� 
	
	return 1;	//ERROR
}

/**
  * ʹ��SD��Ƭѡ����
  * ���ҵȴ�SD��׼��OK
  * ����ֵ:0,�ɹ�;
  *        1,ʧ��;
  */
u8 SD_Select(void)
{
	SD_CS_ENABLE();
	
	if(SD_WaitReady() == 0)
		return 0;	//�ȴ��ɹ�
	
	SD_DisSelect();
	return 1;		//�ȴ�ʧ��
}

/**
  * �ȴ�SD����Ӧ
  * Response:��Ҫ�õ��Ļ�Ӧֵ
  * ����ֵ:0,�ɹ��õ��˸û�Ӧֵ
  *     ����,�õ���Ӧֵʧ��
  */
u8 SD_GetResponse(u8 Response)
{
	u16 Count = 0xFFFF;	//�ȴ�����	
	
	while((SD_SPI_ReadWriteByte(0xFF) != Response) && Count)	//�ȴ��õ���Ӧ�Ļ�Ӧ 
		Count--; 	  
	if(Count == 0)
		return MSD_RESPONSE_FAILURE;	//�õ���Ӧʧ��   
	else 
		return MSD_RESPONSE_NO_ERROR;	//�õ�����Ҫ�Ļ�Ӧ
}

/** 
  * ��sd����ȡһ�����ݰ�������
  * buf:���ݻ�����
  * len:Ҫ��ȡ�����ݳ���.
  * ����ֵ:0,�ɹ�;
  *     ����,ʧ��;	
  */
u8 SD_RecvData(u8*buf, u16 len)
{			  	  
	if(SD_GetResponse(0xFE))	//�ȴ�SD������������ʼ����0xFE
		return 1;				//û�ȵ�������ʼ����0xFE
	
	/* �ȵ���������ʼ����0xFE */
    while(len--)	//��ʼ��������
    {
        *buf = SPI3_ReadWriteByte(0xFF);
        buf++;
    }
	
    /* ��ȡ2���ֽڵ�CRC */
    SD_SPI_ReadWriteByte(0xFF);
    SD_SPI_ReadWriteByte(0xFF);	
	
    return 0;//��ȡ�ɹ�
}

/**
  * ��sd��д��һ�����ݰ���512�ֽڣ������� 
  * buf:���ݻ�����
  * cmd:��������
  * ����ֵ:0,�ɹ�;
  *     ����,ʧ��;	
  */
u8 SD_SendBlock(u8* buf, u8 cmd)
{	
	u16 t;	
	
	if(SD_WaitReady())
		return 1;	//SD��δ׼����
	
	/* SD���Ѿ�׼���� */
	SD_SPI_ReadWriteByte(cmd);	//��������    
	if(cmd != 0XFD)	//���ǽ���ָ��
	{
		for(t = 0; t < 512; t++)
			SPI3_ReadWriteByte(buf[t]);	//����ٶ�,���ٺ�������ʱ��
		
	    SD_SPI_ReadWriteByte(0xFF);		//���������ֽڵ�αcrc
	    SD_SPI_ReadWriteByte(0xFF);
		
		t = SD_SPI_ReadWriteByte(0xFF);	//������Ӧ
		if((t & 0x1F) != 0x05)
			return 2;	//��Ӧ����									  					    
	}
	
    return 0;//д��ɹ�
}

/**
  * ��SD������һ�������
  * ����: u8 cmd   ���������������λ�̶�Ϊ01�� 
  *      u32 arg   �����������Щ����û�в�����
  *       u8 crc   crcУ��ֵ�����λ�̶�Ϊ1��	   
  * ����ֵ:0xFF,ƬѡʧЧ
  *        ����,SD�����ص�Ӧ������r1	
  */
u8 SD_SendCmd(u8 cmd, u32 arg, u8 crc)
{
    u8 r1,cmd1 = 0;	
	u8 retry = 0; 
	
	SD_DisSelect();		//ȡ���ϴ�Ƭѡ
	if(SD_Select())
		return 0xFF;	//ƬѡʧЧ 

	/* Ƭѡ�ɹ� */
	cmd1 = cmd;
	cmd1 &= 0x7F; 		//���λ�̶�Ϊ0
	cmd1 |= 0x40;		//�θ�λ�̶�Ϊ1
    SD_SPI_ReadWriteByte(cmd1);			//д����������
	
    SD_SPI_ReadWriteByte(arg >> 24);	//д���������
    SD_SPI_ReadWriteByte(arg >> 16);
    SD_SPI_ReadWriteByte(arg >> 8);
    SD_SPI_ReadWriteByte(arg);	  
	
    SD_SPI_ReadWriteByte(crc | 0x01);	//д��CRC 
	
	if(cmd == CMD12)
		SD_SPI_ReadWriteByte(0xFF);		//��ֹͣ��ȡʱ,����һ���ֽ�
	
    /* �ȴ���Ӧ,��ʱ�˳� */
	retry = 0x1F;
	do
	{
		r1 = SD_SPI_ReadWriteByte(0xFF);
	}while((r1 & 0x80) && (retry--));	 
	
	//����Ӧ������r1
    return r1;
}	

/**
  * SD����ʼ������
  * ����ֵ:0,�ɹ�;
  *     ����,ʧ��;
  */
u8 SD_Initialize(void)
{
    u8 r1;      //���SD���ķ���ֵ
    u16 retry;  //�������г�ʱ����
    u8 buf[4];  
	u16 i;
	
 	for(i = 0; i < 10; i++)			//���﷢����80��CLK
		SD_SPI_ReadWriteByte(0xFF);	//��������74��CLK�����ϵ����,������SD������������ѹ��Ҫ64��CLK,����10��CLK��Ϊ����SDͬ����
	
	retry = 20;
	do
	{
		r1 = SD_SendCmd(CMD0, 0, 0x95);	//����IDLE״̬
	}while((r1 != 0x01) && (retry--));
	
 	SD_Type = 0;		//Ĭ���޿�
	if(r1 == 0x01)		//����IDLE״̬
	{
		/* CMD8ָ�����ڷ���SD���ӿ�������Ϣ,���������ѹ��;����ֻ��V2.0���Ժ��SD���Ż���Ӧ��ָ��,MMC����V1.x�Ŀ�,�ǲ�֧�ָ������;��Ҳ������������SD���汾 */
		if(SD_SendCmd(CMD8, 0x1AA, 0x87) == 1)		//SD V2.0 ����Ҫ���ÿ��С���̶�Ϊ512�ֽڣ�
		{
			for(i = 0; i < 4; i++)
				buf[i] = SD_SPI_ReadWriteByte(0xFF);	//Get trailing return value of R7 resp
			if((buf[2] == 0x01) && (buf[3] == 0XAA))	//���Ƿ�֧��2.7~3.6V(���CMD8д���ָ������Ͷ�����Ӧ���Ƿ�һ��)
			{
				retry = 0xFFFE;
				do
				{
					SD_SendCmd(CMD55, 0, 0X01);					//����CMD55,��֪SD����һ����APP CMD
					r1 = SD_SendCmd(CMD41, 0x40000000, 0x01);	//����CMD41��ACMDָ�,����HCS = 1,����SD����������֧�ָ�������
				}while(r1 && retry--);		//�ȴ�SD���˳�����״̬
				
				
				if(retry == 0)
					SD_Type = SD_TYPE_ERR;	//����Ŀ�
				else if(SD_SendCmd(CMD58, 0, 0x01) == 0)	//����SD2.0���汾��ʼ
				{
					for(i = 0; i < 4; i++)
						buf[i] = SD_SPI_ReadWriteByte(0xFF);	//�õ�OCRֵ
					if(buf[0] & 0x40)							//���CCS
						SD_Type = SD_TYPE_V2HC; 				//CCS = 1   	
					else 
						SD_Type = SD_TYPE_V2;					//CCS = 0   
				}
			}
		}
		else	//SD V1.x or MMC V3
		{
			SD_SendCmd(CMD55, 0, 0x01);				//����CMD55,��֪SD����һ����APP CMD
			r1 = SD_SendCmd(CMD41, 0, 0x01);		//����CMD41��ACMDָ�,����HCS = 0,����SD��������֧�ָ���������SD V1.x or MMC V3û�и���������
			if(r1 <= 1)
			{		
				SD_Type = SD_TYPE_V1;
				retry = 0xFFFE;
				do 
				{
					SD_SendCmd(CMD55, 0, 0x01);		//����CMD55,��֪SD����һ����APP CMD
					r1 = SD_SendCmd(CMD41, 0, 0x01);//����CMD41��ACMDָ�,����HCS = 0,����SD��������֧�ָ���������SD V1.x or MMC V3û�и���������
				}while(r1 && retry--);				//�ȴ��˳�IDLEģʽ	
			}
			else	//MMC����֧��CMD55+CMD41ʶ��
			{
				SD_Type = SD_TYPE_MMC;	//MMC V3
				retry = 0xFFFE;
				do 
				{											    
					r1 = SD_SendCmd(CMD1, 0, 0x01);	//����CMD1����ͬ��CMD55+CMD41�Ĺ��ܣ�
				}while(r1 && retry--);				//�ȴ��˳�IDLEģʽ  
			}
			
			/* SD V1.x or MMC V3��Ҫ���ÿ��С */
			if((retry == 0) || (SD_SendCmd(CMD16, 512, 0X01) != 0))
				SD_Type = SD_TYPE_ERR;	//����Ŀ�
		}
	}
	
	SD_DisSelect();		//ȡ��Ƭѡ
	SD_SPI_SpeedHigh();	//����ģʽ
	if(SD_Type)			
		return 0;		//SD����ʼ���ɹ�
	else if(r1)
		return r1; 		//����	   
	else
		return 0xAA;	//��������
}

/**
  * ��SD������
  * buf:���ݻ�����
  * sector:��ʼ������ַ
  * cnt:������
  * ����ֵ:����,ʧ��;
  *           0,�ɹ�;  
  */
u8 SD_ReadDisk(u8* buf, u32 sector, u8 cnt)
{
	u8 r1;
	
	if(SD_Type != SD_TYPE_V2HC)
		sector <<= 9;	//ת��Ϊ�ֽڵ�ַ������512��
	if(cnt == 1)		//�����ȡ
	{
		r1 = SD_SendCmd(CMD17, sector, 0x01);	//��һ�����ݵ�����
		if(r1 == 0)		//ָ��ͳɹ�
		{
			r1 = SD_RecvData(buf, 512);			//����512���ֽ�	   
		}
	}
	else	//����ȡ
	{
		r1 = SD_SendCmd(CMD18, sector, 0X01);	//��ȡ������ݵ�����
		if(r1 == 0)		//ָ��ͳɹ�
		{
			do
			{
				r1 = SD_RecvData(buf, 512);		//����512���ֽ�	 
				buf += 512;  
			}while((--cnt) && (r1 == 0));
			
			SD_SendCmd(CMD12, 0, 0x01);			//����ֹͣ������
		}
	}  
	
	SD_DisSelect();	//ȡ��Ƭѡ
	
	return r1;		
}

/**
  * дSD������
  * buf:���ݻ�����
  * sector:��ʼ������ַ
  * cnt:������
  * ����ֵ:0,�ɹ�;
  *     ����,ʧ��;
  */
u8 SD_WriteDisk(u8*buf,u32 sector,u8 cnt)
{
	u8 r1;
	
	if(SD_Type != SD_TYPE_V2HC)
		sector <<= 9;	//ת��Ϊ�ֽڵ�ַ������512��
	if(cnt == 1)		//����д
	{
		r1 = SD_SendCmd(CMD24, sector, 0x01);	//����д����
		if(r1 == 0)		//ָ��ͳɹ�
		{
			r1 = SD_SendBlock(buf, 0xFE);		//д512���ֽ�	    0xFEΪд��������
		}
	}
	else				//���д		
	{
		if(SD_Type != SD_TYPE_MMC)
		{
			SD_SendCmd(CMD55, 0, 0x01);	
			SD_SendCmd(CMD23, cnt, 0X01);		//����ACMDԤ�������ݿ�ָ��	
		}
		
 		r1 = SD_SendCmd(CMD25, sector, 0X01);	//����������
		if(r1==0)
		{
			do
			{
				r1 = SD_SendBlock(buf, 0xFC);	//д512���ֽ�	 
				buf += 512;  
			}while((--cnt) && (r1 == 0));
			
			r1 = SD_SendBlock(0,0xFD);			//���Ͷ��д���ݽ�������:0XFD,�������д����
		}
	} 
	
	SD_DisSelect();	//ȡ��Ƭѡ
	
	return r1;
}

/** 
  * ��ȡSD����������������������  
  * ÿ�������ֽ�����Ϊ512,��Ϊ�������512,���ʼ������ͨ��.
  * ����ֵ:0,ȡ�������� 
  *     ����,SD������������ÿ������512�ֽڣ�	
  */
u32 SD_GetSectorCount(void)
{
    u8 csd[16];
    u32 Capacity;  
    u8 n;
	u16 csize; 
	
	/* ȡCSD��Ϣ,����ڼ����,����0 */
    if(SD_GetCSD(csd) != 0) 
		return 0;
	
    /* ���ΪSDHC��,�������淽ʽ���� */
    if((csd[0] & 0xC0) == 0x40)	 //V2.00�Ŀ�
    {	
		csize = csd[9] + ((u16)csd[8] << 8) + 1;
		Capacity = (u32)csize << 10;		//�õ�������	 		   
    }
	else//V1.XX�Ŀ�
    {	
		n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
		csize = (csd[8] >> 6) + ((u16)csd[7] << 2) + ((u16)(csd[6] & 3) << 10) + 1;
		Capacity = (u32)csize << (n - 9);	//�õ�������   
    }
	
    return Capacity;	//����������������ʵ��������Ҫ*512��
}

/**
  * ��ȡSD����CID��Ϣ,������������Ϣ
  * ����:u8* cid_data(���CID���ڴ�,����16Byte��	  
  * ����ֵ:0,NO_ERR;
  * 	   1,����;
 */
u8 SD_GetCID(u8* cid_data)
{
    u8 r1;
	
    /* ��CMD10����,��CID */
    r1 = SD_SendCmd(CMD10, 0, 0x01);
    if(r1 == 0x00)
	{
		r1 = SD_RecvData(cid_data, 16);	//����16���ֽڵ�����	 
    }
	
	SD_DisSelect();	//ȡ��Ƭѡ
	
	if(r1)
		return 1;	//��ȡʧ��
	else 
		return 0;	//��ȡ�ɹ�
}	

/**
  * ��ȡSD����CSD��Ϣ,�����������ٶ���Ϣ
  * ����:u8* cid_data(���CID,16Byte��	    
  * ����ֵ:0,NO_ERR
  * 	   1,����
  */
u8 SD_GetCSD(u8* csd_data)
{
    u8 r1;
	
    r1 = SD_SendCmd(CMD9, 0, 0x01);		//��CMD9����,��CSD
    if(r1 == 0)
	{
    	r1 = SD_RecvData(csd_data, 16);	//����16���ֽڵ����� 
    }
	
	SD_DisSelect();	//ȡ��Ƭѡ
	
	if(r1)
		return 1;	//��ȡʧ��
	else 
		return 0;	//��ȡ�ɹ�
}  
