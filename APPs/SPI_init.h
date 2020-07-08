#ifndef __SPI_init_H
#define __SPI_init_H



#define SPI_FLAG_TIMEOUT 72000
#define Timeout ((u32)0x10000)
#define dummy 0x00
typedef enum
{ NSS1 = 0x0,
  NSS2 = 0x1,
  NSS3 = 0x2,
  NSS4 = 0x3,
	NSS5 = 0x4,
  NSS6 = 0x5,
  NSS7 = 0x6,
  NSS8 = 0x7
}NSS_SELECT;

#define IS_NSS_SELECT(NUM) (((NUM) == NSS1) || ((NUM) == NSS2) || \
                            ((NUM) == NSS3) || ((NUM) == NSS4) || \
                            ((NUM) == NSS5) || ((NUM) == NSS6) || \
                            ((NUM) == NSS7) || ((NUM) == NSS8))




void SPI_init(void );

ErrorStatus SPI_biduplex_bytes(u8 NSSx,u8* data_s,u8* data_r,u8 Numbertosent,u8 Numbertoread);

void SPI_sent_command(u8 NSSx,u8 data,u8 Numbertoread,u8* data_r);
u8 SPI_read_byte(u8 NSSx);



#endif

