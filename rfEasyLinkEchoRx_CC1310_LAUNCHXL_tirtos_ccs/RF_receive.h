#include "Board.h"
#if defined( TEG_PROTOCOL_30 )
#define SUBG_PACKET_MAX 32
#elif defined( TEG_PROTOCOL_20 )
#define SUBG_PACKET_MAX 22
#else
#define SUBG_PACKET_MAX 24
#endif

#define BUILD_UINT16(loByte, hiByte) \
          ((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

void echoTask_init();
void sendRFTx();

void OADTarget_eraseFlash(uint8_t page);
void OADTarget_writeFlash(uint8_t page, uint32_t offset, uint8_t *pBuf, uint16_t len);
void OADTarget_readFlash(uint8_t page, uint32_t offset, uint8_t *pBuf, uint16_t len);
