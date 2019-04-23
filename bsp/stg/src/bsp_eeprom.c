#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_dma.h"
#include "bsp_qpc.h"
#include "bsp_eeprom.h"
#include "bsp_isr_i2c2.h"

Q_DEFINE_THIS_FILE

/*  We have 64Kb (8000 bytes) of available memory */
#define EEPROM_TOTAL_BYTES   8000U
#define EEPROM_PAGE_SIZE	   32U /* in bytes */
#define EEPROM_TOTAL_PAGES   (EEPROM_TOTAL_BYTES / EEPROM_PAGE_SIZE) /* 250 */
#define EEPROM_LAST_ADDRESS  (EEPROM_TOTAL_BYTES - 1U) /* 0x1F3F */
#define EEPROM_TOTAL_ADDRESS_BYTES 2U

#define EEPROM_PAGE_FIRST_ADDRESS(N) (N * EEPROM_PAGE_SIZE)
#define EEPROM_PAGE_LAST_ADDRESS(N)  ((N + 1U) * EEPROM_PAGE_SIZE - 1U)

/*
      Bus address for the 24LC64 chip
            (7 bit addressing)

   |----------------------------------|
   | MSB                         LSB  |
   | 7   6   5   4   5   2   1    0   |
   |----------------------------------|
   |         I2C address       |      |
   |---------------------------|      |
   | A6| A5| A4| A3| A2| A1| A0| R/W' |
   |---------------------------|      |
   |  CONTROL CODE |CHIP SELECT|      |
   |--------------------------------- |
   | 1 | 0 | 1 | 0 | 0 | 0 | 0 | 1/0  |
   |----------------------------------|

   - R/W: 1 = Read; 0 = Write
   - Possible values: 0xA0 or 0xA1
*/
#define EEPROM_24LC64_ADDRESS	(0x50U << 1U) /* gives 0xA0 */

#define EEPROM_I2C_DEVICE I2C2
//#define EEPROM_WRITE_TIME 5U /* Page write time in ms */
//#define EEPROM_TIMEOUT	  10U /* Timeout for write in ms */

/* -- DMA channels used -- */
#define EEPROM_I2C_DMA_DEVICE        DMA1
#define EEPROM_I2C_DMA_TX_CHANNEL    LL_DMA_CHANNEL_4
#define EEPROM_I2C_DMA_RX_CHANNEL    LL_DMA_CHANNEL_5

/* See "Table 31. Summary of the DMA1 requests for each channel on STM32F09x devices"
   on page 197 of RM0091.
*/
#define EEPROM_I2C_DMA_TX_PERIPH_REQ LL_DMA_REQUEST_2
#define EEPROM_I2C_DMA_RX_PERIPH_REQ LL_DMA_REQUEST_2

/* -- EEPROM Manager AO definition -- */
typedef struct {
    QActive super;
    QTimeEvt timeEvent;
} EepromManager;

typedef struct {
    QEvt super;

    uint16_t startAddress;
    uint8_t nBytes;
} ReadEvt;

typedef struct {
    QEvt super;

    uint16_t startAddress;
    uint8_t nBytes;
    uint8_t* data;
} WriteEvt;

enum EepromManagerSignals {
    EEPROM_TIMEOUT_SIG = Q_USER_SIG,
    EEPROM_I2C_INTERRUPT_SIG,
    EEPROM_READ_PAGE_REQ_SIG,
    EEPROM_WRITE_PAGE_REQ_SIG,
    EEPROM_TRANSMITING_SIG,
    EEPROM_RECEIVING_SIG
};


/* -- Local variables -- */
static uint8_t l_eepromPageReadBuffer[EEPROM_PAGE_SIZE];
static uint8_t l_eepromPageWriteBuffer[EEPROM_PAGE_SIZE];

static EepromManager l_eepromManager;
static QEvt const * l_eepromManager_queueBuffer[20];

/* -- Local function prototypes -- */
static void BSP_EEPROM_SetupDeviceForWrite(uint32_t nBytes);
static void BSP_EEPROM_SetupDeviceForRead_Step1(void);
static void BSP_EEPROM_SetupDeviceForRead_Step2(uint32_t nBytes);
static void BSP_EEPROM_DisableInterruptFlags(void);

static void BSP_EEPROM_SetupDmaTransmission(const uint32_t nBytes);
static void BSP_EEPROM_SetupDmaReception(const uint32_t nBytes);

/* -- Local function prototypes (Active object) -- */
static void BSP_EEPROM_ao_ctor(EepromManager* me);

static QState BSP_EEPROM_ao_init(EepromManager* me, QEvt const * const e);
static QState BSP_EEPROM_ao_ready(EepromManager* me, QEvt const * const e);

static QState BSP_EEPROM_ao_read(EepromManager* me, QEvt const * const e);
static QState BSP_EEPROM_ao_receiving(EepromManager* me, QEvt const * const e);

static QState BSP_EEPROM_ao_write(EepromManager* me, QEvt const * const e);
static QState BSP_EEPROM_ao_transmitting(EepromManager* me, QEvt const * const e);

/* -- ISR implementation -- */
void I2C2_IRQHandler(void)
{
  QK_ISR_ENTRY();
  QACTIVE_POST((QActive*)&l_eepromManager, 0U, (void *)0);
  QK_ISR_EXIT();
}

/* -- Implementation of public functions -- */
void BSP_EEPROM_ao_initAO(void)
{
  BSP_EEPROM_ao_ctor(&l_eepromManager);
}

void BSP_EEPROM_startAO(const uint8_t priority)
{
  QACTIVE_START((QActive*)&l_eepromManager.super,
    priority,
    l_eepromManager_queueBuffer,
    Q_DIM(l_eepromManager_queueBuffer),
    (void*)0, 0U,
    (QEvt*)0);
}

void BSP_EEPROM_read(uint16_t startAddress, uint8_t nBytes)
{

}

void BSP_EEPROM_write(uint16_t startAddress, uint8_t nBytes, uint8_t* data)
{

}

/* -- Implementation of local functions -- */
static void BSP_EEPROM_SetupDmaTransmission(const uint32_t nBytes)
{
  const uint32_t srcAddress = (uint32_t)&l_eepromPageWriteBuffer;
  const uint32_t dstAddress = LL_I2C_DMA_GetRegAddr(EEPROM_I2C_DEVICE, LL_I2C_DMA_REG_DATA_TRANSMIT);

  LL_DMA_InitTypeDef init;
  init.PeriphOrM2MSrcAddress = srcAddress;
  init.MemoryOrM2MDstAddress = dstAddress;
  init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  init.Mode = LL_DMA_MODE_NORMAL;
  init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  init.NbData = nBytes;
  init.PeriphRequest = EEPROM_I2C_DMA_TX_PERIPH_REQ; /* I2C2_TX */
  init.Priority = LL_DMA_PRIORITY_VERYHIGH;

  if(LL_DMA_Init(EEPROM_I2C_DMA_DEVICE, EEPROM_I2C_DMA_TX_CHANNEL, &init) == SUCCESS) {
    LL_DMA_EnableIT_TC(EEPROM_I2C_DMA_DEVICE, EEPROM_I2C_DMA_TX_CHANNEL); /* transfer complete */
    LL_DMA_EnableIT_TE(EEPROM_I2C_DMA_DEVICE, EEPROM_I2C_DMA_TX_CHANNEL); /* transfer error */
    LL_DMA_EnableChannel(EEPROM_I2C_DMA_DEVICE, EEPROM_I2C_DMA_TX_CHANNEL);
  } else {
    /* handle failure here */
  }
}

static void BSP_EEPROM_SetupDmaReception(const uint32_t nBytes)
{
  const uint32_t srcAddress = LL_I2C_DMA_GetRegAddr(EEPROM_I2C_DEVICE, LL_I2C_DMA_REG_DATA_RECEIVE);
  const uint32_t dstAddress = (uint32_t)&l_eepromPageReadBuffer;

  LL_DMA_InitTypeDef init;
  init.PeriphOrM2MSrcAddress = srcAddress;
  init.MemoryOrM2MDstAddress = dstAddress;
  init.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  init.Mode = LL_DMA_MODE_NORMAL;
  init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  init.NbData = nBytes;
  init.PeriphRequest = EEPROM_I2C_DMA_RX_PERIPH_REQ; /* I2C2_RX */
  init.Priority = LL_DMA_PRIORITY_VERYHIGH;

  if(LL_DMA_Init(EEPROM_I2C_DMA_DEVICE, EEPROM_I2C_DMA_RX_CHANNEL, &init) == SUCCESS) {
    LL_DMA_EnableIT_TC(EEPROM_I2C_DMA_DEVICE, EEPROM_I2C_DMA_RX_CHANNEL); /* transfer complete */
    LL_DMA_EnableIT_TE(EEPROM_I2C_DMA_DEVICE, EEPROM_I2C_DMA_RX_CHANNEL); /* transfer error */
    LL_DMA_EnableChannel(EEPROM_I2C_DMA_DEVICE, EEPROM_I2C_DMA_RX_CHANNEL);
  } else {
    /* handle failure here */
  }

  // LL_I2C_EnableDMAReq_RX(EEPROM_I2C_DEVICE);
}

static void BSP_EEPROM_SetupDeviceForWrite(uint32_t nBytes)
{
  if(nBytes > 32U) {
    nBytes = 32U;
  }
  /* Setup the I2C device to communicate with the 24LC64 chip */
  LL_I2C_SetMasterAddressingMode(EEPROM_I2C_DEVICE, LL_I2C_ADDRSLAVE_7BIT);
  /* Setup slave address of the the 24LC64 chip */
  LL_I2C_SetSlaveAddr(EEPROM_I2C_DEVICE, EEPROM_24LC64_ADDRESS);
  /* Configure the transfer direction */
  LL_I2C_SetTransferRequest(EEPROM_I2C_DEVICE, LL_I2C_REQUEST_WRITE);
  /* Configure the number of bytes for transfer */
  LL_I2C_SetTransferSize(EEPROM_I2C_DEVICE, nBytes + EEPROM_TOTAL_ADDRESS_BYTES);

  /*  Enable automatic STOP condition generation */
  LL_I2C_EnableAutoEndMode(EEPROM_I2C_DEVICE);

  /* Master transmitter
     In the case of a write transfer, the TXIS flag is set after each byte transmission,
     after the 9th SCL pulse when an ACK is received. A TXIS event generates an interrupt if
     the TXIE bit is set in the I2C_CR1 register. The flag is cleared when the I2C_TXDR register
     is written with the next data byte to be transmitted (i.e., call to LL_I2C_TransmitData8/2).
  */
  LL_I2C_EnableIT_TX(EEPROM_I2C_DEVICE); /* Enables TXIS interrupt */

}

static void BSP_EEPROM_SetupDeviceForRead_Step1(void)
{
  /* Setup the I2C device to communicate with the 24LC64 chip */
  LL_I2C_SetMasterAddressingMode(EEPROM_I2C_DEVICE, LL_I2C_ADDRSLAVE_7BIT);
  /* Setup slave address of the the 24LC64 chip */
  LL_I2C_SetSlaveAddr(EEPROM_I2C_DEVICE, EEPROM_24LC64_ADDRESS);
  /* Configure the transfer direction */
  LL_I2C_SetTransferRequest(EEPROM_I2C_DEVICE, LL_I2C_REQUEST_WRITE);
  /* Configure the number of bytes for transfer */
  LL_I2C_SetTransferSize(EEPROM_I2C_DEVICE, EEPROM_TOTAL_ADDRESS_BYTES);

  /*  Disable automatic STOP condition generation (software end mode).
      This enables RESTART. */
  LL_I2C_DisableAutoEndMode(EEPROM_I2C_DEVICE);
  /* Enable Transfer Complete Interrupt */
  LL_I2C_EnableIT_TC(EEPROM_I2C_DEVICE);

  /* Master transmitter
     In the case of a write transfer, the TXIS flag is set after each byte transmission,
     after the 9th SCL pulse when an ACK is received. A TXIS event generates an interrupt if
     the TXIE bit is set in the I2C_CR1 register. The flag is cleared when the I2C_TXDR register
     is written with the next data byte to be transmitted (i.e., call to LL_I2C_TransmitData8/2).
  */
  LL_I2C_EnableIT_RX(EEPROM_I2C_DEVICE); /* Sets TXIE bit in the I2C_CR1 register */
}

static void BSP_EEPROM_DisableInterruptFlags(void)
{
  /*  Resets the following control bits: RXIE, TXIE, STOPIE, TCIE, ADDRIE, NACKIE, ERRIE.
      See "Table 100. I2C Interrupt requests" on page 669 of RM0091.
  */
  LL_I2C_DisableIT_RX(EEPROM_I2C_DEVICE);   /* Disable RXNE interrupt */
  LL_I2C_DisableIT_TX(EEPROM_I2C_DEVICE);   /* Disable TXIS interrupt */
  LL_I2C_DisableIT_STOP(EEPROM_I2C_DEVICE); /* Disable STOP detection interrupt */
  LL_I2C_DisableIT_TC(EEPROM_I2C_DEVICE);   /* Disable Transfer Complete interrupt */
  LL_I2C_DisableIT_ADDR(EEPROM_I2C_DEVICE); /* Disable Address match interrupt (slave mode only) */
  LL_I2C_DisableIT_NACK(EEPROM_I2C_DEVICE); /* Disable Not acknowledge received interrupt */
  LL_I2C_DisableIT_ERR(EEPROM_I2C_DEVICE);  /* Disable Error interrupts */
}

static void BSP_EEPROM_SetupDeviceForRead_Step2(uint32_t nBytes)
{
  if(nBytes > 32U) {
    nBytes = 32U;
  }
  /* Setup the I2C device to communicate with the 24LC64 chip */
  LL_I2C_SetMasterAddressingMode(EEPROM_I2C_DEVICE, LL_I2C_ADDRSLAVE_7BIT);
  /* Setup slave address of the the 24LC64 chip */
  LL_I2C_SetSlaveAddr(EEPROM_I2C_DEVICE, EEPROM_24LC64_ADDRESS);
  /* Configure the transfer direction */
  LL_I2C_SetTransferRequest(EEPROM_I2C_DEVICE, LL_I2C_REQUEST_READ);
  /* Configure the number of bytes for transfer */
  LL_I2C_SetTransferSize(EEPROM_I2C_DEVICE, nBytes);

  /*  Enable automatic STOP condition generation */
  LL_I2C_EnableAutoEndMode(EEPROM_I2C_DEVICE);


  LL_I2C_EnableIT_RX(EEPROM_I2C_DEVICE); /* Enables RXNE interrupt */
}

/* The master automatically sends the START condition followed by the slave
  address as soon as it detects that the bus is free (BUSY = 0). */

/* -- Active Object functions -- */
static void BSP_EEPROM_ao_ctor(EepromManager* me)
{
  QActive_ctor(&me->super, Q_STATE_CAST(&BSP_EEPROM_ao_init));
  QTimeEvt_ctorX(&me->timeEvent, &me->super, EEPROM_TIMEOUT_SIG, 0U);
}

static QState BSP_EEPROM_ao_init(EepromManager* me, QEvt const * const e)
{
  /* Important: you must've called BSP_MX_I2C2_Init/0 before, see BSP_init/0. */

  /* In order to enable the I2C interrupts, the following sequence is required:
      1. Configure and enable the I2C IRQ channel in the NVIC.
      2. Configure the I2C to generate interrupts (depends on current state).
  */
  BSP_EEPROM_DisableInterruptFlags();
  NVIC_EnableIRQ(I2C2_IRQn);

  /* Enable the device */
  LL_I2C_Enable(EEPROM_I2C_DEVICE);

  return Q_TRAN(&BSP_EEPROM_ao_ready);
}

static QState BSP_EEPROM_ao_ready(EepromManager* me, QEvt const * const e)
{
  QState status;
  switch(e->sig) {
    case Q_INIT_SIG:
      status = Q_HANDLED();
      break;
    case Q_ENTRY_SIG:
      status = Q_HANDLED();
      break;
    case Q_EXIT_SIG:
      status = Q_HANDLED();
      break;
    case EEPROM_READ_PAGE_REQ_SIG:
      status = Q_TRAN(&BSP_EEPROM_ao_read);
      break;
    case EEPROM_WRITE_PAGE_REQ_SIG:
      status = Q_TRAN(&BSP_EEPROM_ao_write);
      break;
    default:
      status = Q_SUPER(&QHsm_top);
      break;
  }
  return status;
}

static QState BSP_EEPROM_ao_read(EepromManager* me, QEvt const * const e)
{
  QState status;
  switch(e->sig) {
    case Q_INIT_SIG:
      status = Q_HANDLED();
      break;
    case Q_ENTRY_SIG:
      status = Q_HANDLED();
      break;
    case Q_EXIT_SIG:
      status = Q_HANDLED();
      break;
    default:
      status = Q_SUPER(&QHsm_top);
      break;
  }
  return status;
}

static QState BSP_EEPROM_ao_write(EepromManager* me, QEvt const * const e)
{
  QState status;
  switch(e->sig) {
    case Q_INIT_SIG:
      status = Q_HANDLED();
      break;
    case Q_ENTRY_SIG:
      status = Q_HANDLED();
      break;
    case Q_EXIT_SIG:
      status = Q_HANDLED();
      break;
    default:
      status = Q_SUPER(&QHsm_top);
      break;
  }
  return status;
}
