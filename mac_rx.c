/**************************************************************************************************
  Filename:       mac_rx.c
  Revised:        $Date: 2007-10-08 14:05:36 -0700 (Mon, 08 Oct 2007) $
  Revision:       $Revision: 15624 $

  Description:    Describe the purpose and contents of the file.


  Copyright 2006-2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

/* hal */
#include "hal_defs.h"
#include "hal_types.h"

/* high-level */
#include "mac_high_level.h"
#include "mac_spec.h"

/* exported low-level */
#include "mac_low_level.h"

/* low-level specific */
#include "mac_rx.h"
#include "mac_tx.h"
#include "mac_rx_onoff.h"
#include "mac_radio.h"

/* target specific */
#include "mac_radio_defs.h"
#include "mac_autopend.h"

/* debug */
#include "mac_assert.h"

//add by gjh
#include "hal_uart.h"

/* ------------------------------------------------------------------------------------------------
 *                                            Defines
 * ------------------------------------------------------------------------------------------------
 */
#define MAX_PAYLOAD_BYTES_READ_PER_INTERRUPT   16   /* adjustable to tune performance */

/* receive FIFO bytes needed to start a valid receive (see function rxStartIsr for details) */
#define RX_THRESHOLD_START_LEN    (MAC_PHY_PHR_LEN        +  \
                                   MAC_FCF_FIELD_LEN      +  \
                                   MAC_SEQ_NUM_FIELD_LEN  +  \
                                   MAC_FCS_FIELD_LEN)

/* maximum size of addressing fields (note: command frame identifier processed as part of address) */
#define MAX_ADDR_FIELDS_LEN  ((MAC_EXT_ADDR_FIELD_LEN + MAC_PAN_ID_FIELD_LEN) * 2)

/* addressing mode reserved value */
#define ADDR_MODE_RESERVERED  1

/* length of command frame identifier */
#define CMD_FRAME_ID_LEN      1

/* packet size mask is equal to the maximum value */
#define PHY_PACKET_SIZE_MASK  0x7F      //最多127个字节

/* value for promiscuous off, must not conflict with other mode variants from separate include files */
#define PROMISCUOUS_MODE_OFF  0x00

/* bit of proprietary FCS format that indicates if the CRC is OK */
#define PROPRIETARY_FCS_CRC_OK_BIT  0x80     //最高位为1

/* dummy length value for unused entry in lookup table */
#define DUMMY_LEN   0xBE

/* value for rxThresholdIntState */
#define RX_THRESHOLD_INT_STATE_INACTIVE   0
#define RX_THRESHOLD_INT_STATE_ACTIVE     1
#define RX_THRESHOLD_INT_STATE_RESET      2


/* ------------------------------------------------------------------------------------------------
 *                                             Macros
 * ------------------------------------------------------------------------------------------------
 */
#define MEM_ALLOC(x)   macDataRxMemAlloc(x)
#define MEM_FREE(x)    macDataRxMemFree((uint8 **)x)

/*
 *  Macro for encoding frame control information into internal flags format.
 *  Parameter is pointer to the frame.  NOTE!  If either the internal frame
 *  format *or* the specification changes, this macro will need to be modified.
 */
#define INTERNAL_FCF_FLAGS(p)  ((((p)[1] >> 4) & 0x03) | ((p)[0] & 0x78))

/*
 *  The radio replaces the actual FCS with different information.  This proprietary FCS is
 *  the same length as the original and includes:
 *    1) the RSSI value
 *    2) the average correlation value (used for LQI)
 *    3) a CRC passed bit
 *
 *  These macros decode the proprietary FCS.  The macro parameter is a pointer to the two byte FCS.
 */
#define PROPRIETARY_FCS_RSSI(p)                 ((int8)((p)[0]))
#define PROPRIETARY_FCS_CRC_OK(p)               ((p)[1] & PROPRIETARY_FCS_CRC_OK_BIT)    //取最高位
#define PROPRIETARY_FCS_CORRELATION_VALUE(p)    ((p)[1] & ~PROPRIETARY_FCS_CRC_OK_BIT)   //取低7位


/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */
uint8 macRxActive;
uint8 macRxFilter;
uint8 macRxOutgoingAckFlag;     //此变量用于存储帧控制域中的ACK请求位


/* ------------------------------------------------------------------------------------------------
 *                                       Local Constants
 * ------------------------------------------------------------------------------------------------
 */
static const uint8 CODE macRxAddrLen[] =             //用于匹配究竟MAC帧用的是何种地址模式，源、目的地址模式总共有4种
{
  0,                                                /* no address */    //不存在PAN标识符子域和地址子域
  DUMMY_LEN,                                        /* reserved */      //保留的
  MAC_PAN_ID_FIELD_LEN + MAC_SHORT_ADDR_FIELD_LEN,  /* short address + pan id */       //16位短地址
  MAC_PAN_ID_FIELD_LEN + MAC_EXT_ADDR_FIELD_LEN     /* extended address + pan id */    //64位扩展地址
};


/* ------------------------------------------------------------------------------------------------
 *                                       Local Prototypes
 * ------------------------------------------------------------------------------------------------
 */
static void rxHaltCleanupFinalStep(void);

static void rxStartIsr(void);      //刚收到packet的中断服务程序，    第1个执行
static void rxAddrIsr(void);       //处理地址的中断服务程序，        第2个执行
static void rxPayloadIsr(void);    //payload数据接收的中断服务程序， 第3个执行
static void rxDiscardIsr(void);    //处理丢弃packet的中断服务程序，  
static void rxFcsIsr(void);        //处理FCS的中断服务程序，

static void rxPrepPayload(void);
static void rxDiscardFrame(void);
static void rxDone(void);           //packet接收完的中断服务程序， 第 个执行
static void rxPostRxUpdates(void);  //packet接收完后需更新的中断服务程序， 最后执行


/* ------------------------------------------------------------------------------------------------
 *                                         Local Variables
 * ------------------------------------------------------------------------------------------------
 */
static void    (* pFuncRxState)(void);
static macRx_t  * pRxBuf;      //存储数据的struct

static uint8  rxBuf[MAC_PHY_PHR_LEN + MAC_FCF_FIELD_LEN + MAC_SEQ_NUM_FIELD_LEN];   // 物理帧头 1字节 + 帧控制域 2字节 + 序列号 1字节 = 4字节
static uint8  rxUnreadLen;
static uint8  rxNextLen;
static uint8  rxPayloadLen;
static uint8  rxFilter;
static uint8  rxPromiscuousMode;
static uint8  rxIsrActiveFlag;
static uint8  rxResetFlag;
static uint8  rxFifoOverflowCount;


/**************************************************************************************************
 * @fn          macRxInit
 *
 * @brief       Initialize receive variable states.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macRxInit(void)
{
  macRxFilter          = RX_FILTER_OFF;
  rxPromiscuousMode    = PROMISCUOUS_MODE_OFF;   //原先是关闭混杂模式：PROMISCUOUS_MODE_OFF，被我改为：MAC_PROMISCUOUS_MODE_WITH_BAD_CRC
  pRxBuf               = NULL; /* required for macRxReset() to function correctly */
  macRxActive          = MAC_RX_ACTIVE_NO_ACTIVITY;
  pFuncRxState         = &rxStartIsr;         //函数指针pFuncRxState指向rxStartIsr
  macRxOutgoingAckFlag = 0;
  rxIsrActiveFlag      = 0;
  rxResetFlag          = 0;
  rxFifoOverflowCount  = 0;
}


/**************************************************************************************************
 * @fn          macRxRadioPowerUpInit      //上电后RX的初始化
 *
 * @brief       Initialization for after radio first powers up.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macRxRadioPowerUpInit(void)
{
  /* set threshold at initial value */
  MAC_RADIO_SET_RX_THRESHOLD(RX_THRESHOLD_START_LEN);     //RX_THRESHOLD_START_LEN = MAC_PHY_PHR_LEN + MAC_FCF_FIELD_LEN + MAC_SEQ_NUM_FIELD_LEN + MAC_FCS_FIELD_LEN =1+2+1+2=6字节

  /* clear any accidental threshold interrupt that happened as part of power up sequence */
  MAC_RADIO_CLEAR_RX_THRESHOLD_INTERRUPT_FLAG();

  /* enable threshold interrupts */
  MAC_RADIO_ENABLE_RX_THRESHOLD_INTERRUPT();
}


/**************************************************************************************************
 * @fn          macRxTxReset
 *
 * @brief       Reset the receive state.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macRxTxReset(void)
{
  /* forces receiver off, cleans up by calling macRxHaltCleanup() and macTxHaltCleanup() */
  macRxHardDisable();

  /*
   *   Note : transmit does not require any reset logic
   *          beyond what macRxHardDisable() provides.
   */

  /* restore deault filter mode to off */
  macRxFilter = RX_FILTER_OFF;

  /* return promiscuous mode to default off state */
  macRxPromiscuousMode(MAC_PROMISCUOUS_MODE_OFF);    //关闭混杂模式
}


/**************************************************************************************************
 * @fn          macRxHaltCleanup
 *
 * @brief       Cleanup up the receive logic after receiver is forced off.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macRxHaltCleanup(void)
{
  rxResetFlag = 1;
  if (!rxIsrActiveFlag)
  {
    rxHaltCleanupFinalStep();
    rxResetFlag = 0;
  }
}


/*=================================================================================================
 * @fn          rxHaltCleanupFinalStep
 *
 * @brief       Required cleanup if receiver is halted in the middle of a receive.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void rxHaltCleanupFinalStep(void)
{
  /* cancel any upcoming ACK transmit complete callback */
  MAC_RADIO_CANCEL_ACK_TX_DONE_CALLBACK();

  /* set start of frame threshold */
  MAC_RADIO_SET_RX_THRESHOLD(RX_THRESHOLD_START_LEN);     //重新设置成6字节

  /* flush the receive FIFO */
  MAC_RADIO_FLUSH_RX_FIFO();

  /* clear any receive interrupt that happened to squeak through */
  MAC_RADIO_CLEAR_RX_THRESHOLD_INTERRUPT_FLAG();

  /* if data buffer has been allocated, free it */       //如果分配了数据buffer，就释放它
  if (pRxBuf != NULL)
  {
    MEM_FREE((uint8 **)&pRxBuf);
  }
  pRxBuf = NULL; /* needed to indicate buffer is no longer allocated */

  pFuncRxState = &rxStartIsr;       //函数指针pFuncRxState指向rxStartIsr

  /* if receive was active, perform the post receive updates */
  if (macRxActive || macRxOutgoingAckFlag)
  {
    macRxActive = MAC_RX_ACTIVE_NO_ACTIVITY;
    macRxOutgoingAckFlag = 0;       //0 为不需要应答

    rxPostRxUpdates();
  }
}


/**************************************************************************************************
 * @fn          macRxThresholdIsr
 *
 * @brief       Interrupt service routine called when bytes in FIFO reach threshold value.   //重要！当FIFO中字节数达到阈值时，执行的中断服务程序
 *              It implements a state machine for receiving a packet.                        //读取packet的任何一个部分，都需要通过这个中断服务程序执行
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macRxThresholdIsr(void)
{
  /* if currently reseting, do not execute receive ISR logic */
  if (rxResetFlag)
  {
    return;
  }

  /*
   *  Call the function that handles the current receive state.
   *  A flag is set for the duration of the call to indicate
   *  the ISR is executing.  This is necessary for the reset
   *  logic so it does not perform a reset in the middle of
   *  executing the ISR.
   */
  rxIsrActiveFlag = 1;  //正在执行中断服务程序的标志位
  (*pFuncRxState)();    //执行当前该执行的中断服务程序，pFuncRxState的函数指针时刻在变
  rxIsrActiveFlag = 0;

  /* if a reset occurred during the ISR, peform cleanup here */
  if (rxResetFlag)
  {
    rxHaltCleanupFinalStep();
    rxResetFlag = 0;
  }
}


/*=================================================================================================
 * @fn          rxStartIsr
 *
 * @brief       First ISR state for receiving a packet - compute packet length, allocate         //第一步要执行的服务中断程序，包括计算包长度、分配buffer、初始化buffer
 *              buffer, initialize buffer.  Acknowledgements are handled immediately without     //如果是ACK，则先处理，不用分配buffer
 *              allocating a buffer.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void rxStartIsr(void)
{
  uint8  addrLen;         //记录地址长度的变量
  uint8  ackWithPending;
  uint8  dstAddrMode;     //目的地址模式
  uint8  srcAddrMode;     //源地址模式

  MAC_ASSERT(!macRxActive); /* receive on top of receive */

  /* indicate rx is active */
  macRxActive = MAC_RX_ACTIVE_STARTED;   

  /*
   *  For bullet proof functionality, need to see if the receiver was just turned off.
   *  The logic to request turning off the receiver, disables interrupts and then checks
   *  the value of macRxActive.  If it is TRUE, the receiver will not be turned off.
   *
   *  There is a small hole though.  It's possible to attempt turning off the receiver
   *  in the window from when the receive interrupt fires and the point where macRxActive
   *  is set to TRUE.  To plug this hole, the on/off status must be tested *after*
   *  macRxActive has been set.  If the receiver is off at this point, there is nothing
   *  in the RX fifo and the receive is simply aborted.
   *
   *  Also, there are some considerations in case a hard disable just happened.  Usually,
   *  the receiver will just be off at this point after a hard disable.  The check described
   *  above will account for this case too.  However, if a hard disable were immediately
   *  followed by an enable, the receiver would be on.  To catch this case, the receive
   *  FIFO is also tested to see if it is empty.  Recovery is identical to the other cases.
   */
  if (!macRxOnFlag || MAC_RADIO_RX_FIFO_IS_EMPTY())
  {
    /* reset active flag */
    macRxActive = MAC_RX_ACTIVE_NO_ACTIVITY;

    /*
     *  To be absolutely bulletproof, must make sure no transmit queue'ed up during
     *  the tiny, tiny window when macRxActive was not zero.
     */
    rxPostRxUpdates();

    /* return immediately from here */
    return;
  }

  /*
   *  If interrupts are held off for too long it's possible the previous "transmit done"
   *  callback is pending.  If this is the case, it needs to be completed before
   *  continuing with the receive logic.
   */
  MAC_RADIO_FORCE_TX_DONE_IF_PENDING();

  /*
   *  It's possible receive logic is still waiting for confirmation of an ACK that went out
   *  for the previous receive.  This is OK but the callback needs to be canceled at this point.
   *  That callback execute receive cleanup logic that will run at the completion
   *  of *this* receive.  Also, it is important the flag for the outgoing ACK to be cleared.
   */
  MAC_RADIO_CANCEL_ACK_TX_DONE_CALLBACK();
  macRxOutgoingAckFlag = 0;

  /*
   *  Make a module-local copy of macRxFilter.  This prevents the selected
   *  filter from changing in the middle of a receive.
   */
  rxFilter = macRxFilter;

  /*-------------------------------------------------------------------------------
   *  Read initial frame information from FIFO.
   *
   *   This code is not triggered until the following are in the RX FIFO:
   *     frame length          - one byte containing length of MAC frame (excludes this field)
   *     frame control field   - two bytes defining frame type, addressing fields, control flags
   *     sequence number       - one byte unique sequence identifier
   *     additional two bytes  - these bytes are available in case the received frame is an ACK,
   *                             if so, the frame can be verified and responded to immediately,
   *                             if not an ACK, these bytes will be processed normally
   */ //其实就是一个ACK的长度5字节 + 物理帧头1字节 =6字节

  /* read frame length, frame control field, and sequence number from FIFO */
  MAC_RADIO_READ_RX_FIFO(rxBuf, MAC_PHY_PHR_LEN + MAC_FCF_FIELD_LEN + MAC_SEQ_NUM_FIELD_LEN);   //读取5个字节到rxBuf

  /* bytes to read from FIFO equals frame length minus length of MHR fields just read from FIFO */
  rxUnreadLen = (rxBuf[0] & PHY_PACKET_SIZE_MASK) - MAC_FCF_FIELD_LEN - MAC_SEQ_NUM_FIELD_LEN;  //还未读取的字节数=物理帧头值（帧长度）- 2字节帧控制域 -1字节序列号

  /*
   *  Workaround for chip bug #1547.  The receive buffer can sometimes be corrupted by hardware.
   *  This usually occurs under heavy traffic.  If a corrupted receive buffer is detected
   *  the entire receive buffer is flushed.
   *
   *  In the case that this workaround is not needed, an assert is used to make sure the
   *  receive length field is not corrupted.  This is important because a corrupted receive
   *  length field is utterly fatal and, if not caught here, extremely hard to track down.
   */
  if (macChipVersion == REV_A)
  {
    if ((rxUnreadLen > (MAC_A_MAX_PHY_PACKET_SIZE - MAC_FCF_FIELD_LEN - MAC_SEQ_NUM_FIELD_LEN)) ||    //如果rxUnreadLen更大，说明PHY的物理帧头出错，保留位为1，导致PSDU的长度大于127字节，出错了
        (MAC_FRAME_TYPE(&rxBuf[1]) > MAC_FRAME_TYPE_MAX_VALID))  // MAC_FRAME_TYPE(&rxBuf[1]):取帧控制域的前3bit（帧类型），如果大于3，则出错，没有此类帧
    {
      MAC_RADIO_FLUSH_RX_FIFO();     // the entire receive buffer is flushed.
      rxDone();
      return;
    }
  }
  else
  {
    /* radio supplied a corrupted receive buffer length */
    MAC_ASSERT(rxUnreadLen <= (MAC_A_MAX_PHY_PACKET_SIZE - MAC_FCF_FIELD_LEN - MAC_SEQ_NUM_FIELD_LEN));   //闪灯以示错误
  }



  /*-------------------------------------------------------------------------------
   *  Process ACKs.
   *
   *  If this frame is an ACK, process it immediately and exit from here.    //如果是一个ACK帧，则立即处理，并从此处完成接收
   *  If this frame is not an ACK and transmit is listening for an ACK, let  //如果不是ACK，而发送状态正在等待一个ACK，则让transmit logic知道收到了一个非ACK帧，结束transmit logic
   *  the transmit logic know an non-ACK was received so transmit can complete.
   *
   *  In promiscuous mode ACKs are treated like any other frame.    //在混杂模式下，ACK帧与其它帧被同等对待
   */
  if ((MAC_FRAME_TYPE(&rxBuf[1]) == MAC_FRAME_TYPE_ACK) && (rxPromiscuousMode == PROMISCUOUS_MODE_OFF))     //识别出是应答帧，且不是在混杂模式
  {
    halIntState_t  s;
    uint8 fcsBuf[MAC_FCF_FIELD_LEN];  //定义2个字节的buf存储ACK帧的FCS校验码（其实并非原来的FCS校验码，而是包含了RSSI值与是否校验正确的2个byte，具体看PROPRIETARY_FCS_RSSI的定义）
    /*
     *  There are guaranteed to be two unread bytes in the FIFO.  By defintion, for ACK frames  //FIFO中至少还有2个字节的数据未读
     *  these two bytes will be the FCS.                                                        //如果是ACK帧，则是FCS校验位
     */

    /* read FCS from FIFO (threshold set so bytes are guaranteed to be there) */
    MAC_RADIO_READ_RX_FIFO(fcsBuf, MAC_FCS_FIELD_LEN);    //读取SFR中RFD的两个字节，即为ACK帧的FCS校验码

    /*
     *  This critical section ensures that the ACK timeout won't be triggered in the
     *  millde of receiving the ACK frame.
     */
    HAL_ENTER_CRITICAL_SECTION(s);

    /* see if transmit is listening for an ACK */     //看看是不是发射端在等待一个ACK
    if (macTxActive == MAC_TX_ACTIVE_LISTEN_FOR_ACK)
    {
      MAC_ASSERT(pMacDataTx != NULL); /* transmit buffer must be present */

      /* record link quality metrics for the receive ACK */     //记录RSSI值和correlation
      {
        int8 rssiDbm;
        uint8 corr;

        rssiDbm = PROPRIETARY_FCS_RSSI(fcsBuf) + MAC_RADIO_RSSI_OFFSET;   //获取RSSI，原来在fcsBuf的第一个字节中
        MAC_RADIO_RSSI_LNA_OFFSET(rssiDbm);
        corr = PROPRIETARY_FCS_CORRELATION_VALUE(fcsBuf);    //(fcsBuf)[1] & 0x7F,  存储在fcsBuf的第二个字节的低7位

        pMacDataTx->internal.mpduLinkQuality = macRadioComputeLQI(rssiDbm, corr);   //其实并没有用corr来计算LQI，只用了RSSI，计算ED，即作为LQI
        pMacDataTx->internal.correlation = corr;      //
        pMacDataTx->internal.rssi= rssiDbm;
      }

      /*
       *  It's okay if the ACK timeout is triggered here. The callbacks for ACK received
       *  or ACK not received will check "macTxActive" flag before taking any actions.
       */
      HAL_EXIT_CRITICAL_SECTION(s);

      /*
       *  An ACK was received so transmit logic needs to know.  If the FCS failed,
       *  the transmit logic still needs to know.  In that case, treat the frame
       *  as a non-ACK to complete the active transmit.
       */
      if (PROPRIETARY_FCS_CRC_OK(fcsBuf))    //检验fcsBuf的第二个字节的最高位是否为1，如果是1，则FCS检验成功，如果是0，则校验失败
      {
        /* call transmit logic to indicate ACK was received */
        macTxAckReceivedCallback(MAC_SEQ_NUMBER(&rxBuf[1]), MAC_FRAME_PENDING(&rxBuf[1]));   //记录序列号
      }
      else     //FCS检验错误
      {
        macTxAckNotReceivedCallback();
      }
    }
    else
    {
      HAL_EXIT_CRITICAL_SECTION(s);
    }

    /* receive is done, exit from here */   //接收完毕
    rxDone();
    return;
  }
  else if (macTxActive == MAC_TX_ACTIVE_LISTEN_FOR_ACK)   //ACK超时,或其他问题
  {
    macTxAckNotReceivedCallback();    
  }

  /*-------------------------------------------------------------------------------
   *  Apply filtering.
   *
   *  For efficiency, see if filtering is even 'on' before processing.  Also test
   *  to make sure promiscuous mode is disabled.  If promiscuous mode is enabled,
   *  do not apply filtering.
   */
  if ((rxFilter != RX_FILTER_OFF) && !rxPromiscuousMode)
  {
    if (/* filter all frames */
         (rxFilter == RX_FILTER_ALL) ||

         /* filter non-beacon frames */
         ((rxFilter == RX_FILTER_NON_BEACON_FRAMES) &&
          (MAC_FRAME_TYPE(&rxBuf[1]) != MAC_FRAME_TYPE_BEACON)) ||

         /* filter non-command frames */
         ((rxFilter == RX_FILTER_NON_COMMAND_FRAMES) &&
          ((MAC_FRAME_TYPE(&rxBuf[1]) != MAC_FRAME_TYPE_COMMAND))))
    {
      /* discard rest of frame */
      rxDiscardFrame();
      return;
    }
  }

  /*-------------------------------------------------------------------------------
   *  Compute length of addressing fields.  Compute payload length.
   */

  /* decode addressing modes */
  dstAddrMode = MAC_DEST_ADDR_MODE(&rxBuf[1]);    //取目的地址模式，在帧控制域中，Bit:10~11
  srcAddrMode = MAC_SRC_ADDR_MODE(&rxBuf[1]);     //取源地址模式，在帧控制域中，Bit:14~15

  /*
  *  Workaround for chip bug #1547.  The receive buffer can sometimes be corrupted by hardware.
   *  This usually occurs under heavy traffic.  If a corrupted receive buffer is detected
   *  the entire receive buffer is flushed.
   */
  if (macChipVersion == REV_A)
  {
    if ((srcAddrMode == ADDR_MODE_RESERVERED) || (dstAddrMode == ADDR_MODE_RESERVERED))   //当源地址模式或目的地址模式的两个bit是01这一保留值时，认为出错了
    {
      MAC_RADIO_FLUSH_RX_FIFO();
      rxDone();
      return;
    }
  }

  /*
   *  Compute the addressing field length.  A lookup table based on addressing
   *  mode is used for efficiency.  If the source address is present and the
   *  frame is intra-PAN, the PAN Id is not repeated.  In this case, the address
   *  length is adjusted to match the smaller length.
   */
  addrLen = macRxAddrLen[dstAddrMode] + macRxAddrLen[srcAddrMode];     //计算MHR中地址域的总字节数（包括4项：目的PAN标识符 + 目的地址域 + 源PAN标识符 + 源地址域 ）
  if ((srcAddrMode != SADDR_MODE_NONE) && MAC_INTRA_PAN(&rxBuf[1]))   //判断是否内部PAN，标志位在帧控制域的Bit:6
  {
    addrLen -= MAC_PAN_ID_FIELD_LEN;    //如果是，则减去PAN ID所占的2个字节
  }

  /*
   *  If there are not enough unread bytes to include the computed address
   *  plus FCS field, the frame is corrupted and must be discarded.
   */
  if ((addrLen + MAC_FCS_FIELD_LEN) > rxUnreadLen)  //如果出现地址域字节数 + 2字节的FCS > 未读的字节数 ，则收到的包出错了
  {
    /* discard frame and exit */
    rxDiscardFrame();
    return;
  }

  /* payload length is equal to unread bytes minus address length, minus the FCS */
  rxPayloadLen = rxUnreadLen - addrLen - MAC_FCS_FIELD_LEN;         //payload的字节数=待读字节数-地址域字节数-2字节的FCS

  /*-------------------------------------------------------------------------------
   *  Allocate memory for the incoming frame.
   */
  pRxBuf = (macRx_t *) MEM_ALLOC(sizeof(macRx_t) + rxPayloadLen);
  if (pRxBuf == NULL)                  //buffer分配失败
  {
    /* Cancel the outgoing TX ACK */
    MAC_RADIO_CANCEL_TX_ACK();

    /* buffer allocation failed, discard the frame and exit*/
    rxDiscardFrame();
    return;
  }

  /*-------------------------------------------------------------------------------
   *  Set up to process ACK request.  Do not ACK if in promiscuous mode.
   */
  ackWithPending = 0;
  if (!rxPromiscuousMode)    //如果是非混杂模式
  {
    macRxOutgoingAckFlag = MAC_ACK_REQUEST(&rxBuf[1]);    //将帧控制域中的ACK请求位赋值给macRxOutgoingAckFlag
  }

  /*-------------------------------------------------------------------------------
   *  Process any ACK request.
   */
  if (macRxOutgoingAckFlag)    //如果需要接收方返回应答
  {
    halIntState_t  s;

    /*
     *  This critical section ensures that the callback ISR is initiated within time
     *  to guarantee correlation with the strobe.
     */
    HAL_ENTER_CRITICAL_SECTION(s);

    /* Do not ack data packet with pending more data */
    if( MAC_FRAME_TYPE(&rxBuf[1]) == MAC_FRAME_TYPE_COMMAND )  //如果是命令帧
    {
      if( macRxCheckMACPendingCallback())        //被封装的函数，看不到
      {
        /* Check is any mac data pending for end devices */
        ackWithPending = MAC_RX_FLAG_ACK_PENDING;
      }
      else
      {
        if( macSrcMatchIsEnabled )
        {
          /* When autopend is enabled, check if allpending is set to true */
          if( MAC_SrcMatchCheckAllPending() == MAC_AUTOACK_PENDING_ALL_ON )
          {
            ackWithPending = MAC_RX_FLAG_ACK_PENDING;
          }
        }
        else
        {
          /* When autopend is disabled, check the application pending callback */
          if( macRxCheckPendingCallback() )
          {
            ackWithPending = MAC_RX_FLAG_ACK_PENDING;
          }
        }
      }
    }

    if( ackWithPending == MAC_RX_FLAG_ACK_PENDING )
    {
      MAC_RADIO_TX_ACK_PEND();
    }
    else
    {
      MAC_RADIO_TX_ACK();
    }


    /* request a callback to macRxAckTxDoneCallback() when the ACK transmit has finished */
    MAC_RADIO_REQUEST_ACK_TX_DONE_CALLBACK();
    HAL_EXIT_CRITICAL_SECTION(s);
  }

 /*-------------------------------------------------------------------------------
  *  Populate the receive buffer going up to high-level. 将收到的数据向上层传递
  */

  /* configure the payload buffer */
  pRxBuf->msdu.p = (uint8 *) (pRxBuf + 1);    //payload的指针
  pRxBuf->msdu.len = rxPayloadLen;            //payload的长度

  /* set internal values */
  pRxBuf->mac.srcAddr.addrMode  = srcAddrMode;    //记录源地址模式
  pRxBuf->mac.dstAddr.addrMode  = dstAddrMode;    //记录目的地址模式
  pRxBuf->mac.timestamp         = MAC_RADIO_BACKOFF_CAPTURE();
  pRxBuf->mac.timestamp2        = MAC_RADIO_TIMER_CAPTURE();
  pRxBuf->internal.frameType    = MAC_FRAME_TYPE(&rxBuf[1]);    //记录MAC帧类型
  pRxBuf->mac.dsn               = MAC_SEQ_NUMBER(&rxBuf[1]);    //记录帧序列号
  pRxBuf->internal.flags        = INTERNAL_FCF_FLAGS(&rxBuf[1]) | ackWithPending;
  pRxBuf->sec.securityLevel     = MAC_SEC_LEVEL_NONE;

  /*-------------------------------------------------------------------------------
   *  If the processing the addressing fields does not require more bytes from
   *  the FIFO go directly address processing function.  Otherwise, configure
   *  interrupt to jump there once bytes are received.
   */
  if (addrLen == 0)  //地址域长度为0byte
  {
    /* no addressing fields to read, prepare for payload interrupts */
    pFuncRxState = &rxPayloadIsr;  //pFuncRxState函数指针指向处理payload的中断服务程序
    rxPrepPayload();      //不用处理地址，直接准备接收处理payload
  }
  else
  {
    /* need to read and process addressing fields, prepare for address interrupt */
    rxNextLen = addrLen;     // rxNextLen等于地址域长度
    MAC_RADIO_SET_RX_THRESHOLD(rxNextLen);   //需要处理地址域的数据，接收地址域
    pFuncRxState = &rxAddrIsr;      //pFuncRxState函数指针指向处理地址的中断服务程序rxAddrIsr
  }
}


/*=================================================================================================
 * @fn          rxAddrIsr         //处理地址的中断服务程序
 *
 * @brief       Receive ISR state for decoding address.  Reads and stores the address information
 *              from the incoming packet.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void rxAddrIsr(void)
{
  uint8 buf[MAX_ADDR_FIELDS_LEN];   //分配buffer
  uint8 dstAddrMode;
  uint8 srcAddrMode;
  uint8  * p;

  MAC_ASSERT(rxNextLen != 0); /* logic assumes at least one address byte in buffer */

  /*  read out address fields into local buffer in one shot */
  MAC_RADIO_READ_RX_FIFO(buf, rxNextLen);       //读取地址域到buf

  /* set pointer to buffer with addressing fields */
  p = buf;          //将指针p指向buf

  /* destination address */
  dstAddrMode = MAC_DEST_ADDR_MODE(&rxBuf[1]);     //目的地址模式
  if (dstAddrMode != SADDR_MODE_NONE)   
  {
    pRxBuf->mac.srcPanId = pRxBuf->mac.dstPanId = BUILD_UINT16(p[0], p[1]);  //读取PAN id
    p += MAC_PAN_ID_FIELD_LEN;  //指针p前移2字节，buf数字前移2字节，将指向目的地址所在字节
    if (dstAddrMode == SADDR_MODE_EXT)   //目的地址包含64位扩展地址
    {
      sAddrExtCpy(pRxBuf->mac.dstAddr.addr.extAddr, p);   //将目的地址（扩展）复制到pRxBuf->mac.dstAddr.addr.extAddr中
      p += MAC_EXT_ADDR_FIELD_LEN;      //指针p前移8字节，buf数字前移2字节，将指向源地址所在字节
    }
    else     //目的地址包含16位短地址
    {
      pRxBuf->mac.dstAddr.addr.shortAddr = BUILD_UINT16(p[0], p[1]);  //将目的地址（短）复制到pRxBuf->mac.dstAddr.addr.shortAddr中
      p += MAC_SHORT_ADDR_FIELD_LEN;    //指针p前移2字节，buf数字前移2字节，将指向源地址所在字节
    }
  }

  /* sources address */
  srcAddrMode = MAC_SRC_ADDR_MODE(&rxBuf[1]);    //源地址模式
  if (srcAddrMode != SADDR_MODE_NONE)
  {
    if (!(pRxBuf->internal.flags & MAC_RX_FLAG_INTRA_PAN))
    {
      pRxBuf->mac.srcPanId = BUILD_UINT16(p[0], p[1]);   //读取源地址的PAN id
      p += MAC_PAN_ID_FIELD_LEN;
    }
    if (srcAddrMode == SADDR_MODE_EXT)    //源地址包含64位扩展地址
    {
      sAddrExtCpy(pRxBuf->mac.srcAddr.addr.extAddr, p);      //将源地址（扩展）复制到pRxBuf->mac.srcAddr.addr.extAddr中
    }
    else      //源地址包含16位短地址
    {
      pRxBuf->mac.srcAddr.addr.shortAddr = BUILD_UINT16(p[0], p[1]);  //将源地址（扩展）复制到pRxBuf->mac.srcAddr.addr.shortAddr中
    }
  }

  /*-------------------------------------------------------------------------------
   *  Prepare for payload interrupts.
   */
  pFuncRxState = &rxPayloadIsr;       //pFuncRxState函数指针指向处理接收数据的中断服务程序rxPayloadIsr
  rxPrepPayload();            //执行准备接收数据的函数
}


/*=================================================================================================
 * @fn          rxPrepPayload
 *
 * @brief       Common code to prepare for the payload ISR.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void rxPrepPayload(void)
{
  if (rxPayloadLen == 0)    //如果rxPayloadLen=0，即无payload
  {
    MAC_RADIO_SET_RX_THRESHOLD(MAC_FCS_FIELD_LEN);   //准备FCS校验
    pFuncRxState = &rxFcsIsr;      //pFuncRxState函数指针指向处理FCS的中断服务程序rxFcsIsr
  }
  else
  {
    rxNextLen = MIN(rxPayloadLen, MAX_PAYLOAD_BYTES_READ_PER_INTERRUPT);   //每次最多读取16个字节的数据，（16这个值可调）
    MAC_RADIO_SET_RX_THRESHOLD(rxNextLen);    //接收rxNextLen个字节的数据
  }
}


/*=================================================================================================
 * @fn          rxPayloadIsr
 *
 * @brief       Receive ISR state for reading out and storing the packet payload.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void rxPayloadIsr(void)
{
  MAC_RADIO_READ_RX_FIFO(pRxBuf->msdu.p, rxNextLen);   //将rxNextLen个数据读取到pRxBuf->msdu.p中
  
  //add by jionghong  我自己加的，为了输出读取到的payload
     halIntState_t   intState;
  // Hold off interrupts
  HAL_ENTER_CRITICAL_SECTION(intState);
 // HalUARTWrite(0, &rxNextLen , 1);
  HalUARTWrite(0, pRxBuf->msdu.p , rxNextLen);
 // HalUARTWrite(0, "\n", 1);         //回车换行
  // Release interrupts
  HAL_EXIT_CRITICAL_SECTION(intState);
  
  
  
  pRxBuf->msdu.p += rxNextLen;    //指针指向的更新，随着数据的接收，不断向前
  rxPayloadLen -= rxNextLen;      //更新待读取的payload字节数
  
  rxPrepPayload();   //重新设置读取payload的字节数
}


/*=================================================================================================
 * @fn          rxFcsIsr
 *
 * @brief       Receive ISR state for handling the FCS.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void rxFcsIsr(void)
{
  uint8 crcOK;    //用于记录是否通过CRC校验的变量
  uint8 ackWithPending = 0;

  /* read FCS, rxBuf is now available storage */
  MAC_RADIO_READ_RX_FIFO(rxBuf, MAC_FCS_FIELD_LEN);   //读取FCS的2个byte，（其实并非原来的FCS校验码，而是包含了RSSI值与是否校验正确的2个byte，具体看PROPRIETARY_FCS_RSSI的定义）

  /*
   *  The FCS has actually been replaced within the radio by a proprietary version of the FCS.
   *  This proprietary FCS is two bytes (same length as the real FCS) and contains:
   *    1) the RSSI value
   *    2) the average correlation value (used for LQI)
   *    3) a CRC passed bit
   */

  /* save the "CRC-is-OK" status */
  crcOK = PROPRIETARY_FCS_CRC_OK(rxBuf);      //读取CRC校验结果 

  /*
   *  See if the frame should be passed up to high-level MAC.  If the CRC is OK, the
   *  the frame is always passed up.  Frames with a bad CRC are also passed up *if*
   *  a special variant of promiscuous mode is active.
   */
  
  //add by jionghong
 // rxPromiscuousMode = MAC_PROMISCUOUS_MODE_WITH_BAD_CRC;
  
  if ( crcOK || (rxPromiscuousMode == MAC_PROMISCUOUS_MODE_WITH_BAD_CRC))    //如果CRC校验通过，且是非混杂模式
  {
    int8 rssiDbm;
    uint8 corr;

    /*
     *  As power saving optimization, set state variable to indicate physical receive
     *  has completed and then request turning of the receiver.  This means the receiver
     *  can be off (if other conditions permit) during execution of the callback function.
     *
     *  The receiver will be requested to turn off once again at the end of the receive
     *  logic.  There is no harm in doing this.
     */
    macRxActive = MAC_RX_ACTIVE_DONE;
    macRxOffRequest();

    /* decode RSSI and correlation values */
    rssiDbm = PROPRIETARY_FCS_RSSI(rxBuf) + MAC_RADIO_RSSI_OFFSET;      //提取RSSI值，并加上offest
    MAC_RADIO_RSSI_LNA_OFFSET(rssiDbm);
    corr = PROPRIETARY_FCS_CORRELATION_VALUE(rxBuf);      //提取correlation值

    /* Read the source matching result back */
    if( macSrcMatchIsEnabled && MAC_RADIO_SRC_MATCH_RESULT() )
    {
      /* This result will not overwrite the previously determined pRxBuf->internal.flags */
      ackWithPending = MAC_RX_FLAG_ACK_PENDING;
    }

    /* record parameters that get passed up to high-level */
    pRxBuf->internal.flags |= ( crcOK | ackWithPending );
    pRxBuf->mac.mpduLinkQuality = macRadioComputeLQI(rssiDbm, corr);
    pRxBuf->mac.rssi = rssiDbm;
    pRxBuf->mac.correlation = corr;

    /* set the MSDU pointer to point at start of data */
    pRxBuf->msdu.p = (uint8 *) (pRxBuf + 1);
    
    /* finally... execute callback function */
    macRxCompleteCallback(pRxBuf);
    pRxBuf = NULL; /* needed to indicate buffer is no longer allocated */
  }
  else   //如果CRC出错
  {
    /*
     *  The CRC is bad so no ACK was sent.  Cancel any callback and clear the flag.
     *  (It's OK to cancel the outgoing ACK even if an ACK was not requested.  It's
     *  slightly more efficient to do so.)
     */
    
    
    MAC_RADIO_CANCEL_ACK_TX_DONE_CALLBACK();
    macRxOutgoingAckFlag = 0;

    /* the CRC failed so the packet must be discarded */
    MEM_FREE((uint8 **)&pRxBuf);
    pRxBuf = NULL;  /* needed to indicate buffer is no longer allocated */
  }

  /* reset threshold level, reset receive state, and complete receive logic */
  MAC_RADIO_SET_RX_THRESHOLD(RX_THRESHOLD_START_LEN);   //一个接收任务完成，重置一些初始值
  pFuncRxState = &rxStartIsr;       //pFuncRxState函数指针指向处刚接收packet的中断服务程序rxStartIsr
  rxDone();
}


/*=================================================================================================
 * @fn          rxDone
 *
 * @brief       Common exit point for receive.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void rxDone(void)
{
  /* if the receive FIFO has overflowed, flush it here */
  if (MAC_RADIO_RX_FIFO_HAS_OVERFLOWED())
  {
    MAC_RADIO_FLUSH_RX_FIFO();
  }

  /* mark receive as inactive */
  macRxActive = MAC_RX_ACTIVE_NO_ACTIVITY;

  /* if there is no outgoing ACK, run the post receive updates */
  if (!macRxOutgoingAckFlag)
  {
    rxPostRxUpdates();
  }
}


/**************************************************************************************************
 * @fn          macRxAckTxDoneCallback
 *
 * @brief       Function called when the outoing ACK has completed transmitting.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macRxAckTxDoneCallback(void)
{
  macRxOutgoingAckFlag = 0;

  /*
   *  With certain interrupt priorities and timing conditions, it is possible this callback
   *  could be executed before the primary receive logic completes.  To prevent this, the
   *  post updates are only executed if receive logic is no longer active.  In the case the
   *  post updates are not executed here, they will execute when the main receive logic
   *  completes.
   */
  if (!macRxActive)
  {
    rxPostRxUpdates();
  }
}


/*=================================================================================================
 * @fn          rxPostRxUpdates
 *
 * @brief       Updates that need to be performed once receive is complete.
 *
 *              It is not fatal to execute this function if somehow receive is active.  Under
 *              certain timing/interrupt conditions a new receive may have started before this
 *              function executes.  This should happen very rarely (if it happens at all) and
 *              would cause no problems.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void rxPostRxUpdates(void)
{
  /* turn off receiver if permitted */
  macRxOffRequest();

  /* update the transmit power, update may have been blocked by transmit of outgoing ACK */
  macRadioUpdateTxPower();

  /* initiate and transmit that was queued during receive */
  macTxStartQueuedFrame();
}


/*=================================================================================================
 * @fn          rxDiscardFrame
 *
 * @brief       Initializes for discarding a packet.  Must be called before ACK is strobed.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void rxDiscardFrame(void)
{
  MAC_ASSERT(pFuncRxState == &rxStartIsr); /* illegal state for calling discard frame function */

  if (rxUnreadLen == 0)
  {
    rxDone();
  }
  else
  {
    rxNextLen = MIN(rxUnreadLen, MAX_PAYLOAD_BYTES_READ_PER_INTERRUPT);
    MAC_RADIO_SET_RX_THRESHOLD(rxNextLen);
    pFuncRxState = &rxDiscardIsr;
  }
}


/*=================================================================================================
 * @fn          rxDiscardIsr
 *
 * @brief       Receive ISR state for discarding a packet.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void rxDiscardIsr(void)
{
  uint8 buf[MAX_PAYLOAD_BYTES_READ_PER_INTERRUPT];

  MAC_RADIO_READ_RX_FIFO(buf, rxNextLen);
  rxUnreadLen -= rxNextLen;

  /* read out and discard bytes until all bytes of packet are disposed of */
  if (rxUnreadLen != 0)
  {
    if (rxUnreadLen < MAX_PAYLOAD_BYTES_READ_PER_INTERRUPT)
    {
      rxNextLen = rxUnreadLen;
      MAC_RADIO_SET_RX_THRESHOLD(rxNextLen);
    }
  }
  else
  {
    /* reset threshold level, reset receive state, and complete receive logic */
    MAC_RADIO_SET_RX_THRESHOLD(RX_THRESHOLD_START_LEN);
    pFuncRxState = &rxStartIsr;
    rxDone();
  }
}


/**************************************************************************************************
 * @fn          maxRxRifoOverflowIsr
 *
 * @brief       This interrupt service routine is called when RX FIFO overflow. Note that this
 *              exception does not retrieve the good frames that are trapped in the RX FIFO.
 *              It simply halts and cleanup the RX.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macRxFifoOverflowIsr(void)
{
  rxFifoOverflowCount++; /* This flag is used for debug purpose only */
  macRxHaltCleanup();
}


/**************************************************************************************************
 * @fn          macRxPromiscuousMode
 *
 * @brief       Sets promiscuous mode - enabling or disabling it.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macRxPromiscuousMode(uint8 mode)
{
  rxPromiscuousMode = mode;

  if (rxPromiscuousMode == MAC_PROMISCUOUS_MODE_OFF)
  {
      MAC_RADIO_TURN_ON_RX_FRAME_FILTERING();       //关闭混杂模式
  }
  else
  {
    MAC_ASSERT((mode == MAC_PROMISCUOUS_MODE_WITH_BAD_CRC)   ||
               (mode == MAC_PROMISCUOUS_MODE_COMPLIANT));  /* invalid mode */

    MAC_RADIO_TURN_OFF_RX_FRAME_FILTERING();        //开启混杂模式
  }
}



/**************************************************************************************************
 *                                  Compile Time Integrity Checks
 **************************************************************************************************
 */

/* check for changes to the spec that would affect the source code */
#if ((MAC_A_MAX_PHY_PACKET_SIZE   !=  0x7F )   ||  \
     (MAC_FCF_FIELD_LEN           !=  2    )   ||  \
     (MAC_FCF_FRAME_TYPE_POS      !=  0    )   ||  \
     (MAC_FCF_FRAME_PENDING_POS   !=  4    )   ||  \
     (MAC_FCF_ACK_REQUEST_POS     !=  5    )   ||  \
     (MAC_FCF_INTRA_PAN_POS       !=  6    )   ||  \
     (MAC_FCF_DST_ADDR_MODE_POS   !=  10   )   ||  \
     (MAC_FCF_FRAME_VERSION_POS   !=  12   )   ||  \
     (MAC_FCF_SRC_ADDR_MODE_POS   !=  14   ))
#error "ERROR!  Change to the spec that requires modification of source code."
#endif

/* check for changes to the internal flags format */
#if ((MAC_RX_FLAG_VERSION      !=  0x03)  ||  \
     (MAC_RX_FLAG_ACK_PENDING  !=  0x04)  ||  \
     (MAC_RX_FLAG_SECURITY     !=  0x08)  ||  \
     (MAC_RX_FLAG_PENDING      !=  0x10)  ||  \
     (MAC_RX_FLAG_ACK_REQUEST  !=  0x20)  ||  \
     (MAC_RX_FLAG_INTRA_PAN    !=  0x40))
#error "ERROR!  Change to the internal RX flags format.  Requires modification of source code."
#endif

/* validate CRC OK bit optimization */
#if (MAC_RX_FLAG_CRC_OK != PROPRIETARY_FCS_CRC_OK_BIT)
#error "ERROR!  Optimization relies on these bits having the same position."
#endif

#if (MAC_RX_ACTIVE_NO_ACTIVITY != 0x00)
#error "ERROR! Zero is reserved value of macRxActive. Allows boolean operations, e.g !macRxActive."
#endif

#if (MAC_PROMISCUOUS_MODE_OFF != 0x00)
#error "ERROR! Zero is reserved value of rxPromiscuousMode. Allows boolean operations, e.g !rxPromiscuousMode."
#endif


/**************************************************************************************************
*/
