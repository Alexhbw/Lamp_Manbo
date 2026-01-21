#include "miniPC_process.h"
#include "string.h"
#include "robot_def.h"
#include "daemon.h"

static Nac_Instance *nac_instance; // 用于和视觉通信的串口实例
static uint8_t *vis_recv_buff __attribute__((unused));
static Daemon_Instance *nac_daemon_instance;








// 全局变量区
extern uint16_t CRC_INIT;
/**
 * @brief 处理视觉传入的数据
 *
 * @param recv
 * @param rx_buff
 */
static void RecvProcess(Nac_Recv_s *recv, uint8_t *rx_buff)
{
	recv->x     = (int16_t)(rx_buff[1]<<8 | (rx_buff[2]));
	recv->y     = (int16_t)(rx_buff[3]<<8 | (rx_buff[4]));
	
	recv->get   = (int)rx_buff[5];
}

/**
 * @brief 回调函数，确认帧头后用于解析视觉数据
 *
 */
static void DecodeNac()
{
    DaemonReload(nac_daemon_instance); // 喂狗


    if (nac_instance->usart->recv_buff[0] == nac_instance->recv_data->header1  &&nac_instance->usart->recv_buff[6] == nac_instance->recv_data->tail) {
        // 读取视觉数据
        RecvProcess(nac_instance->recv_data, nac_instance->usart->recv_buff);
    }

}


/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void NacOfflineCallback(void *id)
{

    USARTServiceInit(nac_instance->usart);

}



/**
 * @brief 发送数据处理函数
 *
 * @param send 待发送数据
 * @param tx_buff 发送缓冲区
 *
 */
static void SendProcess(Nac_Send_s *send, uint8_t *tx_buff,uint8_t set)
{
		tx_buff[0]      = send->header;
	
		tx_buff[1]      = set;
	
		tx_buff[2]     = send->tail;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void NacSend(uint8_t set)
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    SendProcess(nac_instance->send_data, send_buff,set);
    USARTSend(nac_instance->usart, send_buff, VISION_SEND_SIZE, USART_TRANSFER_BLOCKING);
}

/**
 * @brief 用于注册一个视觉接收数据结构体,返回一个视觉接收数据结构体指针
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Nac_Recv_s *NacRecvRegister(Nac_Recv_Init_Config_s *recv_config)
{
    Nac_Recv_s *recv_data = (Nac_Recv_s *)malloc(sizeof(Nac_Recv_s));
    memset(recv_data, 0, sizeof(Nac_Recv_s));

    recv_data->header1 = recv_config->header1;
	
		recv_data->tail = recv_config->tail;

    return recv_data;
}

/**
 * @brief 用于注册一个视觉发送数据结构体,返回一个视觉发送数据结构体指针
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Nac_Send_s *NacSendRegister(Nac_Send_Init_Config_s *send_config)
{
    Nac_Send_s *send_data = (Nac_Send_s *)malloc(sizeof(Nac_Send_s));
    memset(send_data, 0, sizeof(Nac_Send_s));

    send_data->header   = send_config->header;
    send_data->tail     = send_config->tail;
    return send_data;
}

Nac_Recv_s *NacInit(UART_HandleTypeDef *nac_usart_handle)
{
    nac_instance = (Nac_Instance *)malloc(sizeof(Nac_Instance));
    memset(nac_instance, 0, sizeof(Nac_Instance));
	
		USART_Init_Config_s conf;
    conf.module_callback = DecodeNac;
    conf.recv_buff_size  = VISION_RECV_SIZE;
    conf.usart_handle    = nac_usart_handle;

    nac_instance->usart                = USARTRegister(&conf);
    Nac_Recv_Init_Config_s recv_config = {
        .header1 = NAC_RECV_HEADER1,

			  .tail    = NAC_RECV_TAIL,
    };

    nac_instance->recv_data            = NacRecvRegister(&recv_config);
    Nac_Send_Init_Config_s send_config = {
        .header        = NAC_SEND_HEADER,
         .tail          = NAC_SEND_TAIL,
    };
    nac_instance->send_data = NacSendRegister(&send_config);
    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback     = NacOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id     = NULL,
        .reload_count = 5, // 50ms
    };
    nac_daemon_instance = DaemonRegister(&daemon_conf);

    return nac_instance->recv_data;
}
