/*
 * @version: no version
 * @LastEditors: qingmeijiupiao
 * @Description: MCP2515封装类，继承HXC_CAN
 * @author: qingmeijiupiao
 * @LastEditTime: 2025-07-24 11:29:44
 */
#ifndef HXC_MCP2515_HPP
#define HXC_MCP2515_HPP
#include "HXC_CAN.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"


// 使用延时模式接收CAN消息
// 延时模式:CPU占用低，但是总线数据在2000HZ以上时会丢包
// 非延时模式:CPU占用高，但是总线数据接收速度更高
#define USE_NONE_DELAY_RECEIVE 0

// MCP2515 寄存器枚举
enum MCP2515_REGISTER : uint8_t {
    MCP_RXF0SIDH = 0x00, // 接收过滤器0的标准标识符高字节
    MCP_RXF0SIDL = 0x01, // 接收过滤器0的标准标识符低字节
    MCP_RXF0EID8 = 0x02, // 接收过滤器0的扩展标识符高字节
    MCP_RXF0EID0 = 0x03, // 接收过滤器0的扩展标识符低字节
    MCP_RXF1SIDH = 0x04, // 接收过滤器1的标准标识符高字节
    MCP_RXF1SIDL = 0x05, // 接收过滤器1的标准标识符低字节
    MCP_RXF1EID8 = 0x06, // 接收过滤器1的扩展标识符高字节
    MCP_RXF1EID0 = 0x07, // 接收过滤器1的扩展标识符低字节
    MCP_RXF2SIDH = 0x08, // 接收过滤器2的标准标识符高字节
    MCP_RXF2SIDL = 0x09, // 接收过滤器2的标准标识符低字节
    MCP_RXF2EID8 = 0x0A, // 接收过滤器2的扩展标识符高字节
    MCP_RXF2EID0 = 0x0B, // 接收过滤器2的扩展标识符低字节
    MCP_CANSTAT  = 0x0E, // CAN状态寄存器
    MCP_CANCTRL  = 0x0F, // CAN控制寄存器
    MCP_RXF3SIDH = 0x10, // 接收过滤器3的标准标识符高字节
    MCP_RXF3SIDL = 0x11, // 接收过滤器3的标准标识符低字节
    MCP_RXF3EID8 = 0x12, // 接收过滤器3的扩展标识符高字节
    MCP_RXF3EID0 = 0x13, // 接收过滤器3的扩展标识符低字节
    MCP_RXF4SIDH = 0x14, // 接收过滤器4的标准标识符高字节
    MCP_RXF4SIDL = 0x15, // 接收过滤器4的标准标识符低字节
    MCP_RXF4EID8 = 0x16, // 接收过滤器4的扩展标识符高字节
    MCP_RXF4EID0 = 0x17, // 接收过滤器4的扩展标识符低字节
    MCP_RXF5SIDH = 0x18, // 接收过滤器5的标准标识符高字节
    MCP_RXF5SIDL = 0x19, // 接收过滤器5的标准标识符低字节
    MCP_RXF5EID8 = 0x1A, // 接收过滤器5的扩展标识符高字节
    MCP_RXF5EID0 = 0x1B, // 接收过滤器5的扩展标识符低字节
    MCP_TEC      = 0x1C, // 传输错误计数器
    MCP_REC      = 0x1D, // 接收错误计数器
    MCP_RXM0SIDH = 0x20, // 接收掩码0的标准标识符高字节
    MCP_RXM0SIDL = 0x21, // 接收掩码0的标准标识符低字节
    MCP_RXM0EID8 = 0x22, // 接收掩码0的扩展标识符高字节
    MCP_RXM0EID0 = 0x23, // 接收掩码0的扩展标识符低字节
    MCP_RXM1SIDH = 0x24, // 接收掩码1的标准标识符高字节
    MCP_RXM1SIDL = 0x25, // 接收掩码1的标准标识符低字节
    MCP_RXM1EID8 = 0x26, // 接收掩码1的扩展标识符高字节
    MCP_RXM1EID0 = 0x27, // 接收掩码1的扩展标识符低字节
    MCP_CNF3     = 0x28, // 配置寄存器3
    MCP_CNF2     = 0x29, // 配置寄存器2
    MCP_CNF1     = 0x2A, // 配置寄存器1
    MCP_CANINTE  = 0x2B, // CAN中断使能寄存器
    MCP_CANINTF  = 0x2C, // CAN中断标志寄存器
    MCP_EFLG     = 0x2D, // 错误标志寄存器
    MCP_TXB0CTRL = 0x30, // 传输缓冲区0控制寄存器
    MCP_TXB0SIDH = 0x31, // 传输缓冲区0的标准标识符高字节
    MCP_TXB0SIDL = 0x32, // 传输缓冲区0的标准标识符低字节
    MCP_TXB0EID8 = 0x33, // 传输缓冲区0的扩展标识符高字节
    MCP_TXB0EID0 = 0x34, // 传输缓冲区0的扩展标识符低字节
    MCP_TXB0DLC  = 0x35, // 传输缓冲区0的数据长度码
    MCP_TXB0DATA = 0x36, // 传输缓冲区0的数据，0x36后面7个地址分别为传输缓冲区1的数据
    MCP_TXB1CTRL = 0x40, // 传输缓冲区1控制寄存器
    MCP_TXB1SIDH = 0x41, // 传输缓冲区1的标准标识符高字节
    MCP_TXB1SIDL = 0x42, // 传输缓冲区1的标准标识符低字节
    MCP_TXB1EID8 = 0x43, // 传输缓冲区1的扩展标识符高字节
    MCP_TXB1EID0 = 0x44, // 传输缓冲区1的扩展标识符低字节
    MCP_TXB1DLC  = 0x45, // 传输缓冲区1的数据长度码
    MCP_TXB1DATA = 0x46, // 传输缓冲区1的数据，0x46后面7个地址分别为传输缓冲区2的数据
    MCP_TXB2CTRL = 0x50, // 传输缓冲区2控制寄存器
    MCP_TXB2SIDH = 0x51, // 传输缓冲区2的标准标识符高字节
    MCP_TXB2SIDL = 0x52, // 传输缓冲区2的标准标识符低字节
    MCP_TXB2EID8 = 0x53, // 传输缓冲区2的扩展标识符高字节
    MCP_TXB2EID0 = 0x54, // 传输缓冲区2的扩展标识符低字节
    MCP_TXB2DLC  = 0x55, // 传输缓冲区2的数据长度码
    MCP_TXB2DATA = 0x56, // 传输缓冲区2的数据，0x56后面7个地址分别为传输缓冲区3的数据
    MCP_RXB0CTRL = 0x60, // 接收缓冲区0控制寄存器
    MCP_RXB0SIDH = 0x61, // 接收缓冲区0的标准标识符高字节
    MCP_RXB0SIDL = 0x62, // 接收缓冲区0的标准标识符低字节
    MCP_RXB0EID8 = 0x63, // 接收缓冲区0的扩展标识符高字节
    MCP_RXB0EID0 = 0x64, // 接收缓冲区0的扩展标识符低字节
    MCP_RXB0DLC  = 0x65, // 接收缓冲区0的数据长度码
    MCP_RXB0DATA = 0x66, // 接收缓冲区0的数据,0x66后面7个地址分别为接收缓冲区1的数据
    MCP_RXB1CTRL = 0x70, // 接收缓冲区1控制寄存器
    MCP_RXB1SIDH = 0x71, // 接收缓冲区1的标准标识符高字节
    MCP_RXB1SIDL = 0x72, // 接收缓冲区1的标准标识符低字节
    MCP_RXB1EID8 = 0x73, // 接收缓冲区1的扩展标识符高字节
    MCP_RXB1EID0 = 0x74, // 接收缓冲区1的扩展标识符低字节
    MCP_RXB1DLC  = 0x75, // 接收缓冲区1的数据长度码
    MCP_RXB1DATA = 0x76  // 接收缓冲区1的数据，0x76后面7个地址分别为接收缓冲区2的数据
};

// MCP2515 模式枚举
enum MCP2515_MODE {
    NORMAL     = 0x00,
    SLEEP      = 0x20,
    LOOPBACK   = 0x40,
    LISTENONLY = 0x60,
    CONFIG     = 0x80
};
class HXC_MCP2515:public HXC_CAN{
public:

    /**
     * @brief 构造函数
     * @return {*}
     * @author: qingmeijiupiao
     * @param {SPIClass} *_spi SPI总线
     * @param {uint8_t} _cs_pin CS引脚
     * @param {uint8_t} _int_pin INT引脚，用于中断
     */
    HXC_MCP2515(spi_device_handle_t *_spi,uint8_t _cs_pin,uint8_t _int_pin);

    /**
     * @description:  初始化CAN总线
     * @author: qingmeijiupiao
     * @param {CAN_RATE} rate CAN总线速率枚举
     */
    hxc_err_t setup(CAN_RATE rate,spi_bus_config_t& bus_config);
    /**
     * @brief 发送CAN消息
     * @author qingmeijiupiao
     * @param {HXC_CAN_message_t} msg CAN消息结构体
     * @return {hxc_err_t} ESP_OK:成功 ESP_FAIL:失败
     */
    hxc_err_t send(HXC_CAN_message_t msg) override;
    
    /**
     * @brief  发送CAN消息
     * @author  qingmeijiupiao
     * @param {HXC_CAN_message_t*} msg CAN消息结构体指针
     * @return {hxc_err_t} ESP_OK:成功 ESP_FAIL:失败
     */
    hxc_err_t send(HXC_CAN_message_t *msg) override;
    
protected:


    /**
     * @brief 读取寄存器
     * @return {uint8_t} 读取到的数据
     * @author: qingmeijiupiao
     * @param {uint8_t} address 寄存器地址
     */    
    uint8_t readRegister(uint8_t address);
    /**
     * @brief : 设置寄存器
     * @return  {*}
     * @Author : qingmeijiupiao
     * @param {uint8_t} address 寄存器地址
     * @param {uint8_t} data 数据
     */
    void setRegister(uint8_t address,uint8_t data);

    /**
     * @brief :  修改寄存器
     * @return  {*}
     * @Author : qingmeijiupiao
     * @param {uint8_t} address 寄存器地址
     * @param {uint8_t} mask 掩码,需要修改的位为1，不需要修改的位为0
     * @param {uint8_t} data 数据
     * 
     */
    void modifyRegister(uint8_t address,uint8_t mask, uint8_t data);

    /**
     * @brief   从接收缓冲区读取CAN消息
     * @return  {*}
     * @Author : qingmeijiupiao
     * @param {uint8_t} buffer_num 接收缓冲区编号 0-1
     * @param {HXC_CAN_message_t} *msg CAN消息结构体指针
     */
    void read_can_message(uint8_t buffer_num, HXC_CAN_message_t *msg);

    /**
    * @brief : 获取中断标志
    * @return  {uint8_t} 中断标志 
    * @author : qingmeijiupiao
    */
    uint8_t getInterrupts();

    /**
     * @brief :  清除中断标志
     * @return  {*}
     * @Author : qingmeijiupiao
     */
    void clearInterrupts();

    /**
     * @brief :  设置过滤器
     * @return  {*}
     * @Author : qingmeijiupiao
     * @param {uint32_t} address 过滤器地址
     * @param {uint32_t} mask 过滤器掩码
     */
    void set_filter(uint32_t address,uint32_t mask);

    /**
     * @brief :  SPI总线加锁
     * @return  {*}
     * @Author : qingmeijiupiao
     */
    void lock_spi();

    /**
     * @brief :  SPI总线解锁
     * @return  {*}
     * @Author : qingmeijiupiao
     */
    void unlock_spi();
    
    /**
     * @brief :  将CAN消息发送到TX缓冲区
     * @return  {hxc_err_t} ESP_OK:成功 ESP_FAIL:失败
     * @Author : qingmeijiupiao
     * @param {uint8_t} TX_num TX缓冲区编号 0-2
     * @param {HXC_CAN_message_t*} msg CAN消息结构体指针
     */

    hxc_err_t send_msg_to_tx_buffer(uint8_t TX_num, HXC_CAN_message_t* msg);

    /**
     * @brief : 接收中断处理函数
     * @return  {*}
     * @Author : qingmeijiupiao
     */
    void interrupt();
    spi_device_handle_t *spi;//SPI总线
    uint8_t cs_pin;//CS引脚
    uint8_t int_pin;//中断引脚
    HXC_CAN_message_t rx_buffer[2];//接收缓冲对象

    //中断包装函数
    static void Interrupt_repakage_func(void *arg);

    //spi总线信号量
    static std::map<spi_device_handle_t*,SemaphoreHandle_t> spi_semaphore;
};

#endif