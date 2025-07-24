#include "HXC_MCP2515.hpp"
std::map<spi_device_handle_t*,SemaphoreHandle_t> HXC_MCP2515::spi_semaphore;

void HXC_MCP2515::Interrupt_repakage_func(void *arg) {
    HXC_MCP2515 *can = (HXC_MCP2515*)arg;
    while (1) {
        if(!digitalRead(can->int_pin)) {
            can->interrupt();
        } else {
            #if USE_NONE_DELAY_RECEIVE == 1
            // 主动让出CPU使用权方式,接收频率高时会导致CPU占用率过高
            vPortYield();
            #else
            // 延时方式接收,总线上超过2000HZ会丢包
            vTaskDelay(1);
            #endif
        }
    }
}
HXC_MCP2515::HXC_MCP2515(spi_device_handle_t *_spi,uint8_t _cs_pin,uint8_t _int_pin){
    spi=_spi;
    cs_pin=_cs_pin;
    int_pin=_int_pin;
    
}

//spi互斥锁上锁
void HXC_MCP2515::lock_spi(){
    xSemaphoreTake(spi_semaphore[spi],portMAX_DELAY);
}

//解锁spi总线互斥锁
void HXC_MCP2515::unlock_spi(){
    xSemaphoreGive(spi_semaphore[spi]);
}

hxc_err_t HXC_MCP2515::setup(CAN_RATE rate,spi_bus_config_t& bus_config){
    pinMode(int_pin,INPUT);
    this->can_rate=rate;
    if(spi_semaphore.find(spi)==spi_semaphore.end()){
        spi_semaphore[spi]=xSemaphoreCreateMutex();//创建信号量
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST,&bus_config,SPI_DMA_CH_AUTO));//初始化SPI总线
        unlock_spi();
    }
    // 设备配置 - 注意成员顺序必须与结构体定义一致
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 1000000,
        .input_delay_ns = 0,
        .spics_io_num = cs_pin,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    // 添加设备
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, spi));
    //加锁
    lock_spi();

    // 配置模式
    MCP2515_MODE mode=CONFIG;
    modifyRegister(MCP_CANCTRL,0xE0,mode);
    // 等待配置完成
    for (int i = 0; i < 10; i++) {
        uint8_t newmode = readRegister(MCP_CANSTAT);
        newmode &= 0xE0;
        if (newmode == mode) {
            break;
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    // 启用接收缓冲区
    setRegister(MCP_RXB0CTRL,0b01100110);  // 启用接收缓冲区 0
    
    setRegister(MCP_RXB1CTRL, 0b01100000);  // 启用接收缓冲区 1

    // 启用接收中断
    modifyRegister(MCP_CANINTE, 0x03, 0x03);  // 使能接收缓冲区 0,1 中断


    uint8_t cfg[3]={0x00,0x00,0x00};
    // 配置速率
    switch (rate) {
        case CAN_RATE_1MBIT:
            cfg[0]=0x00;
            cfg[1]=0x80;
            cfg[2]=0x80;
            break;
        case CAN_RATE_800KBIT:
            break;
        case CAN_RATE_500KBIT:
            cfg[0]=0x00;
            cfg[1]=0x90;
            cfg[2]=0x82;
            break;
        case CAN_RATE_250KBIT:
            cfg[0]=0x00;
            cfg[1]=0xB1;
            cfg[2]=0x85;
            break;
        case CAN_RATE_125KBIT:
            cfg[0]=0x01;
            cfg[1]=0xB1;
            cfg[2]=0x85;
            break;
        case CAN_RATE_100KBIT:
            cfg[0]=0x01;
            cfg[1]=0xB4;
            cfg[2]=0x86;
            break;
        default:
            cfg[0]=0x00;
            cfg[1]=0x80;
            cfg[2]=0x80;
            break;
    }
    setRegister(MCP_CNF1, cfg[0]);
    setRegister(MCP_CNF2, cfg[1]);
    setRegister(MCP_CNF3, cfg[2]);

    
    mode=NORMAL;
    // 切换回普通模式
    modifyRegister(MCP_CANCTRL,0xE0,mode);
    
    
    // 解锁
    unlock_spi();

    // 注意这里优先级必须是最低否则在非延时接收时会喂不了看门狗导致重启
    xTaskCreate(Interrupt_repakage_func,"CAN_INT",1024*2,this,5,NULL);//创建can接收任务
    is_setup=true;//设置已初始化标志位
    return ESP_OK;
}
uint8_t HXC_MCP2515::readRegister(uint8_t address){
    spi_transaction_t t={};
    // memset(&t, 0, sizeof(t));
    
    uint8_t tx_data[3] = {0x03, address, 0x00};
    uint8_t rx_data[3] = {0};
    
    t.length = 24; // 3 bytes * 8 bits
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;
    
    spi_device_transmit(*spi, &t);
    
    return rx_data[2];
}

void HXC_MCP2515::setRegister(uint8_t address,uint8_t data){
    spi_transaction_t t={};
    // memset(&t, 0, sizeof(t));
    
    uint8_t tx_data[3] = {0x02, address, data};
    
    t.length = 24; // 3 bytes * 8 bits
    t.tx_buffer = tx_data;
    
    spi_device_transmit(*spi, &t);
}

void HXC_MCP2515::modifyRegister(uint8_t address,uint8_t mask, uint8_t data){
    spi_transaction_t t={};
    // memset(&t, 0, sizeof(t));    
    uint8_t tx_data[4] = {0x05, address, mask, data};
    
    t.length = 32; // 4 bytes * 8 bits
    t.tx_buffer = tx_data;
    
    spi_device_transmit(*spi, &t);
}

hxc_err_t HXC_MCP2515::send(HXC_CAN_message_t* msg){

    lock_spi();

    uint8_t tx_ctrl_flag=0;
    tx_ctrl_flag = readRegister(MCP_TXB0CTRL);
    if((tx_ctrl_flag&0x08)==0){
        send_msg_to_tx_buffer(0,msg);
        unlock_spi();
        return ESP_OK;
    }
    tx_ctrl_flag = readRegister(MCP_TXB1CTRL);
    if((tx_ctrl_flag&0x08)==0){
        send_msg_to_tx_buffer(1,msg);
        unlock_spi();
        return ESP_OK;
    }
    tx_ctrl_flag = readRegister(MCP_TXB2CTRL);
    if((tx_ctrl_flag&0x08)==0){
        send_msg_to_tx_buffer(2,msg);
        unlock_spi();
        return ESP_OK;
    }
    unlock_spi();
    return ESP_FAIL;
    
    
}
hxc_err_t HXC_MCP2515::send(HXC_CAN_message_t msg){
    return send(&msg);
}

hxc_err_t HXC_MCP2515::send_msg_to_tx_buffer(uint8_t TX_num, HXC_CAN_message_t* msg) {
    
    uint8_t TXBnCTRL=0x30+(TX_num<<8);
    uint8_t TXRTSCTRL=0x0D;
    uint8_t TXBnSIDH=0x31+(TX_num<<8);
    uint8_t TXBnSIDL=0x32+(TX_num<<8);
    uint8_t TXBnEID8=0x33+(TX_num<<8);
    uint8_t TXBnEID0=0x34+(TX_num<<8);
    uint8_t TXBnDLC=0x35+(TX_num<<8);
    uint8_t TXBnD0=0x36+(TX_num<<8);
    uint8_t temp=0;
    
    

    if(!msg->extd){//标准帧
        temp=(msg->identifier>>3)&0xFF;
        setRegister(TXBnSIDH,temp);
        temp=(msg->identifier<<5)&0xFF;
        setRegister(TXBnSIDL,temp);
    }else{//扩展帧
        temp=(msg->identifier)&0xFF;
        setRegister(TXBnEID0,temp);
        temp=(msg->identifier>>8)&0xFF;
        setRegister(TXBnEID8,temp);
        temp=(msg->identifier>>16)&0x3;
        temp|=(msg->identifier>>(13))&0xE0;
        temp|=0x8;//设置为扩展帧
        setRegister(TXBnSIDL,temp);
        temp=(msg->identifier>>21)&0xFF;
        setRegister(TXBnSIDH,temp);
    }
    temp=msg->data_length_code&0x0F;//数据长度
    temp|=msg->rtr?0x40:0x00;//远程帧
    setRegister(TXBnDLC,temp);
    for(uint8_t i=0;i<msg->data_length_code;i++){
        setRegister(TXBnD0+i,msg->data[i]);//数据
    }

    // 发送消息,第三位控制发送
    modifyRegister(TXBnCTRL, (1<<3), (1<<3));

    

    return ESP_OK;
}

void HXC_MCP2515::set_filter(uint32_t address,uint32_t mask){
    lock_spi();
    
    bool is_std=address>0x7FF?false:true;//判断是否标准帧
    // 进入配置模式
    modifyRegister(MCP_CANCTRL, 0xE0, CONFIG);
    // 等待进入配置模式
    while ((readRegister(MCP_CANSTAT) & 0xE0) != CONFIG) {
        ESP_LOGW("HXC_MCP2515", "Waiting for MCP2515 to enter configuration mode...");
        vTaskDelay(1 / portTICK_RATE_MS);
    }
    if(is_std){//标准帧
    setRegister(MCP_RXM0SIDL,(mask<<5)&0xFF);
    setRegister(MCP_RXM0SIDH,(mask>>3)&0xFF);

    setRegister(MCP_RXF0SIDL,(address<<5)&0xFF);
    setRegister(MCP_RXF0SIDH,(address>>3)&0xFF);

    }else{//扩展帧
        modifyRegister(MCP_RXM0SIDL, 0xFF,mask & 0xFF);
        modifyRegister(MCP_RXM0SIDH, 0xFF, (mask >> 8) & 0xFF);
        modifyRegister(MCP_RXM0EID0, 0xFF, (mask >> 16) & 0xFF);
        modifyRegister(MCP_RXM0EID8, 0xFF, (mask >> 24) & 0xFF);

        modifyRegister(MCP_RXF0SIDL, 0xFF, address & 0xFF);
        modifyRegister(MCP_RXF0SIDH, 0xFF, (address >> 8) & 0xFF);
        modifyRegister(MCP_RXF0EID0, 0xFF, (address >> 16) & 0xFF);
        modifyRegister(MCP_RXF0EID8, 0xFF, (address >> 24) & 0xFF);
    }


    // 配置 RXB0 控制寄存器（启用滤波器）
    setRegister(MCP_RXB0CTRL, 0b00001100);
    setRegister(MCP_RXB1CTRL, 0b00001000);
    // 返回正常模式
    modifyRegister(MCP_CANCTRL, 0xE0, NORMAL);
    // 等待进入正常模式
    while ((readRegister(MCP_CANSTAT) & 0xE0) != NORMAL) {
        ESP_LOGW("HXC_MCP2515", "Waiting for MCP2515 to enter normal mode...");
        vTaskDelay(1 / portTICK_RATE_MS);
    }

    unlock_spi();
}

uint8_t HXC_MCP2515::getInterrupts(){
    /* 
    * @bit7 MERRF：消息错误中断标志位
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    * 
    * @bit6 WAKIF：唤醒中断标志位
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    * 
    * @bit5 ERRIF：错误中断标志位（EFLG寄存器中多个源）
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    * 
    * @bit4 TX2IF：发送缓冲区2空中断标志位
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    * 
    * @bit3 TX1IF：发送缓冲区1空中断标志位
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    * 
    * @bit2 TX0IF：发送缓冲区0空中断标志位
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    * 
    * @bit1 RX1IF：接收缓冲区1满中断标志位
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    * 
    * @bit0 RX0IF：接收缓冲区0满中断标志位
    * 1 = 中断待处理（必须由MCU清除以重置中断条件）
    * 0 = 无中断待处理
    */
    return readRegister(MCP_CANINTF);
}

void HXC_MCP2515::clearInterrupts(){
    setRegister(MCP_CANINTF, 0);
}

void HXC_MCP2515::read_can_message(uint8_t buffer_num, HXC_CAN_message_t *msg){
    auto clear_rxbuffer=[](HXC_CAN_message_t * buffer){
        buffer->extd=false;
        buffer->identifier=0;
        buffer->rtr=false;
        buffer->data_length_code=0;
        memset(buffer->data,0,8);
    };
    if (buffer_num == 0) {  // 如果接收缓冲区 0 中断被触发
        clear_rxbuffer(msg);
        uint8_t Register_buffer[]={0x00,0x00};
        Register_buffer[0]=readRegister(MCP_RXB0SIDH);
        Register_buffer[1]=readRegister(MCP_RXB0SIDL);
        msg->extd=Register_buffer[1]&0x8;
        if(msg->extd){
            msg->identifier=readRegister(MCP_RXB0EID0);
            msg->identifier|=readRegister(MCP_RXB0EID8)<<8;
            msg->identifier|=((Register_buffer[1])&0x3)<<16;
            msg->identifier|=((Register_buffer[1]&0xE0)>>5)<<18;
            msg->identifier|=Register_buffer[0]<<21;
        }else{
            msg->identifier=Register_buffer[0]<<3;
            msg->identifier|=(Register_buffer[1]&0xE0)>>5;
        }
        uint8_t dlc=readRegister(MCP_RXB0DLC);
        msg->rtr=dlc&0x20;
        msg->data_length_code=dlc&0x0F;
        for(int i=0;i<msg->data_length_code;i++){
            msg->data[i]=readRegister(MCP_RXB0DATA+i);
        }
        return;
    }
    if (buffer_num == 1) {
        clear_rxbuffer(&this->rx_buffer[1]);
        uint8_t Register_buffer[]={0x00,0x00};
        Register_buffer[0]=readRegister(MCP_RXB1SIDH);
        Register_buffer[1]=readRegister(MCP_RXB1SIDL);
        msg->extd=Register_buffer[1]&0x8;
        if(msg->extd){
            msg->identifier=readRegister(MCP_RXB1EID0);
            msg->identifier|=readRegister(MCP_RXB1EID8)<<8;
            msg->identifier|=((Register_buffer[1])&0x3)<<17;
            msg->identifier|=((Register_buffer[1]&0xE0)>>5)<<19;
            msg->identifier|=Register_buffer[0]<<22;
        }else{
            msg->identifier=Register_buffer[0]<<3;
            msg->identifier|=(Register_buffer[1]&0xE0)>>5;
        }
        uint8_t dlc=readRegister(MCP_RXB1DLC);
        msg->rtr=dlc&0x20;
        msg->data_length_code=dlc&0x0F;
        for(int i=0;i<msg->data_length_code;i++){
            msg->data[i]=readRegister(MCP_RXB1DATA+i);
        }
        return;
    }
}


void HXC_MCP2515::interrupt(){
    lock_spi();

    uint8_t interruptFlags = getInterrupts();  // 检查中断标志
    if (interruptFlags & 0x01) {  // 如果接收缓冲区 0 中断被触发
        // 读取接收数据
        read_can_message(0,&this->rx_buffer[0]);

        // 
        if(func_map.find(this->rx_buffer[0].identifier)!=func_map.end()){
            func_map[this->rx_buffer[0].identifier](&this->rx_buffer[0]);

        }
    }
    if (interruptFlags & 0x02) {  // 如果接收缓冲区 1 中断被触发
        // 读取接收数据
        read_can_message(1,&this->rx_buffer[1]);

        //
        if(func_map.find(this->rx_buffer[1].identifier)!=func_map.end()){
            func_map[this->rx_buffer[1].identifier](&this->rx_buffer[1]);
        }
    }
    
    // 清除中断标志s
   clearInterrupts();

   unlock_spi();
}