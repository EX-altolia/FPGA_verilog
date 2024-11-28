# 学习记录
学习资料来源于正点原子官方网站[https://www.yuanzige.com/]
#### 名词解释

1.域（Domain）：应用程序运行环境，例如os（操作系统）和BSP；
2.BSP：板级支持包，底层硬件和应用程序沟通的桥梁

鹿小班型号：XC7Z020CLG484-1
riguke型号：XC7Z020CLG484-2


#### 官方手册笔记（ug585）

1.GPIO(p294)：对引脚进行简单观测（input）与控制（output）
Each GPIO is independently and dynamically programmed as input, output, or interrupt sensing
这里的"interrupt sensing" （中断感知）是指 GPIO 引脚可以配置为中断源，能够在特定条件下触发中断事件。
类似在线逻辑分析仪中的触发信号 
软件通过存储映射的一组寄存器来控制GPIO 


2.MIO（multiuse io）：连接ps端的外设或者静态存储器，通过配置与编程进行多路复用到PS端的引脚上   

    
3.EMIO（）：将PL端的引脚给PS端使用，达到扩展端口的效果 
   
4.寄存器组：器件引脚实际在软件中的表示，需要在正确的配置下才可读或者可写
每个寄存器部分的内容与其作用可以读文档:  
        
    1.DATA_RO：只读，反映引脚当前值;
    2.DATA：当GPIO被正确配置为输出的时候，该寄存器可以控制输出的数值;
    3.MASK_DATA_LSW: 用于屏蔽DATA的低十六位;
    4.MASK_DATA_MSW: 用于屏蔽DATA的高十六位;
    5.DIRM：用于控制该io引脚是作为输入还是输出，0则关闭输出，1则允许输出;
    6.OEN：当GPIO被正确配置为输出的时候，该寄存器用于打开或者关闭输出使能，0则关闭，1则打开;


5.MIO与EMIO都是：1.配置硬件;2.配置使能;3.操作。

6.官方exam，vitis打开版级支持包可以找到例程![image-2](https://github.com/user-attachments/assets/3b165f34-b396-48c5-a710-b9bf5763807d)





#### 一些常用示例，函数 

###### 较为陌生的变量类型或者一些数据类型
    1.XScuGic本身是一个结构体，封装了Zynq-7020中断控制器的硬件信息和相关操作函数。  
    它是Xilinx SDK中提供的一个驱动结构体，包含了与中断控制器交互的必要信息和操作接口。 
    若加上*就是指向 XScuGic 结构体的实例。在裸机开发中，通常会通过指针来引用硬件外设，以便动态管理和操作硬件资源。  
    例如：XScuGic *Intc;通常用于表示和控制中断控制器的状态。

    2.XGpioPs本身是一个结构体，定义了操作 PS GPIO 外设所需的数据和状态。 
    它包含了 GPIO 设备的相关信息，例如设备的配置、控制寄存器等。 
    故其函数接口提供了对 GPIO 外设的操作，包括配置输入/输出方向、读取输入状态、输出数据等。 
    最常进行的操作就是：    
        ①获取gpio配置信息;
        ②初始化gpio设备;
        ③设置gpio输出输入方向;
        ④使能该gpio;
        ⑤对该gpio进行读写操作.

    3.


##### GPIO相关函数以及简单示例 
````C
    //1.初始化    
    #define led0 8;//MIO或EMIO定义（注意一下MIO或者EMIO的定义方式，EMIO相对自由，MIO外设相对固定）
    XGpioPs Gpio;/* The driver instance for GPIO Device. */
    XGpioPs_Config * ConfigPtr;
    ConfigPtr = XGpioPs_LookupConfig(GPIO_DEVICE_ID);  // 根据器件id寻找器件配置信息
    XGpioPs_CfgInitialize(&Gpio, ConfigPtr, ConfigPtr->BaseAddr);

    //2.设置输入输出与读值
    #define LED0 8  /* LED button 绑定到对应的GPIO寄存器*/
    XGpioPs_SetDirectionPin(&Gpio, LED0, 1);   // LED0 设置为输出
    XGpioPs_SetOutputEnablePin(&Gpio, LED0, 1);   // LED0 输出使能
    XGpioPs_WritePin(&Gpio, LED0, 0x1);  // 点亮LED0（写入值
    key1_value = XGpioPs_ReadPin(&Gpio, KEY1);//读出数值
````    
若为AXI总线控制下的GPIO，则在手册pg144-axi-gpio中有配置顺序：
![pg144_page16](https://github.com/user-attachments/assets/c6f8cb86-9dfa-4211-a9e0-f4aa08811603)






##### 中断操作
arm处理器主要有两种处理方式：中断或者轮询 


中断相关函数以及简单示例
###### 1.GPIO中断
````C
    #define GPIO_DEVICE_ID      XPAR_XGPIOPS_0_DEVICE_ID      //PS 端 GPIO 器件 ID 13  
    #define INTC_DEVICE_ID      XPAR_SCUGIC_SINGLE_DEVICE_ID  //通用中断控制器 ID 14  
    #define GPIO_INTERRUPT_ID   XPAR_XGPIOPS_0_INTR           //PS 端 GPIO 中断 ID 

    #define KEY  11         //KEY 连接到 MIO11 
    #define LED  0          //LED 连接到 MIO0 

    XGpioPs gpio;   //PS 端 GPIO 驱动实例 
    XScuGic intc;   //通用中断控制器驱动实例 
    u32 key_press;  //KEY 按键按下的标志 
    u32 key_val;    //用于控制 LED 的键值 

//中断处理函数
static void intr_handler(void *callback_ref){ //这里的CallBackRef 是指向上层回调引用的指针 
    XGpioPs *gpio = (XGpioPs *) callback_ref; 
    //读取 KEY 按键引脚的中断状态，判断是否发生中断 
    if (XGpioPs_IntrGetStatusPin(gpio, KEY)){ 
        key_press = TRUE; 
        XGpioPs_IntrDisablePin(gpio, KEY); //屏蔽按键 KEY 中断      
    } 
}

//建立中断系统，使能 KEY 按键的下降沿中断
// GicInstancePtr 是一个指向 XScuGic 驱动实例的指针 
// gpio 是一个指向连接到中断的 GPIO 组件实例的指针 
// GpioIntrId 是 Gpio 中断 ID 
// 如果成功返回 XST_SUCCESS, 否则返回 XST_FAILURE 
int setup_interrupt_system(XScuGic *gic_ins_ptr, XGpioPs *gpio, u16 GpioIntrId){ 
    int status; 
    XScuGic_Config *IntcConfig;     //中断控制器配置信息

    //查找中断控制器配置信息并初始化中断控制器驱动 
    IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID); 
    if (NULL == IntcConfig) { return XST_FAILURE; } 
    //上面这里的检查有错就返回，无误就执行下面的初始化
    status = XScuGic_CfgInitialize(gic_ins_ptr, IntcConfig, IntcConfig->CpuBaseAddress); 
    //状态有问题就返回，没问题就继续
    if (status != XST_SUCCESS) { return XST_FAILURE; } 
    
    //设置并使能中断异常
    //这个函数支持七种异常情况，为该异常情况注册一个处理模块
    //XIL_EXCEPTION_ID_INT  这个是中断请求的标识，意味着将为gic_ins_ptr这个中断实例注册为一个中断请求的程序
    //而后将 XScuGic_InterruptHandler函数（这个函数用来处理通过
    //Zynq的GIC（General Interrupt Controller）传入的中断请求）作为中断处理程序传递给Xil_ExceptionRegisterHandler
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler) XScuGic_InterruptHandler, gic_ins_ptr); 
    //全局启用中断处理，这里可能有错，视频中的是Xil_ExceptionEnableMask(); 
    Xil_ExceptionEnable(); 

    //为中断设置中断处理函数，这里设置的就是上面定义的intr_handler中断处理函数
    status = XScuGic_Connect(gic_ins_ptr, GpioIntrId, (Xil_ExceptionHandler) intr_handler, (void *) gpio); 
    if (status != XST_SUCCESS) { return status; } 

    //使能来自于 Gpio 器件的中断 
    XScuGic_Enable(gic_ins_ptr, GpioIntrId); 
     //设置 KEY 按键的中断类型为下降沿中断 
    XGpioPs_SetIntrTypePin(gpio, KEY, XGPIOPS_IRQ_TYPE_EDGE_FALLING);
    //使能按键 KEY 中断 
    XGpioPs_IntrEnablePin(gpio, KEY);  

    return XST_SUCCESS; 
}

//非合规代码，为了突出中心，重点关注全部的一个流程等等
int main(){
    //省略部分初始化例程，用以突出中断的使用
    //创建中断处理函数，有错误的话就返回错误信息
    status = setup_interrupt_system(&intc, &gpio, GPIO_INTERRUPT_ID); 
    if (status != XST_SUCCESS) { 
        xil_printf("Setup interrupt system failed\r\n"); 
        return XST_FAILURE; 
    } 

    //中断触发时，key_press 为 TURE，延时一段时间后判断按键是否按下，是则反转 LED 
    while (1) { 
        //中断处理函数的作用就是在触发中断并且判断无误之后传回key_press值
        if (key_press) { 
            usleep(20000); 
            if (XGpioPs_ReadPin(&gpio, KEY) == 0) { 
                key_val = ~key_val; 
                XGpioPs_WritePin(&gpio, LED, key_val); 
            } 
            //在中断处理函数中，key_press返回true，屏蔽了按键中断
            //因此在这里就要将标志拉低，清空，并且重新使能中断，这样下次再按下才会开启
            key_press = FALSE; 
            XGpioPs_IntrClearPin(&gpio, KEY);      //清除按键 KEY 中断
            XGpioPs_IntrEnablePin(&gpio, KEY);     //使能按键 KEY 中断 （允许GIC接收来自key的中断）        
        } 
     } 

    return XST_SUCCESS; 
}

````
###### AXI 自定义IP核
自定义IP核一般使用AXI接口，在vitis中可被简化为对固定地址的寄存器进行读写的操作

这是顶层功能模块（未接入AXI的），在接入AXI总线后，寄存器[31]被设置为使能位（sw_ctrl），寄存器[9:0]被设置为配置值（set_freq_step）
````verilog
    breath_led(
    input          sys_clk        , //系统时钟 50MHz
    input          sys_rst_n      , //系统复位，低电平有效
	input          sw_ctrl        , //呼吸灯开关控制信号 1：亮 0:灭
    input          set_en         , //设置呼吸灯频率设置使能信号
    input   [9:0]  set_freq_step  , //设置呼吸灯频率变化步长  
    output         led              //LED灯
    );
````
这是vitis内的主函数内容，主要展示读写操作
````C
    #include "stdio.h" 
    #include "xparameters.h" 
    #include "xil_printf.h" 
    #include "breath_led_ip.h" 
    #include "xil_io.h" 
    #include "sleep.h" 
    //有些头文件是多余的，因为是完整工程截下来的，便于找不到用哪个头文件时看一下

    //这里基地址就是AXIIP核接入AXI interconnect之后的映射地址，后面的值在bsp内可以找到定义
    #define  LED_IP_BASEADDR   XPAR_BREATH_LED_IP_0_S0_AXI_BASEADDR  //LED IP 基地址   
    #define  LED_IP_REG0       BREATH_LED_IP_S0_AXI_SLV_REG0_OFFSET  //LED IP 寄存器地址 0  
    #define  LED_IP_REG1       BREATH_LED_IP_S0_AXI_SLV_REG1_OFFSET  //LED IP 寄存器地址 1
    
    //写操作
    //寄存器一共三十二位，这里的值是因为寄存器[31]被设置为使能位，寄存器[9:0]被设置为配置值
    //换为32位就是1000——0000——0000——0000——0000——0000——0000——0001
    BREATH_LED_IP_mWriteReg(LED_IP_BASEADDR,LED_IP_REG1,0x80000001); 

    //读操作
    led_state = BREATH_LED_IP_mReadReg(LED_IP_BASEADDR,LED_IP_REG0); 

````


###### UART中断
该示例较为典型，故抄录了完整代码
````C
#include "xparameters.h"		//器件参数信息
#include "xuartps.h"			//包含PS UART的函数声明
#include "xil_printf.h"			//包含print()函数
#include "xscugic.h"			//包含中断的函数声明
#include "stdio.h"				//包含printf函数的声明

#define UART_DEVICE_ID     XPAR_PS7_UART_0_DEVICE_ID    //串口设备ID
#define INTC_DEVICE_ID     XPAR_SCUGIC_SINGLE_DEVICE_ID //中断ID
#define UART_INT_IRQ_ID    XPAR_XUARTPS_0_INTR          //串口中断ID

//注意一下哪个实例对应的是什么东西
XScuGic Intc;              //中断控制器驱动程序实例
XUartPs Uart_Ps;           //串口驱动程序实例
//串口设备初始化
int uart_init(XUartPs* uart_ps)
{
	int status;//标示初始化状态
    XUartPs_Config *uart_cfg;//uart设备指针

    uart_cfg = XUartPs_LookupConfig(UART_DEVICE_ID);//搜索uart设备id与信息
    if (NULL == uart_cfg)
       return XST_FAILURE;//错误的话就返回了
    status = XUartPs_CfgInitialize(uart_ps, uart_cfg, uart_cfg->BaseAddress);//设备初始化，存储映射的指针指向基地址
    if (status != XST_SUCCESS)
       return XST_FAILURE;//错误的话就返回

    //UART设备自检
     status = XUartPs_SelfTest(uart_ps);
    if (status != XST_SUCCESS)
       return XST_FAILURE;//错误的话就返回

    //设置工作模式:正常模式
    XUartPs_SetOperMode(uart_ps, XUARTPS_OPER_MODE_NORMAL);
    //设置波特率:115200
    XUartPs_SetBaudRate(uart_ps,115200);
    //设置RxFIFO的中断触发等级
    //FIFO 阈值设置为 1，表示当 FIFO 中的一个数据字节已经被接收到（即 FIFO 队列有 1 个字节数据），就会触发相关的中断或数据处理机制。
    XUartPs_SetFifoThreshold(uart_ps, 1);

    return XST_SUCCESS;
}
//UART中断处理函数
void uart_intr_handler(void *call_back_ref)
{
	XUartPs *uart_instance_ptr = (XUartPs *) call_back_ref;
    u32 rec_data = 0 ;  //接收的数据
    u32 isr_status   ;  //中断状态标志

    //读取中断ID寄存器，判断触发的是哪种中断
    isr_status = XUartPs_ReadReg(uart_instance_ptr->Config.BaseAddress,XUARTPS_IMR_OFFSET); //这里赌的是中断掩码寄存器的内容。它表示了当前允许的中断类型。
    isr_status &= XUartPs_ReadReg(uart_instance_ptr->Config.BaseAddress,XUARTPS_ISR_OFFSET);//这一行读取的是 UART 中断状态寄存器的内容，表示哪些中断已经发生。
    //两端读取进行与操作，为1的地方就是允许且发生了的中断

    //判断中断标志位RxFIFO是否触发，XUARTPS_IXR_RXOVR 是一个常量，代表
    //接收 FIFO 溢出（Receive FIFO Overflow）中断的标志位。
    //说明这个是溢出中断模式
    if (isr_status & (u32)XUARTPS_IXR_RXOVR){
        rec_data = XUartPs_RecvByte(XPAR_PS7_UART_0_BASEADDR);//该行从 UART 接收 FIFO 中读取一个字节的数据
        //XUartPs_Recv则会接受缓冲buffer内所有的内容
        //清除中断标志
        XUartPs_WriteReg(uart_instance_ptr->Config.BaseAddress,XUARTPS_ISR_OFFSET, XUARTPS_IXR_RXOVR) ;
    }
    //改行送回一个字节，就是刚刚读出来的字节
    XUartPs_SendByte(XPAR_PS7_UART_0_BASEADDR,rec_data);
}
//串口中断初始化
//这一段基本上用的都是模板，记住大致顺序与作用，后面调用即可
//intc为实例化的一个中断控制器
int uart_intr_init(XScuGic *intc, XUartPs *uart_ps)
{
	int status;//uart状态
    //初始化中断控制器
    XScuGic_Config *intc_cfg;//中断控制器指针
    intc_cfg = XScuGic_LookupConfig(INTC_DEVICE_ID);//查找配置信息
    if (NULL == intc_cfg)
        return XST_FAILURE;//失败则返回
    status = XScuGic_CfgInitialize(intc, intc_cfg,intc_cfg->CpuBaseAddress);//初始化
    if (status != XST_SUCCESS)
        return XST_FAILURE;//失败则返回

    //设置并打开中断异常处理功能
    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
            (Xil_ExceptionHandler)XScuGic_InterruptHandler,
            (void *)intc);
    Xil_ExceptionEnable();

    //为中断设置中断处理函数，这里的中断处理函数是我们自己写的 uart_intr_handler
    //光从目前学到的中断来看，中断信号与中断函数似乎类似于QT或者JS中的
    //信号与槽函数，二者之间相对独立
    XScuGic_Connect(intc, UART_INT_IRQ_ID,
            (Xil_ExceptionHandler) uart_intr_handler,(void *) uart_ps);
    //设置UART的中断触发方式，设置的是XUARTPS_IXR_RXOVR代表的方式
    XUartPs_SetInterruptMask(uart_ps, XUARTPS_IXR_RXOVR);
    //使能GIC中的串口中断
    XScuGic_Enable(intc, UART_INT_IRQ_ID);
    return XST_SUCCESS;
}
//main函数
int main(void)
{
   int status;
   status = uart_init(&Uart_Ps);    //串口初始化
   if (status == XST_FAILURE) {
       xil_printf("Uart Initial Failed\r\n");
       return XST_FAILURE;}

   uart_intr_init(&Intc, &Uart_Ps); //串口中断初始化
   while (1);
   return status;
}

````
##### 私有定时器中断
````C
#include "xparameters.h"				//包含器件的参数信息
#include "xscutimer.h"					//定时器中断的函数声明
#include "xscugic.h"					//包含中断的函数声明
#include "xgpiops.h"					//PS端GPIO的函数声明
#include "xil_printf.h"
//注意此段定义，中断程序实例与定时器本身是不同的定义
#define TIMER_DEVICE_ID     XPAR_XSCUTIMER_0_DEVICE_ID   //定时器ID
#define INTC_DEVICE_ID      XPAR_SCUGIC_SINGLE_DEVICE_ID //中断ID
#define TIMER_IRPT_INTR     XPAR_SCUTIMER_INTR           //定时器中断ID
#define GPIO_DEVICE_ID      XPAR_XGPIOPS_0_DEVICE_ID     //宏定义 GPIO_PS ID
#define MIO_LED_0             8                            //led连接到 MIO0
#define MIO_LED_1             7

//私有定时器的时钟频率 = CPU时钟频率/2 = 333MHz
//0.2s闪烁一次,0.2*1000_000_000/(1000/333) - 1 = 3F83C3F
#define TIMER_LOAD_VALUE    0x3F83C3F                    //定时器装载值
XScuGic Intc;               //中断控制器驱动程序实例
XScuTimer Timer;            //定时器驱动程序实例
XGpioPs Gpio;               //GPIO设备的驱动程序实例，这个是老演员了

//MIO引脚初始化，此段可以作为标准化的写法，一般来说input output inout都是事先确定好的，不会再动
int mio_init(XGpioPs *mio_ptr)
{
	int status;

    XGpioPs_Config *mio_cfg_ptr;
    mio_cfg_ptr = XGpioPs_LookupConfig(GPIO_DEVICE_ID);
    if (NULL == mio_cfg_ptr)
        return XST_FAILURE;
    status = XGpioPs_CfgInitialize(mio_ptr, mio_cfg_ptr, mio_cfg_ptr->BaseAddr);
    if (status != XST_SUCCESS)
        return XST_FAILURE;

    //设置指定引脚的方向： 0 输入， 1 输出
    XGpioPs_SetDirectionPin(&Gpio, MIO_LED_0, 1);
    //使能指定引脚输出： 0 禁止输出使能， 1 使能输出
    XGpioPs_SetOutputEnablePin(&Gpio, MIO_LED_0, 1);
    //设置指定引脚的方向： 0 输入， 1 输出
    XGpioPs_SetDirectionPin(&Gpio, MIO_LED_1, 1);
    //使能指定引脚输出： 0 禁止输出使能， 1 使能输出
    XGpioPs_SetOutputEnablePin(&Gpio, MIO_LED_1, 1);
    return XST_SUCCESS;
}

//定时器中断处理程序，这一段用来给中断初始化进行连接，在中断发生后执行想要的操作，也可以放在后面，但是要事先定义
void timer_intr_handler(void *CallBackRef)
{
    //LED状态,用于控制LED灯状态翻转
    static int led_state = 0;
    XScuTimer *timer_ptr = (XScuTimer *) CallBackRef;//实参传进来定时器的句柄，对该句柄进行操作可以清空定时器等等
    if(led_state == 0)
        led_state = 1;
    else
        led_state = 0;
    //向指定引脚写入数据： 0 或 1
    XGpioPs_WritePin(&Gpio, MIO_LED_0,~led_state);
    XGpioPs_WritePin(&Gpio, MIO_LED_1,led_state);
    //清除定时器中断标志，注意操作的是什么对象，不要用多了*或者&
    XScuTimer_ClearInterruptStatus(timer_ptr);
}
//定时器中断初始化
void timer_intr_init(XScuGic *intc_ptr,XScuTimer *timer_ptr)
{
	//初始化中断控制器，三板斧
    XScuGic_Config *intc_cfg_ptr;
    intc_cfg_ptr = XScuGic_LookupConfig(INTC_DEVICE_ID);
    XScuGic_CfgInitialize(intc_ptr, intc_cfg_ptr,intc_cfg_ptr->CpuBaseAddress);
    //设置并打开中断异常处理功能
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,(Xil_ExceptionHandler)XScuGic_InterruptHandler, intc_ptr);
    Xil_ExceptionEnable();

    //设置定时器中断，注意连接到正确的自定义的中断处理函数上
    XScuGic_Connect(intc_ptr, TIMER_IRPT_INTR,(Xil_ExceptionHandler)timer_intr_handler, (void *)timer_ptr);
    XScuGic_Enable(intc_ptr, TIMER_IRPT_INTR); //使能GIC中的定时器中断
    //
    XScuTimer_EnableInterrupt(timer_ptr);      //使能定时器中断
}

//main函数
int main()
{
	int status;

    mio_init(&Gpio);                 //MIO引脚初始化
    
    status = timer_init(&Timer);     //定时器初始化
    if (status != XST_SUCCESS) {
        xil_printf("Timer Initial Failed\r\n");
        return XST_FAILURE;
    }
    //注意传参的格式
    timer_intr_init(&Intc,&Timer);   //定时器中断初始化
    
    XScuTimer_Start(&Timer);         //启动定时器

    while(1);//中断与定时器不会占用，这样子主函数只需要在这循环就可以了
    return 0;
}
````

##### flash读写功能（PS）
本段为轮询模式下的flash读写，每一个非报告命令写入（需要确认是否完成的命令）都会通过不断读取状态寄存器以向flash确认操作是否完成
````C
#include "xparameters.h"    /* SDK generated parameters */
#include "xqspips.h"        /* QSPI device driver */
#include "xil_printf.h"

#define QSPI_DEVICE_ID      XPAR_XQSPIPS_0_DEVICE_ID

//发送到Flash器件的指令
#define WRITE_STATUS_CMD    0x01
#define WRITE_CMD           0x02
#define READ_CMD            0x03
#define WRITE_DISABLE_CMD   0x04
#define READ_STATUS_CMD     0x05
#define WRITE_ENABLE_CMD    0x06
#define FAST_READ_CMD       0x0B
#define DUAL_READ_CMD       0x3B
#define QUAD_READ_CMD       0x6B
#define BULK_ERASE_CMD      0xC7
#define SEC_ERASE_CMD       0xD8
#define READ_ID             0x9F

//Flash BUFFER中各数据的偏移量
//命令部分
#define COMMAND_OFFSET      0 // Flash instruction
//地址部分
#define ADDRESS_1_OFFSET    1 // MSB byte of address to read or write
#define ADDRESS_2_OFFSET    2 // Middle byte of address to read or write
#define ADDRESS_3_OFFSET    3 // LSB byte of address to read or write
//数据部分
#define DATA_OFFSET         4 // Start of Data for Read/Write
//虚拟字节，用于确认握手（类似握手信号）
#define DUMMY_OFFSET        4 // Dummy byte offset for reads
//各个数据长度或者包长定义等等
#define DUMMY_SIZE          1 // Number of dummy bytes for reads
#define RD_ID_SIZE          4 // Read ID command + 3 bytes ID response
#define BULK_ERASE_SIZE     1 // Bulk Erase command size
#define SEC_ERASE_SIZE      4 // Sector Erase command + Sector address
#define OVERHEAD_SIZE       4 // control information: command and address
#define SECTOR_SIZE         0x10000
#define NUM_SECTORS         0x100
#define NUM_PAGES           0x10000
#define PAGE_SIZE           256

//要被写的页数
#define PAGE_COUNT      16

#define TEST_ADDRESS    0x00055000//写偏移地址
#define UNIQUE_VALUE    0x05//识别id
//计算的最大数据
#define MAX_DATA        (PAGE_COUNT * PAGE_SIZE)

//各个函数定义
void FlashErase(XQspiPs *QspiPtr, u32 Address, u32 ByteCount);//擦除
void FlashWrite(XQspiPs *QspiPtr, u32 Address, u32 ByteCount, u8 Command);//写
void FlashRead(XQspiPs *QspiPtr, u32 Address, u32 ByteCount, u8 Command);//读
int  FlashReadID(void);//读ID与配置
void FlashQuadEnable(XQspiPs *QspiPtr);//全通道读写功能启用
int  QspiFlashPolledExample(XQspiPs *QspiInstancePtr, u16 QspiDeviceId);//示例函数，其实这个相当于主函数了，只是如果有哪个函数出问题了会跳出来给主函数报错
//qspi flash读写控制器实例
static XQspiPs QspiInstance;

//测试数据
int Test = 5;
//读写缓存
u8 ReadBuffer[MAX_DATA + DATA_OFFSET + DUMMY_SIZE];
u8 WriteBuffer[PAGE_SIZE + DATA_OFFSET];


int main(void)
{
    int Status;//初始化状态，用于标识是否初始化成功
    xil_printf("QSPI Flash Polled Example Test \r\n");
    //直接就是跑例程，重点在后面的函数
    Status = QspiFlashPolledExample(&QspiInstance,QSPI_DEVICE_ID);
    if (Status != XST_SUCCESS) {
        xil_printf("QSPI Flash Polled Example Test Failed\r\n");
        return XST_FAILURE;}
    xil_printf("Successfully ran QSPI Flash Polled Example Test\r\n");
 return XST_SUCCESS;
}
//读写示例
int QspiFlashPolledExample(XQspiPs *QspiInstancePtr, u16 QspiDeviceId)
{
	//int Status;
    u8 *BufferPtr;
    u8 UniqueValue;
    int Count;
    int Page;
    XQspiPs_Config *QspiConfig;

    //初始化QSPI驱动
    QspiConfig = XQspiPs_LookupConfig(QspiDeviceId);//获取配置，下一句初始化
    XQspiPs_CfgInitialize(QspiInstancePtr, QspiConfig, QspiConfig->BaseAddress);
    //初始化读写BUFFER
    //这里将前面的偏移量与数据等等进行计算填充写缓存
    for (UniqueValue = UNIQUE_VALUE, Count = 0; Count < PAGE_SIZE;
         Count++, UniqueValue++) {
        WriteBuffer[DATA_OFFSET + Count] = (u8)(UniqueValue + Test);
    }
    //这里将全部读缓存全存零
    memset(ReadBuffer, 0x00, sizeof(ReadBuffer));

    //通过该函数可以设置qspi读写控制器模式设置为手动启动和手动片选模式
    XQspiPs_SetOptions(QspiInstancePtr, XQSPIPS_MANUAL_START_OPTION |
            XQSPIPS_FORCE_SSELECT_OPTION |
            XQSPIPS_HOLD_B_DRIVE_OPTION);
    //设置QSPI时钟的分频系数，这里八分频
    XQspiPs_SetClkPrescaler(QspiInstancePtr, XQSPIPS_CLK_PRESCALE_8);
    //片选信号置为有效
    XQspiPs_SetSlaveSelect(QspiInstancePtr);
    //读Flash ID
    FlashReadID();
    //使能Flash Quad模式，即全通道都启用
    FlashQuadEnable(QspiInstancePtr);
    //擦除Flash
    FlashErase(QspiInstancePtr, TEST_ADDRESS, MAX_DATA);
    //向Flash中写入数据
    for (Page = 0; Page < PAGE_COUNT; Page++) {
       FlashWrite(QspiInstancePtr, (Page * PAGE_SIZE) + TEST_ADDRESS,
              PAGE_SIZE, WRITE_CMD);
    }
    //使用QUAD模式从Flash中读出数据
    FlashRead(QspiInstancePtr, TEST_ADDRESS, MAX_DATA, QUAD_READ_CMD);

    //通过遍历对比写入Flash与从Flash中读出的数据
    BufferPtr = &ReadBuffer[DATA_OFFSET + DUMMY_SIZE];
    for (UniqueValue = UNIQUE_VALUE, Count = 0; Count < MAX_DATA;Count++, UniqueValue++) {
        if (BufferPtr[Count] != (u8)(UniqueValue + Test)) {
             return XST_FAILURE;}}

  return XST_SUCCESS;
}
//写函数，这里是直接调的xilinx官方，仅添加注释
void FlashWrite(XQspiPs *QspiPtr, u32 Address, u32 ByteCount, u8 Command)
{
	u8 WriteEnableCmd = { WRITE_ENABLE_CMD };//读使能指令存入
	u8 ReadStatusCmd[] = { READ_STATUS_CMD, 0 };  //指令要求两字节，所以多接一个零
	u8 FlashStatus[2];

	//发送读使能之后flash才可以被写，而读使能需要实现单独发一个
	XQspiPs_PolledTransfer(QspiPtr, &WriteEnableCmd, NULL,sizeof(WriteEnableCmd));
    //通过前面设置的偏移地址，在写缓存内规划好要写入的数据的存储位置，并且装入命令，这里的命令一般会是读指令
	WriteBuffer[COMMAND_OFFSET]   = Command;
	WriteBuffer[ADDRESS_1_OFFSET] = (u8)((Address & 0xFF0000) >> 16);
	WriteBuffer[ADDRESS_2_OFFSET] = (u8)((Address & 0xFF00) >> 8);
	WriteBuffer[ADDRESS_3_OFFSET] = (u8)(Address & 0xFF);

    //将写缓存内的内容写入，指令，地址，数据都一并发送，长度通过常量进行计算
	XQspiPs_PolledTransfer(QspiPtr, WriteBuffer, NULL,ByteCount + OVERHEAD_SIZE);

	//这里的死循环通过不断读取flash的状态以确认是否写完成，写完成之后才会退出循环
	while (1) {
		//这里在轮询flash的状态寄存器，具体操作为：写入读指令，将寄存器内内容取出存储
		XQspiPs_PolledTransfer(QspiPtr, ReadStatusCmd, FlashStatus,
					sizeof(ReadStatusCmd));

		//如果取出来的是写结束，就退出，反之一直卡在这（卡太久了就要考虑是不是哪里步骤错了
		if ((FlashStatus[1] & 0x01) == 0) {break;}
	}
}
//读函数
void FlashRead(XQspiPs *QspiPtr, u32 Address, u32 ByteCount, u8 Command)
{
	//依旧是先获取地址，将指令存储到写缓存中（这里一般传入的就是读指令了
	WriteBuffer[COMMAND_OFFSET]   = Command;
	WriteBuffer[ADDRESS_1_OFFSET] = (u8)((Address & 0xFF0000) >> 16);
	WriteBuffer[ADDRESS_2_OFFSET] = (u8)((Address & 0xFF00) >> 8);
	WriteBuffer[ADDRESS_3_OFFSET] = (u8)(Address & 0xFF);
    //如果写的是读指令的一种，就将计数器增加，以方便下次的接连的读取
	if ((Command == FAST_READ_CMD) || (Command == DUAL_READ_CMD) ||(Command == QUAD_READ_CMD)){
		ByteCount += DUMMY_SIZE;}
	//这里就正式写入了读指令到寄存器中，读数据位置根据读次数与溢出地址（假如溢出了）去计算，读出来后存入读缓存
	XQspiPs_PolledTransfer(QspiPtr, WriteBuffer, ReadBuffer,ByteCount + OVERHEAD_SIZE);
}
//擦除函数
void FlashErase(XQspiPs *QspiPtr, u32 Address, u32 ByteCount)
{   
	u8 WriteEnableCmd = { WRITE_ENABLE_CMD };
	u8 ReadStatusCmd[] = { READ_STATUS_CMD, 0 };  /* must send 2 bytes */
	u8 FlashStatus[2];
	int Sector;
    //如果擦除全flash内容，就会调用下面的这段批量擦除指令（有的指令是擦除一小块，有的一擦一个区
	if (ByteCount == (NUM_SECTORS * SECTOR_SIZE)) {
		//先发一次写使能，使得擦除指令可以被写入
		XQspiPs_PolledTransfer(QspiPtr, &WriteEnableCmd, NULL,sizeof(WriteEnableCmd));
		//写缓存内装入批量擦除指令
		WriteBuffer[COMMAND_OFFSET]   = BULK_ERASE_CMD;
        //向寄存器写入批量擦除指令，也就不需要返回什么数据了
		XQspiPs_PolledTransfer(QspiPtr, WriteBuffer, NULL,BULK_ERASE_SIZE);
        //这段循环和前面一样，轮询寄存器，在确认擦除完毕之后才会退出循环
		while (1) {
			XQspiPs_PolledTransfer(QspiPtr, ReadStatusCmd,
						FlashStatus,
						sizeof(ReadStatusCmd));
			if ((FlashStatus[1] & 0x01) == 0) {break;}
		}
		return;
	}

	//如果擦除部分flash内容，就会调用下面的这段部分擦除指令
    //这一段和上面的基本完全一致，除了要注意擦除完毕后的地址递增，因为擦过一次就不能重复了，会自动擦下面的内容
	for (Sector = 0; Sector < ((ByteCount / SECTOR_SIZE) + 1); Sector++) {
		XQspiPs_PolledTransfer(QspiPtr, &WriteEnableCmd, NULL,sizeof(WriteEnableCmd));
		WriteBuffer[COMMAND_OFFSET]   = SEC_ERASE_CMD;
		WriteBuffer[ADDRESS_1_OFFSET] = (u8)(Address >> 16);
		WriteBuffer[ADDRESS_2_OFFSET] = (u8)(Address >> 8);
		WriteBuffer[ADDRESS_3_OFFSET] = (u8)(Address & 0xFF);

		XQspiPs_PolledTransfer(QspiPtr, WriteBuffer, NULL,SEC_ERASE_SIZE);

		while (1) {
			XQspiPs_PolledTransfer(QspiPtr, ReadStatusCmd,
						FlashStatus,
						sizeof(ReadStatusCmd));
			if ((FlashStatus[1] & 0x01) == 0) {break;}
		}
		Address += SECTOR_SIZE;
	}
}
//读flashID函数
int FlashReadID(void)
{
	int Status;
	//读ID只有一个指令，所以不用传参，是通过全局定义获取的
    //这里向写缓存内存入即将要写的命令
	WriteBuffer[COMMAND_OFFSET]   = READ_ID;
	WriteBuffer[ADDRESS_1_OFFSET] = 0x23;		/* 3 dummy bytes */
	WriteBuffer[ADDRESS_2_OFFSET] = 0x08;
	WriteBuffer[ADDRESS_3_OFFSET] = 0x09;
    //向状态寄存器写入命令，并且接收返回的状态
	Status = XQspiPs_PolledTransfer(&QspiInstance, WriteBuffer, ReadBuffer,RD_ID_SIZE);
	if (Status != XST_SUCCESS) {return XST_FAILURE;}
    //打印返回的状态
	xil_printf("FlashID=0x%x 0x%x 0x%x\n\r", ReadBuffer[1], ReadBuffer[2],ReadBuffer[3]);
	return XST_SUCCESS;
}
//全通道读写功能启用函数
void FlashQuadEnable(XQspiPs *QspiPtr)
{//这段大概就是把启动全通道读写的命令存进去然后一次写入就是了，基本可以照抄
	u8 WriteEnableCmd = {WRITE_ENABLE_CMD};
	u8 ReadStatusCmd[] = {READ_STATUS_CMD, 0};
	u8 QuadEnableCmd[] = {WRITE_STATUS_CMD, 0};
	u8 FlashStatus[2];
	if (ReadBuffer[1] == 0x9D) {
		XQspiPs_PolledTransfer(QspiPtr, ReadStatusCmd,FlashStatus,sizeof(ReadStatusCmd));
		QuadEnableCmd[1] = FlashStatus[1] | 1 << 6;
		XQspiPs_PolledTransfer(QspiPtr, &WriteEnableCmd, NULL,sizeof(WriteEnableCmd));
		XQspiPs_PolledTransfer(QspiPtr, QuadEnableCmd, NULL,sizeof(QuadEnableCmd));}
}

````

##### SD卡相关功能
SD卡可以直接通过SDIO或者SPI模式进行操作，但是读写的二进制数据并不能直接被电脑识别，需要通过建立文件系统并且写入正确的文件头等以进行有效数据的读写等等  
而这里用的就是fatfs这一个开源的文件管理系统，其中xilinx也在板级支持包内提供了该库，只需要简单的勾选即可使用fatfs的一些函数
````C

````


#### 辅助功能

##### 1.IP核（AXI）封装
在IP核封装中，最常见的就是封装为AXI接口的ip，泛用性更好 

根据文档，到达IP核编辑界面：
![uip_edit](https://github.com/user-attachments/assets/be6bb7cb-79ef-4683-a86b-df0aae383120)

在编辑界面中文件结构如下：
![ip_edit_build](https://github.com/user-attachments/assets/cbb6c6a4-77e0-4397-8fd9-2bd117994b56)


在AXI_IP核编辑界面中，一般可以简化为如下三个层级，这里以本次界面中的结构示意：

    |breath_led_ip_v1_0：这个是整个IP核例化的模块，在外界便只可看见AXI从机接口和别的一些引出管脚；  
        |breath_led_ip_v1_0_S00_AXI：这个是IP核功能与IP核AXI接口的接驳处，一般是将IP核实现功能的顶层模块例化到该文件下，然后在该文件内编写功能模块与AXI总线的接驳时序或者连接接驳的端口等等；
            |breath_led:这个就是实现功能的模块。

所以一般来说，将一个已有的功能模块封装入AXI自定义ip，需要如下几步：  
    1.将写好的ip例化入breath_led_ip_v1_0_S00_AXI内，并且编写与AXI从机时序的接口时序，并且在预留的user端口处引出余下要连接到外界的端口；
    2.在breath_led_ip_v1_0中添加在breath_led_ip_v1_0_S00_AXI中新加入的端口与参数定义。

vivado预留的用户端口与逻辑的位置一般在参数与端口定义的开头，或者全部代码的末尾。
![image](https://github.com/user-attachments/assets/3409fe65-2023-42ab-91dd-fe7a2cc99335)

![image-1](https://github.com/user-attachments/assets/e86c9468-5697-466d-872d-a07365265b94)



##### xadc
用于读取内部电压与温度，可以在高温时预警
````c
#include "xadcps.h"				//PS端XADC函数的声明

#define XADC_DEVICE_ID   XPAR_XADCPS_0_DEVICE_ID //PS XADC 器件ID
static  XAdcPs           xadc_inst;              //XADC 驱动实例
float temp=0;                 	//温度
//初始化XADC驱动
ConfigPtr = XAdcPs_LookupConfig(XADC_DEVICE_ID);
XAdcPs_CfgInitialize(&xadc_inst, ConfigPtr, ConfigPtr->BaseAddress);
//设置XADC操作模式为“默认安全模式”
XAdcPs_SetSequencerMode(&xadc_inst, XADCPS_SEQ_MODE_SAFE);
//获取原始温度传感器数据
temp_rawdata = XAdcPs_GetAdcData(&xadc_inst, XADCPS_CH_TEMP);
//转换成温度信息
temp = XAdcPs_RawToTemperature(temp_rawdata);
printf("Raw Temp    %lu, Real Temp    %fC \n", temp_rawdata,     temp);
````
打印出来就是：Raw Temp    41278, Real Temp    44.281250C 
