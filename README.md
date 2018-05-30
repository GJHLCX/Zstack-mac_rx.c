# Zstack-mac_rx.c
#Author:GJH

Zstack 协议栈中，MAC层最重要的一个文件是mac_rx.c，里面包含了Zstack中mac层是怎么处理接收到的数据帧的。我把我自己对MAC层接收过程的中文解读加在代码的注释里了。

主要的几个函数：
//刚收到packet的中断服务程序，    第1个执行
 static void rxStartIsr(void);      
 
//处理地址的中断服务程序，        第2个执行
 static void rxAddrIsr(void);       
 
//payload数据接收的中断服务程序， 第3个执行
 static void rxPayloadIsr(void); 
 
//处理丢弃packet的中断服务程序，  
 static void rxDiscardIsr(void); 
 
//处理FCS的中断服务程序，
 static void rxFcsIsr(void);        
