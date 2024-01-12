#define LED PC13                    //调试用的LED
 
//以下为左前方电机引脚定义
#define LF_Motor_IN1 PA5            //LF电机使能引脚1
#define LF_Motor_IN2 PA4            //LF电机使能引脚2
#define LF_PWM PA0                  //LF电机调速引脚
#define LF_ENCODER_A PB6            //LF编码器A相引脚
#define LF_ENCODER_B PB7            //LF编码器B相引脚
 
//以下为右前方电机引脚定义
#define RF_Motor_IN1 PB0            //RF电机使能引脚1
#define RF_Motor_IN2 PB1            //RF电机使能引脚2
#define RF_PWM PA2                  //RF电机调速引脚
#define RF_ENCODER_A PB13           //RF编码器A相引脚
#define RF_ENCODER_B PB12           //RF编码器B相引脚
 
//以下为左后方电机引脚定义
#define LR_Motor_IN1 PA7            //LR电机使能引脚1
#define LR_Motor_IN2 PA6            //LR电机使能引脚2
#define LR_PWM PA1                  //LR电机调速引脚
#define LR_ENCODER_A PB8           //LR编码器A相引脚
#define LR_ENCODER_B PB9           //LR编码器B相引脚
 
//以下为右后电机引脚定义
#define RR_Motor_IN1 PC8           //RR电机使能引脚1
#define RR_Motor_IN2 PC9           //RR电机使能引脚2
#define RR_PWM PA3                  //RR电机调速引脚
#define RR_ENCODER_A PB15           //RR编码器A相引脚
#define RR_ENCODER_B PB14           //RR编码器B相引脚
volatile int i,m,n,position[3];
volatile float f;  //调试用的公共变量
unsigned long nowtime = 0;  //时间 
const long onesecond = 1000;

volatile int LF_Velocity = 0, LF_Count = 0;  //左前方电机编码器，Count计数变量 Velocity存储设定时间内A相下降沿的个数，与实际转速正相关
volatile int RF_Velocity = 0, RF_Count = 0;  //左后方电机编码器，Count计数变量 Velocity存储设定时间内A相下降沿的个数，与实际转速正相关
volatile int LR_Velocity = 0, LR_Count = 0;  //右前方电机编码器，Count计数变量 Velocity存储设定时间内A相下降沿的个数，与实际转速正相关
volatile int RR_Velocity = 0, RR_Count = 0;  //右后方电机编码器，Count计数变量 Velocity存储设定时间内A相下降沿的个数，与实际转速正相关
 
String Target_Value;                                                        //串口获取的速度字符串变量
volatile byte Patrol_data[5]={41,42,43};
volatile int LF_value, RF_value, LR_value, RR_value;                        //用于存储通过PI控制器计算得到的用于调整电机转速的PWM值，最大值65535
float KP = 200, KI = 20;                                                    //PI参数，此处调整会影响启动电流，低速时可能引起震荡
volatile float LF_Target = 0, RF_Target = 0, LR_Target = 0, RR_Target = 0;  //电机转速目标值,5ms定时器最大可用范围±280，2ms定时器，最大可用范围±120
 
///*********** 限幅************
//  以下两个参数让输出的PWM在一个合理区间
//  当输出的PWM小于1500时电机不转 所以要设置一个启始PWM
//  STM32单片机的PWM不能超过65535 所以 PWM_Restrict 起到限制上限的作用
//*****************************/
int startPWM = 1500;       //克服死区的启始PWM
int PWM_Restrict = 64000;  //startPW+PWM_Restric=65500<65535
 
/**********外部中断触发计数器函数（4个电机需要独立的外部中断处理函数）************
  根据转速的方向不同我们将计数器累计为正值或者负值(计数器累计为正值为负值为计数器方向)
  只有方向累计正确了才可以实现正确的调整,否则会出现逆方向满速旋转
  ※※※※※※超级重点※※※※※※
  所谓累计在正确的方向即
  (1)计数器方向
  (2)电机输出方向(控制电机转速方向的接线是正着接还是反着接)
  (3)PI 控制器 里面的误差(Basi)运算是目标值减当前值(Target-Encoder),还是当前值减目标值(Encoder-Target)
  三个方向只有对应上才会有效果否则你接上就是使劲的朝着一个方向(一般来说是反方向)满速旋转，出现这种问题，需要将AB相的线调过来，或改下引脚定义
  我例子里是我自己对应好的,如果其他驱动单片机在自己尝试的时候出现满速旋转就是三个方向没对应上
  下列函数中由于在A相上升沿触发时,B相是低电平,和A相下降沿触发时B是高电平是一个方向,在这种触发方式下,我们将count累计为正,另一种情况将count累计为负
********************************************/
void LF_READ_ENCODER_A()  //左前方电机A相中断
{
  if (digitalRead(LF_ENCODER_A) == HIGH) {
    if (digitalRead(LF_ENCODER_B) == LOW)
      LF_Count++;  //根据另外一相电平判定方向
    else
      LF_Count--;
  } else {
    if (digitalRead(LF_ENCODER_B) == LOW)
      LF_Count--;  //根据另外一相电平判定方向
    else
      LF_Count++;
  }
}
void RF_READ_ENCODER_A()  //右前方电机A相中断
{
  if (digitalRead(RF_ENCODER_A) == HIGH) {
    if (digitalRead(RF_ENCODER_B) == LOW)
      RF_Count++;  //根据另外一相电平判定方向
    else
      RF_Count--;
  } else {
    if (digitalRead(RF_ENCODER_B) == LOW)
      RF_Count--;  //根据另外一相电平判定方向
    else
      RF_Count++;
  }
}
void LR_READ_ENCODER_A()  //左后方电机A相中断
{
  if (digitalRead(LR_ENCODER_A) == HIGH) {
    if (digitalRead(LR_ENCODER_B) == LOW)
      LR_Count++;  //根据另外一相电平判定方向
    else
      LR_Count--;
  } else {
    if (digitalRead(LR_ENCODER_B) == LOW)
      LR_Count--;  //根据另外一相电平判定方向
    else
      LR_Count++;
  }
}
 
void RR_READ_ENCODER_A()  //右后方电机A相中断
{
  if (digitalRead(RR_ENCODER_A) == HIGH) {
    if (digitalRead(RR_ENCODER_B) == LOW)
      RR_Count++;  //根据另外一相电平判定方向
    else
      RR_Count--;
  } else {
    if (digitalRead(RR_ENCODER_B) == LOW)
      RR_Count--;  //根据另外一相电平判定方向
    else
      RR_Count++;
  }
}
 
/***********PI控制器****************/
int LF_Incremental_PI(int LF_Encoder, float LF_Target1) {
  static float LF_Bias, LF_MPWM = 0, LF_Last_bias = 0;      //定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
  LF_Bias = LF_Target1 - LF_Encoder;                        //计算偏差,目标值减去当前值
  LF_MPWM += KP * (LF_Bias - LF_Last_bias) + KI * LF_Bias;  //增量式PI控制计算
  if (LF_MPWM > PWM_Restrict)
    LF_MPWM = PWM_Restrict;  //限幅
  if (LF_MPWM < -PWM_Restrict)
    LF_MPWM = -PWM_Restrict;  //限幅
  LF_Last_bias = LF_Bias;     //保存上一次偏差
  return LF_MPWM;             //增量输出
}
int RF_Incremental_PI(int RF_Encoder, float RF_Target1) {
  static float RF_Bias, RF_MPWM = 0, RF_Last_bias = 0;      //定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
  RF_Bias = RF_Target1 - RF_Encoder;                        //计算偏差,目标值减去当前值
  RF_MPWM += KP * (RF_Bias - RF_Last_bias) + KI * RF_Bias;  //增量式PI控制计算
  if (RF_MPWM > PWM_Restrict)
    RF_MPWM = PWM_Restrict;  //限幅
  if (RF_MPWM < -PWM_Restrict)
    RF_MPWM = -PWM_Restrict;  //限幅
  RF_Last_bias = RF_Bias;     //保存上一次偏差
  return RF_MPWM;             //增量输出
}
int LR_Incremental_PI(int LR_Encoder, float LR_Target1) {
  static float LR_Bias, LR_MPWM = 0, LR_Last_bias = 0;      //定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
  LR_Bias = LR_Target1 - LR_Encoder;                        //计算偏差,目标值减去当前值
  LR_MPWM += KP * (LR_Bias - LR_Last_bias) + KI * LR_Bias;  //增量式PI控制计算
  if (LR_MPWM > PWM_Restrict)
    LR_MPWM = PWM_Restrict;  //限幅
  if (LR_MPWM < -PWM_Restrict)
    LR_MPWM = -PWM_Restrict;  //限幅
  LR_Last_bias = LR_Bias;     //保存上一次偏差
  return LR_MPWM;             //增量输出
}
int RR_Incremental_PI(int RR_Encoder, float RR_Target1) {
  static float RR_Bias, RR_MPWM = 0, RR_Last_bias = 0;      //定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
  RR_Bias = RR_Target1 - RR_Encoder;                        //计算偏差,目标值减去当前值
  RR_MPWM += KP * (RR_Bias - RR_Last_bias) + KI * RR_Bias;  //增量式PI控制计算
  if (RR_MPWM > PWM_Restrict)
    RR_MPWM = PWM_Restrict;  //限幅
  if (RR_MPWM < -PWM_Restrict)
    RR_MPWM = -PWM_Restrict;  //限幅
  RR_Last_bias = RR_Bias;     //保存上一次偏差
  return RR_MPWM;             //增量输出
}
 
/**********电机驱动函数*********/
void LF_Set_PWM(int LF_motora) {
  if (LF_motora > 0)  //如果算出的PWM为正
  {
 
    digitalWrite(LF_Motor_IN1, 1);
    digitalWrite(LF_Motor_IN2, 0);
    pwmWrite(LF_PWM, LF_motora + startPWM);  //让PWM在设定正转方向(我们认为的正转方向)正向输出调整，此处的PWM输出函数跟Mega2560不同
  } else if (LF_motora == 0)                 //如果PWM为0停车
  {
    digitalWrite(LF_Motor_IN2, 0);
    digitalWrite(LF_Motor_IN1, 0);
  } else if (LF_motora < 0)  //如果算出的PWM为负
  {
 
    digitalWrite(LF_Motor_IN1, 0);
    digitalWrite(LF_Motor_IN2, 1);
    pwmWrite(LF_PWM, -LF_motora + startPWM);  //让PWM在设定反转方向反向输出调整
  }
}
void RF_Set_PWM(int RF_motora) {
  if (RF_motora > 0)  //如果算出的PWM为正
  {
 
    digitalWrite(RF_Motor_IN1, 1);
    digitalWrite(RF_Motor_IN2, 0);
    pwmWrite(RF_PWM, RF_motora + startPWM);  //让PWM在设定正转方向(我们认为的正转方向)正向输出调整
  } else if (RF_motora == 0)                 //如果PWM为0停车
  {
    digitalWrite(RF_Motor_IN2, 0);
    digitalWrite(RF_Motor_IN1, 0);
  } else if (RF_motora < 0)  //如果算出的PWM为负
  {
 
    digitalWrite(RF_Motor_IN1, 0);
    digitalWrite(RF_Motor_IN2, 1);
    pwmWrite(RF_PWM, -RF_motora + startPWM);  //让PWM在设定反转方向反向输出调整
  }
}
void LR_Set_PWM(int LR_motora) {
  if (LR_motora > 0)  //如果算出的PWM为正
  {
 
    digitalWrite(LR_Motor_IN1, 1);
    digitalWrite(LR_Motor_IN2, 0);
    pwmWrite(LR_PWM, LR_motora + startPWM);  //让PWM在设定正转方向(我们认为的正转方向)正向输出调整
  } else if (LR_motora == 0)                 //如果PWM为0停车
  {
    digitalWrite(LR_Motor_IN2, 0);
    digitalWrite(LR_Motor_IN1, 0);
  } else if (LR_motora < 0)  //如果算出的PWM为负
  {
 
    digitalWrite(LR_Motor_IN1, 0);
    digitalWrite(LR_Motor_IN2, 1);
    pwmWrite(LR_PWM, -LR_motora + startPWM);  //让PWM在设定反转方向反向输出调整
  }
}
void RR_Set_PWM(int RR_motora) {
  if (RR_motora > 0)  //如果算出的PWM为正
  {
 
    digitalWrite(RR_Motor_IN1, 1);
    digitalWrite(RR_Motor_IN2, 0);
    pwmWrite(RR_PWM, RR_motora + startPWM);  //让PWM在设定正转方向(我们认为的正转方向)正向输出调整
  } else if (RR_motora == 0)                 //如果PWM为0停车
  {
    digitalWrite(RR_Motor_IN2, 0);
    digitalWrite(RR_Motor_IN1, 0);
  } else if (RR_motora < 0)  //如果算出的PWM为负
  {
 
    digitalWrite(RR_Motor_IN1, 0);
    digitalWrite(RR_Motor_IN2, 1);
    pwmWrite(RR_PWM, -RR_motora + startPWM);  //让PWM在设定反转方向反向输出调整
  }
}
/**********定时器中断触发函数（）*********/
HardwareTimer time3(3);  //声明使用3号定时器，电机控制闭环
HardwareTimer time1(1);  //声明使用1号定时器，位置控制滨海
 
void control() {   //3号定时中断
   //  cli();    //关闭所有中断，此处尝试不加也行
  //把采用周期(内部定时中断周期)所累计的脉冲下降沿的个数,赋值给速度
  LF_Velocity = LF_Count;
  RF_Velocity = RF_Count;
  LR_Velocity = LR_Count;
  RR_Velocity = RR_Count;
 
  //脉冲计数器清零
  LF_Count = 0;
  RF_Count = 0;
  LR_Count = 0;
  RR_Count = 0;
 
  //以下为4个电机同时计算PID参数
  LF_value = LF_Incremental_PI(LF_Velocity, LF_Target);  //通过目标值和当前值在PID函数下算出我们需要调整用的PWM值
  RF_value = RF_Incremental_PI(RF_Velocity, RF_Target);
  LR_value = LR_Incremental_PI(LR_Velocity, LR_Target);
  RR_value = RR_Incremental_PI(RR_Velocity, RR_Target);
  //以下为4个电机同时输出PWM值
  LF_Set_PWM(LF_value);
  RF_Set_PWM(RF_value);
  LR_Set_PWM(LR_value);
  RR_Set_PWM(RR_value);
 
  //以下为调试代码，调试完成需要删除，避免浪费CPU资源
 
 // Serial1.print(LF_value);  //输出左前轮的PWM值
 // Serial.print(",");
 // Serial1.println(LF_Velocity);  //输出左前轮的转速
 //  sei();     //打开所有中断，此处尝试不加也行
}
 
void patrol() {  //1号定时中断
                 // i = ~i;
                 // digitalWrite(LED, i);//调试中断的LED灯
 
  while (Serial3.available() > 3)  //正常此时缓存中收到巡线传感器的数据只有2个字节，超过2个字节的数据，则是错误的数据，应当舍弃。
  {
    while (Serial3.read() >= 0) {}  //清空串口寄存器中错误数据的缓存
  }
  while (Serial3.available() > 0)  //检测串口是否接收到了数据
  {
    Patrol_data[0] = Serial3.read();  //读取串口字符串，此为偏移方向
    Patrol_data[1] = Serial3.read();  //读取串口字符串，此为偏移量高8位
    Patrol_data[2] = Serial3.read();  //读取串口字符串，此为偏移量低8位
  }
  m = (Patrol_data[0] & 0x01);                  //当前偏移方向
  n = (Patrol_data[1] * 256 + Patrol_data[2]);  //当前偏移量
  if (m == 0)
    n = 0 - n;          //偏移量取负值
  Serial1.print(LF_Target - RF_Target);
  Serial1.print(",");
  Serial1.println(n);   //将含符号位的偏移量发出
  Serial3.write(0x57);  //发送指令给巡线传感器，数据返回需要时间（约10ms），数据会在下一次中断触发中被接收
}
 
 
void setup()
{
  Serial1.begin(115200);  //打开串口1，PA9、PA10，用于控制和反馈信息传输
  Serial3.begin(9600);  //打开串口3，PB10、PB11，用于与位置传感器通讯
 
 
  delay(1000);
 
  pinMode(LED, OUTPUT);  //调试用的闪烁LED
 
  {  //初始化4个电机的控制和反馈引脚
   pinMode(LF_ENCODER_A, INPUT);  //设置两个相线为输入模式
   pinMode(LF_ENCODER_B, INPUT);
   pinMode(LF_Motor_IN1, OUTPUT_OPEN_DRAIN);  //设置两个驱动引脚为输出模式，由于stm32的引脚接收5V作为输出时需要工作在开漏输出模式下，这与Mega2560函数定义是有差别的，Mega2560可以直接推挽输出5V，引脚直接配置为OUTPUT即可
   pinMode(LF_Motor_IN2, OUTPUT_OPEN_DRAIN);
   pinMode(LF_PWM, PWM_OPEN_DRAIN);
 
   pinMode(RF_ENCODER_A, INPUT);  //设置两个相线为输入模式
   pinMode(RF_ENCODER_B, INPUT);
   pinMode(RF_Motor_IN1, OUTPUT_OPEN_DRAIN);  //设置两个驱动引脚为输出模式，由于stm32的引脚接收5V作为输出时需要工作在开漏输出模式下，这与Mega2560函数定义是有差别的，Mega2560可以直接推挽输出5V，引脚直接配置为OUTPUT即可
   pinMode(RF_Motor_IN2, OUTPUT_OPEN_DRAIN);
   pinMode(RF_PWM, PWM_OPEN_DRAIN);
 
   pinMode(LR_ENCODER_A, INPUT);  //设置两个相线为输入模式
   pinMode(LR_ENCODER_B, INPUT);
   pinMode(LR_Motor_IN1, OUTPUT_OPEN_DRAIN);  //设置两个驱动引脚为输出模式，由于stm32的引脚接收5V作为输出时需要工作在开漏输出模式下，这与Mega2560函数定义是有差别的，Mega2560可以直接推挽输出5V，引脚直接配置为OUTPUT即可
   pinMode(LR_Motor_IN2, OUTPUT_OPEN_DRAIN);
   pinMode(LR_PWM, PWM_OPEN_DRAIN);
 
   pinMode(RR_ENCODER_A, INPUT);  //设置两个相线为输入模式
   pinMode(RR_ENCODER_B, INPUT);
   pinMode(RR_Motor_IN1, OUTPUT_OPEN_DRAIN);  //设置两个驱动引脚为输出模式，由于stm32的引脚接收5V作为输出时需要工作在开漏输出模式下，这与Mega2560函数定义是有差别的，Mega2560可以直接推挽输出5V，引脚直接配置为OUTPUT即可
   pinMode(RR_Motor_IN2, OUTPUT_OPEN_DRAIN);
   pinMode(RR_PWM, PWM_OPEN_DRAIN);
  }
  //下面是外部中断的初始化
  attachInterrupt(LF_ENCODER_A, LF_READ_ENCODER_A, FALLING);  //开启对应A相引脚的外部中断,触发方式为FALLING 即下降沿都触发,触发的中断函数为 LF_ENCODER_A
  attachInterrupt(RF_ENCODER_A, RF_READ_ENCODER_A, FALLING);
  attachInterrupt(LR_ENCODER_A, LR_READ_ENCODER_A, FALLING);
  attachInterrupt(RR_ENCODER_A, RR_READ_ENCODER_A, FALLING);
 
  //下面是3号定时器（电机速度闭环）的初始化，Mega2560的用法与此处有差异，参考引用库函数才行
  time3.pause();                                   // Pause the timer while we're configuring it
  time3.setPeriod(5000);                           // Set up period in microseconds，5000us=5ms
  time3.setMode(TIMER_CH3, TIMER_OUTPUT_COMPARE);  // Set up an interrupt on channel 1
  time3.setCompare(TIMER_CH3, 1);                  // Interrupt 3 count after each update
  time3.attachInterrupt(3, control);               //定时中断函数名声明
 
  //下面是1号定时器（位置闭环）的初始化，Mega2560的用法与此处有差异，参考引用库函数才行
  time1.pause();                                   // Pause the timer while we're configuring it
  time1.setPeriod(100000);                         // Set up period in microseconds，100000us=100ms
  time1.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);  // Set up an interrupt on channel 1
  time1.setCompare(TIMER_CH1, 1);                  // Interrupt 1 count after each update
  time1.attachInterrupt(1, patrol);                //定时中断函数名声明
  //启动3号和1号定时器
  time3.refresh();                                  // Refresh the timer's count, prescale, and overflow
  time1.refresh();                                 // Refresh the timer's count, prescale, and overflow
  time3.resume();                                  // Start the timer counting
  time1.resume();                                  // Start the timer counting
  //  digitalWrite(LED, 1);
}
 
void loop() 
{
   while (Serial1.available() > 0)  //检测串口是否接收到了数据
   {
     Target_Value = Serial.readString();  //读取串口字符串
     i = Target_Value.toFloat();          //将字符串转换为浮点型,并将其赋给目标值
     speed = 20;
 
     if (i == 1)  //实验一，20 +- 5 向右
     {
      
       LF_Target = -(speed + 5);
       LR_Target = -(speed + 5);
       RF_Target = -(speed - 5);
       RR_Target = -(speed - 5);
       nowtime = millis();
       while(millis() <= nowtime + 1.5*onesecond){
        
       }
       LF_Target = 0;
       LR_Target = 0;
       RF_Target = 0;
       RR_Target = 0;
       nowtime = millis();
       while(millis() <= nowtime + 0.5*onesecond){
        
       }
       LF_Target = -(speed - 7.5);
       LR_Target = -(speed - 7.5);
       RF_Target = -(speed + 7.5);
       RR_Target = -(speed + 7.5);
       nowtime = millis();
       while(millis() <= nowtime + 2*onesecond){
        
       }
       LF_Target = 0;
       LR_Target = 0;
       RF_Target = 0;
       RR_Target = 0;
       nowtime = millis();
       while(millis() <= nowtime + 0.5*onesecond){
        
       }
       LF_Target = -(speed + 7.5);
       LR_Target = -(speed + 7.5);
       RF_Target = -(speed - 7.5);
       RR_Target = -(speed - 7.5);
       nowtime = millis();
       while(millis() <= nowtime + 1*onesecond){
        
       }
       LF_Target = 0;
       LR_Target = 0;
       RF_Target = 0;
       RR_Target = 0;
       nowtime = millis();
       while(millis() <= nowtime + 0.5*onesecond){
        
       }
       LF_Target = -(speed - 5);
       LR_Target = -(speed - 5);
       RF_Target = -(speed + 5);
       RR_Target = -(speed + 5);
       nowtime = millis();
       while(millis() <= nowtime + 1*onesecond){
        
       }
       LF_Target = 0;
       LR_Target = 0;
       RF_Target = 0;
       RR_Target = 0;
     } else if (i == 0)  //底盘停止
     {
       LF_Target = 0;
       LR_Target = 0;
       RF_Target = 0;
       RR_Target = 0;
     }
   }
}
