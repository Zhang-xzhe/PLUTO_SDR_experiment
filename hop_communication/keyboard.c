/*
    读取4*4键盘矩阵的键值 
    四行四列 八个引脚
    采用轮询的方式获取行列坐标
    假如S1键按下，先扫描行引脚，即行引脚配置为输入，列引脚持续输出高电平。此时接第一行的引脚会检测到高电平。
    设置行引脚为输出高电平，列引脚配置为输入。此时接第一列的引脚检测到高电平。
    这样我们就得到了第一行第一列的按键被按下。
    注意：树莓派4B的pin脚均被预先设置好输出输出模式（Mode）及固定的原始输入输出电平（V）
    例如某原始模式为输入模式的引脚的原始输入电平值为0，该引脚悬空，
    此时设置该引脚为上拉输入还是下拉输入并不能钳制电平，读该引脚的值只会是其原始输入电平值
    树莓派4B与其他单片机不同的地方在于其会给引脚一个固定的一个原始输入输出电平值
    可在树莓派shell命令行中输入 gpio readall 查看其初始设置和不同编号对应的引脚值
*/

#include <wiringPi.h>
#include <stdio.h>

#define R1 27   //键盘矩阵(row1)     行1     wiringPi编号24（BCM 19）
#define R2 28   //键盘矩阵(row2)     行2     wiringPi编号26（BCM 12）
#define R3 29   //键盘矩阵(row3)     行3     wiringPi编号23（BCM 13） 

#define C1 26   //键盘矩阵(column1)  列1     wiringPi编号28（BCM 20）
#define C2 6   //键盘矩阵(column2)  列2     wiringPi编号25（BCM 26）
#define C3 5    //键盘矩阵(column3)  列3     wiringPi编号6 （BCM 25）


int key[3][3] = {{1,2,3}, {4,5,6}, {7,8,9}};

/*
    int wiringPiSetup (void) 
    功能：初始化树莓派pin脚，使用wiringPi引脚编号表
    作用：当使用这个函数初始化树莓派引脚时，程序使用的是wiringPi 引脚编号表。引脚的编号为 0~16。需要root权限
    返回值：执行状态，-1表示失败

    void pullUpDnControl (int pin, int pud)	
    功能：对一个设置IO模式为 INPUT 的输入引脚设置拉电阻模式。
    参数：
        pin：引脚
        pud：拉电阻模式
            PUD_OFF     关闭拉电阻
            PUD_DOWN    下拉 引脚电平拉到0v 接地
            PUD_UP      上拉 引脚电平拉到3.3v
    
    void digitalWrite (int pin, int value)	
			作用：让对一个已近配置为输出模式的 引脚  输出指定的电平信号
				pin：控制的引脚
				value：引脚输出的电平值。
					可取的值：HIGH，LOW分别代表高低电平
    
    int digitalRead (int pin)
		功能：读取一个引脚的电平值  LOW  HIGH ，返回	
		参数
			pin：读取的引脚
		返回值：引脚上的电平，可以是LOW HIGH 之一
*/

int get_row()
{
    //设置列 C1~C4 输出高电平
    pinMode(C1, OUTPUT);
    digitalWrite(C1, HIGH);
    pinMode(C2, OUTPUT);
    digitalWrite(C2, HIGH);
    pinMode(C3, OUTPUT);
    digitalWrite(C3, HIGH);

    //设置行 R1~R4 为下拉输入
    pinMode(R1, INPUT);
    pullUpDnControl(R1, PUD_DOWN);
    pinMode(R2, INPUT);
    pullUpDnControl(R2, PUD_DOWN);
    pinMode(R3, INPUT);
    pullUpDnControl(R3, PUD_DOWN);

    //哪一行检测到高电平就返回哪一行的行号
    if (digitalRead(R1) == HIGH)
    {
        delay(10);      //延时去抖动
        if (digitalRead(R1) == HIGH)
        {
            return 1;
        }
        
    }
    if (digitalRead(R2) == HIGH)
    {
        delay(10);      //延时去抖动
        if (digitalRead(R2) == HIGH)
        {
            return 2;
        }
        
    }
    if (digitalRead(R3) == HIGH)
    {
        delay(10);      //延时去抖动
        if (digitalRead(R3) == HIGH)
        {
            return 3;
        }
        
    }

    //所有行都没有检测到高电平就返回-1
    return -1;
}

int get_column()
{
    //设置行 R1~R4 输出高电平
    pinMode(R1, OUTPUT);
    digitalWrite(R1, HIGH);
    pinMode(R2, OUTPUT);
    digitalWrite(R2, HIGH);
    pinMode(R3, OUTPUT);
    digitalWrite(R3, HIGH);

    //设置列 C1~C4 为下拉输入
    pinMode(C1, INPUT);
    pullUpDnControl(C1, PUD_DOWN);
    pinMode(C2, INPUT);
    pullUpDnControl(C2, PUD_DOWN);
    pinMode(C3, INPUT);
    pullUpDnControl(C3, PUD_DOWN);

    //哪一列检测到高电平就返回哪一列的列号
    if (digitalRead(C1) == HIGH)
    {
        delay(10);      //延时去抖动
        if (digitalRead(C1) == HIGH)
        {
            return 1;
        }
        
    }
    if (digitalRead(C2) == HIGH)
    {
        delay(10);      //延时去抖动
        if (digitalRead(C2) == HIGH)
        {
            return 2;
        }
        
    }
    if (digitalRead(C3) == HIGH)
    {
        delay(10);      //延时去抖动
        if (digitalRead(C3) == HIGH)
        {
            return 3;
        }
        
    }

    //所有列都没有检测到高电平就返回-1
    return -1;
}

int main(void)
{
	//初始化连接失败时，将消息打印到屏幕
	if(wiringPiSetup() == -1){ 
		printf("setup wiringPi failed !");
		return 1; 
	}
    
    while (1)
    {
        int row, column;
        if( (row = get_row()) > 0)
        {
            if ((column = get_column()) > 0)
            {
                do
                {
                    ;   //等待按键弹起
                } while (get_row() > 0);
                
                printf("(%d,%d)\n",row, column);
            }
            
        }
        
    }
    

	return 0;
}


