#include<stdio.h>
#include<wiringPi.h>

#define A 1
#define B 4
#define C 23
#define D 24
#define E 3
#define F 7
#define G 0
#define LED1 2
#define LED2 21
#define LED3 22
#define LED4 25
int choose_LED(int count)
{
    if(count==1)
    {
        digitalWrite(LED1,HIGH);
        digitalWrite(LED2,LOW);
        digitalWrite(LED3,LOW);
        digitalWrite(LED4,LOW);
        return 1;
    }
    else if(count==2)
    {
        digitalWrite(LED1,LOW);
        digitalWrite(LED2,HIGH);
        digitalWrite(LED3,LOW);
        digitalWrite(LED4,LOW);
        return 1;
    }
    else if(count==3)
    {
        digitalWrite(LED1,LOW);
        digitalWrite(LED2,LOW);
        digitalWrite(LED3,HIGH);
        digitalWrite(LED4,LOW);
        return 1;
    }
    else if(count==4)
    {
        digitalWrite(LED1,LOW);
        digitalWrite(LED2,LOW);
        digitalWrite(LED3,LOW);
        digitalWrite(LED4,HIGH);
        return 1;
    }
    else
    {
        return -1;
    }
}
int choose_num(int num)
{
    if(num==0)
    {
        digitalWrite(A,LOW);
        digitalWrite(B,LOW);
        digitalWrite(C,LOW);
        digitalWrite(D,LOW);
        digitalWrite(E,LOW);
        digitalWrite(F,LOW);
        digitalWrite(G,HIGH);
        return 1;
    }
    else if(num==1)
    {
        digitalWrite(A,HIGH);
        digitalWrite(B,LOW);
        digitalWrite(C,LOW);
        digitalWrite(D,HIGH);
        digitalWrite(E,HIGH);
        digitalWrite(F,HIGH);
        digitalWrite(G,HIGH);
        return 1;
    }
    else if(num==2)
    {
        digitalWrite(A,LOW);
        digitalWrite(B,LOW);
        digitalWrite(C,HIGH);
        digitalWrite(D,LOW);
        digitalWrite(E,LOW);
        digitalWrite(F,HIGH);
        digitalWrite(G,LOW);
        return 1;
    }
    else if(num==3)
    {
        digitalWrite(A,LOW);
        digitalWrite(B,LOW);
        digitalWrite(C,LOW);
        digitalWrite(D,LOW);
        digitalWrite(E,HIGH);
        digitalWrite(F,HIGH);
        digitalWrite(G,LOW);
        return 1;
    }
    else if(num==4)
    {
        digitalWrite(A,HIGH);
        digitalWrite(B,LOW);
        digitalWrite(C,LOW);
        digitalWrite(D,HIGH);
        digitalWrite(E,HIGH);
        digitalWrite(F,LOW);
        digitalWrite(G,LOW);
        return 1;
    }
    else if(num==5)
    {
        digitalWrite(A,LOW);
        digitalWrite(B,HIGH);
        digitalWrite(C,LOW);
        digitalWrite(D,LOW);
        digitalWrite(E,HIGH);
        digitalWrite(F,LOW);
        digitalWrite(G,LOW);
        return 1;
    }
    else if(num==6)
    {
        digitalWrite(A,LOW);
        digitalWrite(B,HIGH);
        digitalWrite(C,LOW);
        digitalWrite(D,LOW);
        digitalWrite(E,LOW);
        digitalWrite(F,LOW);
        digitalWrite(G,LOW);
        return 1;
    }
    else if(num==7)
    {
        digitalWrite(A,LOW);
        digitalWrite(B,LOW);
        digitalWrite(C,LOW);
        digitalWrite(D,HIGH);
        digitalWrite(E,HIGH);
        digitalWrite(F,HIGH);
        digitalWrite(G,HIGH);
        return 1;
    }
    else if(num==8)
    {
        digitalWrite(A,LOW);
        digitalWrite(B,LOW);
        digitalWrite(C,LOW);
        digitalWrite(D,LOW);
        digitalWrite(E,LOW);
        digitalWrite(F,LOW);
        digitalWrite(G,LOW);
        return 1;
    }
    else if(num==9)
    {
        digitalWrite(A,LOW);
        digitalWrite(B,LOW);
        digitalWrite(C,LOW);
        digitalWrite(D,LOW);
        digitalWrite(E,HIGH);
        digitalWrite(F,LOW);
        digitalWrite(G,LOW);
        return 1;
    }
    else
    {
        return -1;
    }
}

int show_one_number(int count,int num)
{
    if(choose_LED(count)<0)
    {
        printf("cannot choose LED");
    }
    if(choose_num(num)<0)
    {
        printf("cannot choose num");
    }
}

int init(void)
{
    if(wiringPiSetup()==-1)
    {
        printf("io cannot init");
        return -1;
    }
    pinMode(A, OUTPUT);
    pinMode(B, OUTPUT);
    pinMode(C, OUTPUT);
    pinMode(D, OUTPUT);
    pinMode(E, OUTPUT);
    pinMode(F, OUTPUT);
    pinMode(G, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);
    pinMode(LED4, OUTPUT);
    return 1;
}

int main()
{
    init();
    int inputnum,temp,tempmod;
    int led[4];
    printf("input a 4d num i will show you back\n");
    scanf("%d",&inputnum);
    while(1)
    {
        int j = 0;
        temp = inputnum;
        for(int i = 1000; temp>=1;)
        {

            tempmod = temp%i;
            led[j] = (temp-tempmod)/i;
            temp = tempmod;
            printf("\n%d\n",j+1);
            show_one_number(j+1,led[j]);
            for(int z =1;z<1000;z++){}
            j++;
            i = i/10;
        }
    }

}
