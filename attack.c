#include <stm32f4xx.h>

#include "HardwareInfo.c"
#include "JMLib.c"
#include <GetData.h>
#include <GetCompoI3.h>
#include <GetADScable10.h>
#include <GetCompassB.h>
#include <SetInBeep.h>
#include "eye.c"
#include "Equation_of_motion.c"
#include <SetLCD5Char.h>
// define global var
double L1 = 0;
double L2 = 0;
double L3 = 0;
double g2r = 0;
long g4 = 0;
int g6in = 0;
int g6out = 0;
int g9in = 0;
double c = 0;
double speed = 0;
double path = 0;
double vw = 0;
double x = 0;
double y = 0;
double angle = 0;
double distance = 0;
int s3 = 0;
int s6 = 0;
int s9 = 0;
int s0 = 0;
int g0in = 0;
int g0out = 0;
int g3in = 0;
int g3out = 0;
double f = 0;
double fc = 0;
double area = 0;
double path00 = 0;
long s = 0;
long g1r = 0;
long g3r = 0;
long g4r = 0;
long g5r = 0;
long g6r = 0;
long g7r = 0;
long g8r = 0;
int g_sum_rec = 0;
long g = 0;
long s1 = 0;
long s2 = 0;
long L4 = 0;
int n = 0;
long g_1 = 0;
long fadd = 0;
int g_0out = 0;
int g_0in = 0;
int g_3out = 0;
int g_3in = 0;
int g_6out = 0;
int g_6in = 0;
int g_9out = 0;
int g_9in = 0;
long g_sum_rt = 0;
long line_rec_add = 0;
long g_4 = 0;
long g_5 = 0;
long g_6 = 0;
long g_add = 0;
long g_debouncing = 0;
int g9out = 0;
long f_debouncing = 0;
long f_add = 0;
long aa = 0;
long g_13 = 0;
long line = 400;
long line_rec = 0;

int main(void)
{
    X4RCU_Init();
    double aimx = 0;
    double aimy = 0;
    double dx = 0;
    double dy = 0;
    long B3 = 0;
    long B3_R = 0;
    long increment = 0;
    long s3_mutation = 0;
    long s9_mutation = 0;
    long s3_rec = 0;
    long s9_rec = 0;
    long area_rec = 0;
    long arear = 0;
    int caim = 0;
    int gate = 0;
    int dribble_debouncing = 0;
    int pre_rotation_debouncing = 0;
    int dribble_sp = 0;
    int dribble_state = 0;
    long cam_data = 0;
    int cam_step = 0;
    int cam_x = 0;
    int cam_y = 0;
    int kick_sp = 0;
    int f_check = 0;
    int camn = 0;
    int g_kicker = 0;
    long var1 = 0;
    long var2 = 0;
    int original_line=0;
    
    //灰度數值
    int g0=0, g45=0, g90=0, g135=0, g180=0, g225=0, g270=0, g315=0;
    //從rcu裏面read保存的灰度數值
    int g0_r=0, g45_r=0, g90_r=0, g135_r=0, g180_r=0, g225_r=0, g270_r=0, g315_r=0;
    //灰度踩到白界為1，沒有踩到為0，且為即時變化型
    int g0_rt=0, g45_rt=0, g90_rt=0, g135_rt=0, g180_rt=0, g225_rt=0, g270_rt=0, g315_rt=0;
    //灰度踩到白界為1，沒有踩到為0，且為記錄型
    int g0_rec=0, g45_rec=0, g90_rec=0, g135_rec=0, g180_rec=0, g225_rec=0, g270_rec=0, g315_rec=0;
    
    g0_r = GetData(1);
    g45_r = GetData(2);
    g90_r = GetData(3);
    g135_r = GetData(4);
    g180_r = GetData(5);
    g225_r = GetData(6);
    g270_r = GetData(7);
    g315_r = GetData(8);
    increment = GetData(9);
    //rcu的灰度保存值加上一个增量
    g0_r=g0_r+increment;
    g45_r=g45_r+increment;
    g90_r=g90_r+increment;
    g135_r=g135_r+increment;
    g180_r=g180_r+increment;
    g225_r=g225_r+increment;
    g270_r=g270_r+increment;
    g315_r=g315_r+increment;
    GetCompoI3(_COMPOUNDEYE3_fronteye_, 14);
    GetCompoI3(_COMPOUNDEYE3_backeye_, 14);
    while (1)
    {
        speed = GetData(10);
        s0 = GetADScable10(_SCABLEAD_s0_);
        s3 = GetADScable10(_SCABLEAD_s3_);
        s6 = GetADScable10(_SCABLEAD_s6_);
        s9 = GetADScable10(_SCABLEAD_s9_);
        g0 = GetADScable10(_SCABLEAD_g0_);
        g45 = GetADScable10(_SCABLEAD_g45_);
        g90 = GetADScable10(_SCABLEAD_g90_);
        g135 = GetADScable10(_SCABLEAD_g135_);
        g180 = GetADScable10(_SCABLEAD_g180_);
        g225 = GetADScable10(_SCABLEAD_g225_);
        g270 = GetADScable10(_SCABLEAD_g270_);
        g315 = GetADScable10(_SCABLEAD_g315_);
        g_kicker = GetADScable10(_SCABLEAD_gkicker_);
        gate = GetADScable10(_SCABLEAD_gate_);
        //灰度判斷碰到白界限部分
        //rec：record记录（用来记录下状态，用于未来做参考判断）
        //rt ： real time实时
        if(g0>g0_r)
        {
        g0_rec=1;
        g0_rt=1;
        }
        else
        {
        g0_rt=0;
        }
        
        if(g45>g45_r)
        {
        g45_rec=1;
        g45_rt=1;
        }
        else
        {
        g45_rt=0;
        }
        
        if(g90>g90_r)
        {
        g90_rec=1;
        g90_rt=1;
        }
        else
        {
        g90_rt=0;
        }
        
        if(g135>g135_r)
        {
        g135_rec=1;
        g135_rt=1;
        }
        else
        {
        g135_rt=0;
        }
        
        if(g180>g180_r)
        {
        g180_rec=1;
        g180_rt=1;
        }
        else
        {
        g180_rt=0;
        }
        
        if(g225>g225_r)
        {
        g225_rec=1;
        g225_rt=1;
        }
        else
        {
        g225_rt=0;
        }
        
        if(g270>g270_r)
        {
        g270_rec=1;
        g270_rt=1;
        }
        else
        {
        g270_rt=0;
        }
        
        if(g315>g315_r)
        {
        g315_rec=1;
        g315_rt=1;
        }
        else
        {
        g315_rt=0;
        }
        
        g_sum_rec=g0_rec+g45_rec+g90_rec+g135_rec+g180_rec+g225_rec+g270_rec+g315_rec;//rec：record记录（用来记录下状态，用于未来做参考判断）
        g_sum_rt=g0_rt+g45_rt+g90_rt+g135_rt+g180_rt+g225_rt+g270_rt+g315_rt;//rt ： real time实时
        
        
        
        
        if(line==400)//line==400為沒有踩到白界
        {
        if(g0_rec==1)
        {line=0;}
        else if(g45_rec==1)
        {line=45;}
        else if(g90_rec==1)
        {line=90;}
        else if(g135_rec==1)
        {line=135;}
        else if(g180_rec==1)
        {line=180;}
        else if(g225_rec==1)
        {line=225;}
        else if(g270_rec==1)
        {line=270;}
        else if(g315_rec==1)
        {line=315;}
        original_line=line;
        }
        c = GetCompassB(_COMPASS_compass_);
        c=c-caim;
        if ( c<0 )
        {
            c=c+360;
        }
        else
        {
        }
        SetInBeep(0);
        if(gate<900)
        {
        dribble_state=1; dribble_debouncing=0;
        }
        else
        {
        dribble_debouncing++;
        if(dribble_debouncing>15)
        {dribble_state=0;}
        }
        
        
        if(fc>=140&&fc<=220&&f>100||gate<1000)//gate<1000意思是
        {
        dribble_sp=-60;pre_rotation_debouncing=0;
        }
        else
        {
        pre_rotation_debouncing++;
        if(pre_rotation_debouncing>15)
        {dribble_sp=0;pre_rotation_debouncing=0;}
        } 
        
        
        eye();
        if ( s3_rec!=0&&s9_rec!=0 )
        {
            if ( s3_mutation==1 )
            {
            }
            else
            {
                if ( s3_rec-s3>100||s3-s3_rec>100 )
                {
                    s3_mutation=1;
                }
            }
            if ( s9_mutation==1 )
            {
            }
            else
            {
                if ( s9_rec-s9>100||s9-s9_rec>100 )
                {
                    s9_mutation=1;
                }
            }
        }
        s3_rec=s3;
        s9_rec=s9;
        y=s6;x=s9;
        if ( s3_mutation==1&&s9_mutation==1 )
        {
            area=area_rec;
        }
        else
        {
            if ( s9_mutation==1 )
            {
                if ( s3<500 )
                {
                    area=1;
                    arear=1;
                }
                else
                {
                    if ( s3>500&&s3<800 )
                    {
                        area=2;
                    }
                    else
                    {
                        if ( s3>800&&s3<1100 )
                        {
                            area=3;
                        }
                        else
                        {
                            if ( s3>1100 )
                            {
                                area=4;
                                arear=4;
                            }
                            else
                            {
                            }
                        }
                    }
                }
                x=1600-s3;
            }
            else
            {
                if ( s3_mutation==1 )
                {
                    if ( s9>1100 )
                    {
                        area=1;
                        arear=1;
                    }
                    else
                    {
                        if ( s9<1100&&s9>800 )
                        {
                            area=2;
                        }
                        else
                        {
                            if ( s9>500&&s9<800 )
                            {
                                area=3;
                            }
                            else
                            {
                                if ( s9<500 )
                                {
                                    area=4;
                                    arear=4;
                                }
                                else
                                {
                                }
                            }
                        }
                    }
                    x=s9;
                }
                else
                {
                    if ( s3<500 )
                    {
                        area=1;
                        arear=1;
                    }
                    else
                    {
                        if ( s3>500&&s3<800 )
                        {
                            area=2;
                        }
                        else
                        {
                            if ( s3>800&&s3<1100 )
                            {
                                area=3;
                            }
                            else
                            {
                                if ( s3>1100 )
                                {
                                    area=4;
                                    arear=4;
                                }
                                else
                                {
                                }
                            }
                        }
                    }
                    x=1600-s3;
                }
            }
        }
        if ( s3+s9>1400&&s3+s9<2000 )
        {
            s3_mutation=0;
            s9_mutation=0;
        }
        else
        {
        }
        area_rec=area;
        if ( c>350||c<10 )
        {
            vw=0;
        }
        else
        {
            if ( c<180 )
            {
                vw=2;
            }
            else
            {
                vw=-2;
            }
        }
        if ( line!=400 )
        {
            line_rec=line;line_rec_add=0;
        }
        else
        {
            if ( line_rec!=400 )
            {
                line_rec_add++;
            }
            else
            {
            }
            if ( line_rec_add>170 )
            {
                line_rec=400;line_rec_add=0;
            }
            else
            {
            }
        }
        if ( dribble_state==1&&f>4&&g_sum_rec<4 )
        {
            speed=0;
        }
        else
        {
            if ( f>4&&g_sum_rec<4 )
            {
                if(f>120)
                {f=120;}
                
                if ( fc>270||fc<90 )
                {
                    if ( fc<20||fc>=340 )
                    {
                         if ( f>120 )//判斷球是否穩定在持球位上
                            {
                                f_add++;f_debouncing=0;
                            }
                            else
                            {
                                f_debouncing++;
                                if ( f_debouncing>100 )
                                {
                                    f_add=0;f_debouncing=0;
                                }
                                else
                                {
                                }
                            }
                            
                        //變量++時候要注意數據溢出問題， 記得讓變量歸零
                        path=0;
                        if ( f_add>10 )
                        {
                            SetInBeep(1);
                            speed=speed+400;
                            if ( f_add>20 )
                            {
                                kicker_sp=100;
                            }
                            else
                            {
                            }
                        }
                        else
                        {
                        }
                    }
                    else
                    {
                        if ( fc<180 )
                        {
                            path=fc+75*f/120;
                        }
                        else
                        {
                            path=fc-75*f/120;
                        }
                    }
                }
                else
                {
                    if ( fc<210&&fc>150 )
                    {
                        if ( fc<=186&&fc>=164 )
                        {
                            path=fc;
                        }
                        else
                        {
                            if ( fc>180 )
                            {
                                path=210;
                            }
                            else
                            {
                                path=150;
                            }
                        }
                    }
                    else
                    {
                        if(fc>180)
                        {path=fc+(90*100)/f;}
                        else
                        {path=fc-(90*100)/f;} 
                    }
                }
                if ( line_rec==3&&(fc<175&&fc>5)||line_rec==6&&(fc<260&&fc>100)||line_rec==9&&(fc<355&&fc>185)||line_rec==0&&(fc<80||fc>280) )
                {
                    speed=800;
                }
                else
                {
                }
                if ( line==3&&(path<180&&path>0)||line==6&&(path<270&&path>90)||line==9&&(path<360&&path>180)||line==0&&(path<90||path>270)||(line==3||line==9)&&s3>300&s9>300 )
                {
                    if ( line==6&&s6>200||line==0&&s0>200||(line==3||line==9)&&s3>300&s9>300 )
                    {
                        if(fc>180)
                        {
                        path=270;
                        }
                        else
                        {
                        path=90;
                        }
                        
                    }
                    else
                    {
                        if ( line==3&&fc>180||line==9&&fc<180 )
                        {
                            if ( fc<180 )
                            {
                                path=fc+75*f/120;
                            }
                            else
                            {
                                path=fc-75*f/120;
                            }
                            if ( line==3&&path<180 )
                            {
                                path=185;
                            }
                            else
                            {
                                if ( line==9&&path>180&&path<=360 )
                                {
                                    path=175;
                                }
                                else
                                {
                                }
                            }
                        }
                        else
                        {
                            speed=0;path=0;
                        }
                    }
                }
                else
                {
                }
            }
            else
            {
                if ( g_sum_rec!=0 )
                {
                    if ( s0>250&&s6>250&&s9>250&&s3>250&&f>5 )
                    {
                        if ( fc<18||fc>=350 )
                        {
                            path=fc;
                        }
                        else
                        {
                            if(f>120)
                            {f=120;}
                            
                            if ( fc<180 )
                            {
                                path=fc+75*f/120;
                            }
                            else
                            {
                                path=fc-75*f/120;
                            }
                        }
                        if ( (line==6||s0>s6)&&(path>270||path<90) )
                        {
                        }
                        else
                        {
                            if ( (line==0||s0<s6)&&path<270&&path>90 )
                            {
                            }
                            else
                            {
                                if ( fc<20||fc>340||fc>160&&fc<200 )
                                {
                                    path=0;speed=0;
                                }
                                else
                                {
                                    if ( fc>180 )
                                    {
                                        path=270;
                                    }
                                    else
                                    {
                                        path=90;
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        if ( s3<200&&s9<200||s0<200&&s6<200 )
                        {
                            path=0;
                            speed=0;
                        }
                        else
                        {
                            if ( s0>200&&s6<200 )
                            {
                                path=0;
                            }
                            else
                            {
                                if ( s0<200&&s6>200 )
                                {
                                    path=180;
                                }
                                else
                                {
                                    if ( s3>200&&s9<200 )
                                    {
                                        path=90;
                                    }
                                    else
                                    {
                                        if ( s9>200&&s3<200 )
                                        {
                                            path=270;
                                        }
                                        else
                                        {
                                            aimx=870;
                                            aimy=500;
                                            dx=aimx-x;
                                            dy=aimy-y;
                                            
                                            path=360-(atan2 (dy, dx)*180/3.1415926+180)-90;
                                            if (path<0)
                                            {
                                            path=path+360;
                                            }
                                            
                                            distance=sqrt(dx*dx+dy*dy);
                                            if ( distance<80 )
                                            {
                                                path=0;speed=0;
                                                if ( c<10||c>350 )
                                                {
                                                }
                                                else
                                                {
                                                    if ( c<180 )
                                                    {
                                                        vw=4;
                                                    }
                                                    else
                                                    {
                                                        vw=-4;
                                                    }
                                                }
                                            }
                                            else
                                            {
                                                if ( distance<160 )
                                                {
                                                    speed=800;
                                                }
                                                else
                                                {
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                else
                {
                    aimx=870;
                    aimy=500;
                    dx=aimx-x;
                    dy=aimy-y;
                    
                    path=360-(atan2 (dy, dx)*180/3.1415926+180)-90;
                    if (path<0)
                    {
                    path=path+360;
                    }
                    
                    distance=sqrt(dx*dx+dy*dy);
                    if ( distance<80 )
                    {
                        path=0;speed=0;
                        if ( c<10||c>350 )
                        {
                        }
                        else
                        {
                            if ( c<180 )
                            {
                                vw=4;
                            }
                            else
                            {
                                vw=-4;
                            }
                        }
                    }
                    else
                    {
                        if ( distance<160 )
                        {
                            speed=800;
                        }
                        else
                        {
                        }
                    }
                }
            }
        }
        if ( s3>250&&s9>250&&s6>250&&s0>250&&g_sum_rt==0&&g_sum_rec!=0 )
        {
            g_add++;
            if(g_add>15)
            {
            g0_rec=0;
            g45_rec=0;
            g90_rec=0;
            g135_rec=0;
            g180_rec=0;
            g225_rec=0;
            g270_rec=0;
            g315_rec=0;
            g_sum_rec=0;
            line=400;
            }
        }
        else
        {
            g_add=0;
        }
        SetMotor2 (6,kick_sp);//kick
        SetMotor2 (5,dribble_sp);//dribble
        
        Equation_of_motion();
        SetLCD5Char(130, 60, s0, YELLOW, BLACK);
        SetLCD5Char(210, 140, s3, YELLOW, BLACK);
        SetLCD5Char(130, 220, s6, YELLOW, BLACK);
        SetLCD5Char(50, 140, s9, YELLOW, BLACK);
        SetLCD5Char(130, 110, f, 63488, BLACK);
        SetLCD5Char(130, 130, fc, 63488, BLACK);
        SetLCD5Char(130, 150, c, 31, BLACK);
        SetLCD5Char(130, 90, line, 65535, BLACK);
        SetLCD5Char(240, 20, g_sum_rec, YELLOW, BLACK);
        SetLCD5Char(240, 40, g_sum_rt, YELLOW, BLACK);
    }
    while(1);
}

