#include "usb.h"
#include "usb_host_hid.h"

#include "brc.h"
#include "slnode.h"
#include "app_host_hid_joypad.h"
#include "MPU6050.h"
#include "FutabaServo.h"

void BRC_SetCWCCW(int ch,bool ccw)
{
    switch(ch)
    {
        case 0: LAT_CWCCW1=ccw; break;
        case 1: LAT_CWCCW2=ccw; break;
        case 2: LAT_CWCCW3=ccw; break;
    }
}

int DeadZone(int v,int dz)
{
    if(v> dz) return v-dz;
    if(v<-dz) return v+dz;
    return 0;
}

int main(void)
{   
    int i;

    BRC_InitializeLEDSW();
    BRC_LEDRed(1);
    BRC_LEDGreen(0);
    
    BRC_SetupClockPLL();
    BRC_SetupUSBClock();
    BRC_InitializeSysClock(10000l);
    BRC_IORemap();

    BRC_InitializeMotorPulse();
    // MotorPulse RP bind
    StartIORemap();
    RPOUT_CLK1=BRC_OCxRPO_MPCH(0);
    RPOUT_CLK2=BRC_OCxRPO_MPCH(1);
    RPOUT_CLK3=BRC_OCxRPO_MPCH(2);
    //RPOUT_CLK4=BRC_OCxRPO_MPCH(3);
    //RPOUT_CLK5=BRC_OCxRPO_MPCH(4);
    //RPOUT_CLK6=BRC_OCxRPO_MPCH(5);
    //RPOUT_DE21=BRC_OCxRPO_MPCH(1);
    //RPOUT_DE22=BRC_OCxRPO_MPCH(2);
    EndIORemap();

    // Serial Port Binding
#if 0
    StartIORemap();
  	RPINR19bits.U2RXR=RPIN_DE22;   // UART 2 RX <- RP_DE22
	RPOUT_DE21=3;      // RP_DE21 <- UART 2 TX
    EndIORemap();
    InitializeSerial(SIB115200);
#endif
    InitializeFutabaServo(SIB115200);
    
    TRIS_ENABLE12=TRIS_OUT;    TRIS_DCY12=TRIS_OUT;
    TRIS_M12=TRIS_OUT;         TRIS_TQ12=TRIS_OUT;
    LAT_ENABLE12=0;  LAT_DCY12=0;  LAT_M12=0;  LAT_TQ12=0;
    TRIS_CWCCW1=TRIS_OUT; TRIS_CWCCW2=TRIS_OUT; TRIS_CWCCW3=TRIS_OUT;
    LAT_CWCCW1=0; LAT_CWCCW2=0; LAT_CWCCW3=0;
    
    //initial motor speed
    BRC_SetMotorSpeed(0,0);
    BRC_SetMotorSpeed(1,0);
    BRC_SetMotorSpeed(2,0);
    // some led swtich board ?
    BRC_InitializeLEDSwitchBoard();
    //init mpu
    InitializeMPU6050();
    //calibrate mpu offset - read func returns struct like below
	/*
	// FIFO reading buffer
	int FIFObuff[FIFObuffN];
	int FIFOc;
	// number of state variables
	int FIFOdataN;  
	// Raw data
	int GX,GY,GZ,AX,AY,AZ,TMP;
	// Process data
	long GyroAngleX,GyroAngleY;
	long GyroAVX,GyroAVY;
	long AccAngleX,AccAngleY;
	long CGAngleX,CGAngleY;
	long CGFltX,CGFltY;

	// zero offset
	int zofc;    // count timer
	long zof[10]; // offset
	*/
    ReadMPU6050(1);
    
#define Count RegFileL[0]
#define ControlCount RegFileL[1]
    Count=0;
    ControlCount=0;
    
    InitializeSerialLoop(1,SLB115200,4,"BRC_USBtest " __DATE__ " " __TIME__);
	SLReply32(63,1,0x12345678);
    SLReplyProfile(63,"start up");
    for(i=0;i<16;i++)
    {
        RegFileL[i]=0; RegFileS[i]=0;
    }

    // USB_JOYPAD ; do not change or use carefully
    // Initializer for USB interface and JoyPad handler
    USBHostInit(0);
    APP_HostHIDJoyPadInitialize();
    // /USB_JOYPAD

    // ANSEL: digital config (0)
    // ANSELA=0x0000; 
    ANSELB=0x0000; ANSELC=0x0000;
    ANSELD=0x0000; ANSELE=0x0000;
    // ANSELF=0x0000;
    ANSELG=0x0000;
    //TRIS_CCN10=TRIS_OUT; TRIS_CCN11=TRIS_OUT;
    //TRIS_CCN12=TRIS_OUT; TRIS_CCN13=TRIS_OUT;
    //TRIS_CCN14=TRIS_OUT; TRIS_CCN15=TRIS_OUT;
    //TRIS_CCN16=TRIS_OUT; TRIS_CCN17=TRIS_OUT;
    
    long clockstep=20;
    long nextclock=BRC_SysClock+clockstep;
    // state variables
    enum { SVCM_Position, SVCM_Velocity, SVCM_Lean };
    struct _SV 
    {
        long theta, thetav, pos, vel;
        long theta_ref, thetav_ref, pos_ref, vel_ref, pos_ref_0;
        long theta_0;
        long acc;
        int cm;  // controlmode 0: pos 1: vel 2: agl ; SVCM_*
        int gm[4];  // gainmask 1,0,-1
    } SV[2];
#define SVX (SV[0])
#define SVY (SV[1])
    SVX.pos=0; SVY.pos=0;
    SVX.cm=99; SVY.cm=99;
    SVX.theta_0=SVY.theta_0=0;
    for(i=0;i<4;i++) { SVX.gm[i]=SVY.gm[i]=1; }
    int YawRate=0;
    
    RegFileL[4]= 400;
    RegFileL[5]=3500;
    RegFileL[6]= 500;
    RegFileL[7]=5000;
    
    
    FSServoOnOff(1,1);
    
    while(1)
    {
        ReadMPU6050(0);

        SVX.theta = imu.CGAngleY - SVX.theta_0;  // lean to x axis dir
        SVX.thetav= imu.GyroAVY;                 // angular vel of above 
        SVY.theta =-imu.CGAngleX - SVY.theta_0;  // lean to y axis dir
        SVY.thetav=-imu.GyroAVX;                 // angular vel of above 
        if(ControlCount>0)
        {  // main control
            for(i=0;i<2;i++)
            {
                SV[i].acc=SDR8(
                        SDR12((long)(RegFileL[4])*(SV[i].gm[0])*(SV[i].theta -SV[i].theta_ref))+
                        SDR12((long)(RegFileL[5])*(SV[i].gm[1])*(SV[i].thetav-SV[i].thetav_ref))+
                        SDR8( (long)(RegFileL[6])*(SV[i].gm[2])*(SV[i].pos   -SV[i].pos_ref))+
                        SDR8( (long)(RegFileL[7])*(SV[i].gm[3])*(SV[i].vel   -SV[i].vel_ref))
                       );
                SV[i].vel+=SDR4(SV[i].acc);
                SV[i].pos+=SDR4(SV[i].vel);
            }
            if((SVX.vel>30000)||(SVX.vel<-30000)) ControlCount=0; // force stop
            if((SVY.vel>30000)||(SVY.vel<-30000)) ControlCount=0;
            int ps0=SVX.vel;
            int ps1=SVY.vel;
            int ps2=YawRate;
            int ms0=  ps0    +                         +ps2;
            int ms1=-(ps0>>1)+(int)(((long)ps1*222)>>8)+ps2;
            int ms2=-(ps0>>1)-(int)(((long)ps1*222)>>8)+ps2;
            BRC_SetMotorSpeed(0,ms0);
            BRC_SetMotorSpeed(1,ms1);
            BRC_SetMotorSpeed(2,ms2);

            // lean angle equilibrium point
            if((padState.button&0x100)==0x100)            
            for(i=0;i<2;i++)
            {
                SV[i].theta_0-=SDR4(SV[i].pos-SV[i].pos_ref);
            }
        }
        else
        {
            SVX.vel=0;
            SVY.vel=0;
            BRC_SetMotorSpeed(0,0);
            BRC_SetMotorSpeed(1,0);
            BRC_SetMotorSpeed(2,0);
        }
 
       
   }
    
}


// gain setting
// set32 4 4 400; set32 4 5 3500;set32 4 6 500;set32 4 7 5000
