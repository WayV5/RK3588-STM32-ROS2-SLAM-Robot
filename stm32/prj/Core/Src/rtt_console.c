#include "rtt_console.h"
#include "imu.h"
#include "SEGGER_RTT.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern ImuData    g_imu_data;
extern Attitude   g_attitude;
extern uint8_t    g_imu_ready;

// RTT channel assignment
#define RTT_CH_TERMINAL   0   // text commands + replies
#define RTT_CH_SCOPE_MTR  1   // motor waveform (8ch int16)
#define RTT_CH_SCOPE_IMU  2   // IMU raw waveform (8ch int16)

// Scope period (ms)
#define SCOPE_PERIOD_MS   10
#define CMD_BUF_SIZE      64

// Motor scope format name
#define JSCOPE_MTR_NAME   "JScope_I2I2I2I2I2I2I2I2"
#define JSCOPE_IMU_NAME   "JScope_I2I2I2I2I2I2I2I2"

// Telemetry view mode
typedef enum { VIEW_MOTOR = 0, VIEW_IMU, VIEW_ALL } TelemView;
static TelemView g_view = VIEW_MOTOR;

#pragma pack(push, 1)
typedef struct {
	int16_t m1_act, m1_tgt, m2_act, m2_tgt;
	int16_t m3_act, m3_tgt, m4_act, m4_tgt;
} MotorScope;
typedef struct {
	int16_t ax, ay, az;   // accel ×100  (m/s²)
	int16_t gx, gy, gz;   // gyro  ×1000 (rad/s)
	int16_t mx, my;       // mag   ×10   (µT)
} ImuScope;
#pragma pack(pop)

static uint8_t g_scope_mtr_buf[512];
static uint8_t g_scope_imu_buf[512];

void rtt_console_init(void)
{
	SEGGER_RTT_ConfigUpBuffer(RTT_CH_SCOPE_MTR, JSCOPE_MTR_NAME,
				  (char*)g_scope_mtr_buf, sizeof(g_scope_mtr_buf),
				  SEGGER_RTT_MODE_NO_BLOCK_SKIP);
	SEGGER_RTT_ConfigUpBuffer(RTT_CH_SCOPE_IMU, JSCOPE_IMU_NAME,
				  (char*)g_scope_imu_buf, sizeof(g_scope_imu_buf),
				  SEGGER_RTT_MODE_NO_BLOCK_SKIP);

	SEGGER_RTT_printf(RTT_CH_TERMINAL,
		"\n"
		"========================================\n"
		"  STM32 Debug Console\n"
		"  J-LINK RTT ready.\n"
		"========================================\n"
		"Commands:\n"
		"  m<N> <speed>   set motor target (mm/s)\n"
		"  all <speed>    set all motors\n"
		"  kp/ki/kd/kf    PID params\n"
		"  s              stop\n"
		"  status         motor status\n"
		"  imu            IMU snapshot\n"
		"  view motor|imu|all   telemetry display\n"
		"========================================\n\n");
}

// --- view command ---
static void cmd_view(const char *arg)
{
	if (strcmp(arg, "imu") == 0) {
		g_view = VIEW_IMU;
		SEGGER_RTT_printf(RTT_CH_TERMINAL, "Telemetry: IMU only\n");
	} else if (strcmp(arg, "all") == 0) {
		g_view = VIEW_ALL;
		SEGGER_RTT_printf(RTT_CH_TERMINAL, "Telemetry: motor + IMU\n");
	} else {
		g_view = VIEW_MOTOR;
		SEGGER_RTT_printf(RTT_CH_TERMINAL, "Telemetry: motor only\n");
	}
}

// --- status ---
static void cmd_status(void)
{
	SEGGER_RTT_printf(RTT_CH_TERMINAL,
		"Motor  Target(mm/s)  Actual(mm/s)  PWM   Kp    Ki    Kd     Kf\n");
	for (int i = 0; i < MOTOR_COUNT; i++) {
		Motor *m = motor_get((MotorID)i);
		SEGGER_RTT_printf(RTT_CH_TERMINAL,
			"  M%d    %6d         %6d        %4ld   %d.%02d  %d.%03d  %d.%03d  %d.%02d\n",
			i+1, m->target_speed, m->actual_speed, m->pwm_output,
			(int)m->pid.Kp, (int)(m->pid.Kp*100)%100,
			(int)m->pid.Ki, (int)(m->pid.Ki*1000)%1000,
			(int)m->pid.Kd, (int)(m->pid.Kd*1000)%1000,
			(int)m->ff_gain, (int)(m->ff_gain*100)%100);
	}
}

// --- imu snapshot (RAW chip frame) ---
static void cmd_imu(void)
{
	if (!g_imu_ready) {
		SEGGER_RTT_printf(RTT_CH_TERMINAL, "[IMU] NOT READY\n");
		return;
	}
	int ax = (int)(g_imu_data.accel[0]*100);
	int ay = (int)(g_imu_data.accel[1]*100);
	int az = (int)(g_imu_data.accel[2]*100);
	int gx = (int)(g_imu_data.gyro[0]*1000);
	int gy = (int)(g_imu_data.gyro[1]*1000);
	int gz = (int)(g_imu_data.gyro[2]*1000);
	int mx = (int)(g_imu_data.mag[0]*10);
	int my = (int)(g_imu_data.mag[1]*10);
	int mz = (int)(g_imu_data.mag[2]*10);
	int tc = (int)(g_imu_data.temp_c*10);
	int r  = (int)(g_attitude.roll*100);
	int p  = (int)(g_attitude.pitch*100);
	int y  = (int)(g_attitude.yaw*100);
	SEGGER_RTT_printf(RTT_CH_TERMINAL,
		"--- IMU (RAW chip frame) ----------\n"
		"  Accel: X=%4d.%02d Y=%4d.%02d Z=%4d.%02d\n"
		"  Gyro:  X=%4d.%03d Y=%4d.%03d Z=%4d.%03d\n"
		"  Mag:   X=%4d.%d Y=%4d.%d Z=%4d.%d  Temp=%d.%d\n"
		"  Att:   R=%4d.%02d P=%4d.%02d Y=%4d.%02d\n",
		ax/100,(ax<0?-ax:ax)%100, ay/100,(ay<0?-ay:ay)%100, az/100,(az<0?-az:az)%100,
		gx/1000,(gx<0?-gx:gx)%1000, gy/1000,(gy<0?-gy:gy)%1000, gz/1000,(gz<0?-gz:gz)%1000,
		mx/10,(mx<0?-mx:mx)%10, my/10,(my<0?-my:my)%10, mz/10,(mz<0?-mz:mz)%10,
		tc/10,(tc<0?-tc:tc)%10,
		r/100,(r<0?-r:r)%100, p/100,(p<0?-p:p)%100, y/100,(y<0?-y:y)%100);
}

// --- motor ---
static void cmd_motor(int idx, int16_t v) {
	motor_control_set_target((MotorID)idx, v);
	SEGGER_RTT_printf(RTT_CH_TERMINAL, "M%d -> %d\n", idx+1, v);
}
static void cmd_all(int16_t v) {
	for (int i=0;i<MOTOR_COUNT;i++) motor_control_set_target((MotorID)i,v);
	SEGGER_RTT_printf(RTT_CH_TERMINAL, "All -> %d\n", v);
}
static void cmd_stop(void) {
	motor_control_stop_all();
	SEGGER_RTT_printf(RTT_CH_TERMINAL, "Stopped.\n");
}
static void cmd_pid(int idx, char t, float v) {
	Motor *m = motor_get((MotorID)idx);
	if (!m) return;
	switch(t){
	case 'p':pid_set_params(&m->pid,v,m->pid.Ki,m->pid.Kd);break;
	case 'i':pid_set_params(&m->pid,m->pid.Kp,v,m->pid.Kd);break;
	case 'd':pid_set_params(&m->pid,m->pid.Kp,m->pid.Ki,v);break;
	case 'f':motor_control_set_ff_gain((MotorID)idx,v);break;
	}
	SEGGER_RTT_printf(RTT_CH_TERMINAL, "M%d K%c=%d.%02d\n",idx+1,t,(int)v,(int)(v*100)%100);
}
static int parse_idx(const char *s){ int i=atoi(s)-1; return (i>=0&&i<MOTOR_COUNT)?i:-1; }

static void process_command(const char *line)
{
	while (*line==' '||*line=='\t') line++;
	if (*line=='\0') return;
	char cmd[16],a1[16],a2[16];
	sscanf(line,"%15s%15s%15s",cmd,a1,a2);
	if (!strcmp(cmd,"status")) cmd_status();
	else if (!strcmp(cmd,"imu")) cmd_imu();
	else if (!strcmp(cmd,"view")) cmd_view(a1);
	else if (!strcmp(cmd,"s")) cmd_stop();
	else if (!strncmp(cmd,"all",3)) cmd_all((int16_t)atoi(a1));
	else if (cmd[0]=='m'&&cmd[1]>='1'&&cmd[1]<='4'&&cmd[2]=='\0')
		{ int i=parse_idx(&cmd[1]); if(i>=0)cmd_motor(i,(int16_t)atoi(a1)); }
	else if (!strncmp(cmd,"kp",2)){
		int i=(a1[0]=='a')?-1:parse_idx(a1);
		float v=(float)atof(a2);
		if(i>=0)cmd_pid(i,'p',v); else for(int j=0;j<MOTOR_COUNT;j++)cmd_pid(j,'p',v);
	}else if (!strncmp(cmd,"ki",2)){
		int i=(a1[0]=='a')?-1:parse_idx(a1);
		float v=(float)atof(a2);
		if(i>=0)cmd_pid(i,'i',v); else for(int j=0;j<MOTOR_COUNT;j++)cmd_pid(j,'i',v);
	}else if (!strncmp(cmd,"kd",2)){
		int i=(a1[0]=='a')?-1:parse_idx(a1);
		float v=(float)atof(a2);
		if(i>=0)cmd_pid(i,'d',v); else for(int j=0;j<MOTOR_COUNT;j++)cmd_pid(j,'d',v);
	}else if (!strncmp(cmd,"kf",2)){
		int i=(a1[0]=='a')?-1:parse_idx(a1);
		float v=(float)atof(a2);
		if(i>=0)cmd_pid(i,'f',v); else for(int j=0;j<MOTOR_COUNT;j++)cmd_pid(j,'f',v);
	}else SEGGER_RTT_printf(RTT_CH_TERMINAL,"?: %s\n",cmd);
}

void rtt_console_poll(void)
{
	static char buf[CMD_BUF_SIZE];
	static int pos=0;
	while (SEGGER_RTT_HasKey()) {
		int c=SEGGER_RTT_GetKey();
		if (c<0) break;
		if (c=='\r'||c=='\n') { if(pos>0){buf[pos]='\0';process_command(buf);pos=0;} }
		else if (c=='\b'||c==0x7f) { if(pos>0)pos--; }
		else if (pos<CMD_BUF_SIZE-1) { buf[pos++]=(char)c; SEGGER_RTT_Write(RTT_CH_TERMINAL,&c,1); }
	}
}

// --- scopes (10ms) ---
static void scope_motor(void) {
	MotorScope d;
	Motor *m0=motor_get(MOTOR_M1_LB),*m1=motor_get(MOTOR_M2_LF);
	Motor *m2=motor_get(MOTOR_M3_RF),*m3=motor_get(MOTOR_M4_RB);
	d.m1_act=m0->actual_speed; d.m1_tgt=m0->target_speed;
	d.m2_act=m1->actual_speed; d.m2_tgt=m1->target_speed;
	d.m3_act=m2->actual_speed; d.m3_tgt=m2->target_speed;
	d.m4_act=m3->actual_speed; d.m4_tgt=m3->target_speed;
	SEGGER_RTT_Write(RTT_CH_SCOPE_MTR,&d,sizeof(d));
}
static void scope_imu(void) {
	ImuScope d;
	d.ax=(int16_t)(g_imu_data.accel[0]*100);
	d.ay=(int16_t)(g_imu_data.accel[1]*100);
	d.az=(int16_t)(g_imu_data.accel[2]*100);
	d.gx=(int16_t)(g_imu_data.gyro[0]*1000);
	d.gy=(int16_t)(g_imu_data.gyro[1]*1000);
	d.gz=(int16_t)(g_imu_data.gyro[2]*1000);
	d.mx=(int16_t)(g_imu_data.mag[0]*10);
	d.my=(int16_t)(g_imu_data.mag[1]*10);
	SEGGER_RTT_Write(RTT_CH_SCOPE_IMU,&d,sizeof(d));
}
void rtt_scope_output(void)
{
	static uint32_t last_ms=0;
	uint32_t now=HAL_GetTick();
	if (now-last_ms<SCOPE_PERIOD_MS) return;
	last_ms=now;
	scope_motor();
	scope_imu();
}

// --- telemetry (500ms) ---
#define TELEM_PERIOD_MS 500

static void telem_motor(void) {
	Motor *m0=motor_get(MOTOR_M1_LB),*m1=motor_get(MOTOR_M2_LF);
	Motor *m2=motor_get(MOTOR_M3_RF),*m3=motor_get(MOTOR_M4_RB);
	SEGGER_RTT_printf(RTT_CH_TERMINAL,
		"M1(LB):%4d/%4d pwm=%-4ld | M2(LF):%4d/%4d pwm=%-4ld | "
		"M3(RF):%4d/%4d pwm=%-4ld | M4(RB):%4d/%4d pwm=%-4ld\r\n",
		m0->actual_speed,m0->target_speed,m0->pwm_output,
		m1->actual_speed,m1->target_speed,m1->pwm_output,
		m2->actual_speed,m2->target_speed,m2->pwm_output,
		m3->actual_speed,m3->target_speed,m3->pwm_output);
}
static void telem_imu(void) {
	int ax=(int)(g_imu_data.accel[0]*100),ay=(int)(g_imu_data.accel[1]*100),az=(int)(g_imu_data.accel[2]*100);
	int gx=(int)(g_imu_data.gyro[0]*1000),gy=(int)(g_imu_data.gyro[1]*1000),gz=(int)(g_imu_data.gyro[2]*1000);
	int mx=(int)(g_imu_data.mag[0]*10),my=(int)(g_imu_data.mag[1]*10),mz=(int)(g_imu_data.mag[2]*10);
	int r=(int)(g_attitude.roll*10),p=(int)(g_attitude.pitch*10),y=(int)(g_attitude.yaw*10);
	SEGGER_RTT_printf(RTT_CH_TERMINAL,
		"IMU A:%4d.%02d %4d.%02d %4d.%02d"
		" G:%4d.%03d %4d.%03d %4d.%03d"
		" M:%4d.%d %4d.%d %4d.%d"
		" | R=%4d.%d P=%4d.%d Y=%4d.%d\r\n",
		ax/100,(ax<0?-ax:ax)%100, ay/100,(ay<0?-ay:ay)%100, az/100,(az<0?-az:az)%100,
		gx/1000,(gx<0?-gx:gx)%1000, gy/1000,(gy<0?-gy:gy)%1000, gz/1000,(gz<0?-gz:gz)%1000,
		mx/10,(mx<0?-mx:mx)%10, my/10,(my<0?-my:my)%10, mz/10,(mz<0?-mz:mz)%10,
		r/10,(r<0?-r:r)%10, p/10,(p<0?-p:p)%10, y/10,(y<0?-y:y)%10);
}

void rtt_telemetry_output(void)
{
	static uint32_t last_ms=0;
	uint32_t now=HAL_GetTick();
	if (now-last_ms<TELEM_PERIOD_MS) return;
	last_ms=now;
	switch(g_view){
	case VIEW_IMU:  telem_imu(); break;
	case VIEW_ALL:  telem_motor(); telem_imu(); break;
	default:        telem_motor(); break;
	}
}
