/* Actual firmware parser/outer loop/IMU conversion, with only MCU registers and
 * physical I2C replies replaced. No board access and no flashing. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifndef FIRMWARE_UNDER_TEST
#define FIRMWARE_UNDER_TEST "../main.c"
#endif
#define main firmware_main
#include FIRMWARE_UNDER_TEST
#undef main
static uint8_t bus_registers[256],bus_data[12];
static int bus_ok=1,bus_data_ready=1;
int test_imu_write(uint8_t reg,uint8_t value){bus_registers[reg]=value;return bus_ok;}
int test_imu_read(uint8_t reg,uint8_t*out,unsigned size){
 if(!bus_ok)return 0;
 if(reg==0&&size==1){*out=0xea;return 1;}
 if(reg==0x1a&&size==1){*out=bus_data_ready;return 1;}
 if(reg==0x2d&&size==12){memcpy(out,bus_data,12);return 1;}
 return 0;
}
static void raw(unsigned offset,int16_t value){bus_data[offset]=(uint16_t)value>>8;bus_data[offset+1]=value;}
static void feed(unsigned kind,const uint8_t*p,unsigned n,int corrupt){
 uint8_t b[73]={0xa5,0x5a,1,(uint8_t)kind,(uint8_t)n,1,0};
 for(unsigned i=0;i<n;i++)b[i+7]=p[i];
 p16(b+7+n,crc(b+2,n+5)^(corrupt?1:0));
 for(unsigned i=0;i<n+9;i++)parse(b[i]);
}
static void reset(void){
 stop();clear_fault();hello_ok=1;configured=1;navigation_recovery=0;nonce=123;ms=0;output_pwm_cap=PWM_CAP;
 control_interval_ms=CONTROL_MS;battery_mv=12000;gd.IDR=8;ad.DR=1354;ad.SR=ADC_SR_EOC;u1.SR=USART_SR_TXE;
 const int8_t ps[4]=DEFAULT_PWM_SIGNS,es[4]=DEFAULT_ENCODER_SIGNS;
 for(unsigned i=0;i<4;i++){
  enc[i]->CNT=previous[i]=0;filtered[i]=0;actual[i]=0;
  pwm_sign[i]=ps[i];enc_sign[i]=es[i];
 }
 bus_ok=bus_data_ready=1;memset(bus_data,0,sizeof(bus_data));
 raw(4,16384);icm20948_init();
}
static void body_cmd(int vx,int wz,int corrupt){
 uint8_t p[8];p32(p,nonce);p16(p+4,vx);p16(p+6,wz);feed(BODY_COMMAND,p,8,corrupt);
}
static void manual_cmd(int value){
 uint8_t p[12];p32(p,nonce);for(unsigned i=0;i<4;i++)p16(p+4+2*i,value);feed(0x11,p,12,0);
}
static void imu_and_pid(void){
 reset();assert(imu.present&&imu.who==0xea&&!imu.calibrated);
 assert(bus_registers[1]==0x33&&bus_registers[0x14]==0x31);
 unsigned yaw_offset=6+2*IMU_YAW_AXIS;
 raw(yaw_offset,100);
 for(unsigned n=1;n<=IMU_BIAS_SAMPLES;n++)icm20948_sample(n*CONTROL_MS,actual,1);
 assert(imu.calibrated&&imu.bias_raw==100.f);
 raw(yaw_offset,1100);icm20948_sample((IMU_BIAS_SAMPLES+1)*CONTROL_MS,actual,1);
 const int gyro_signs[3]=IMU_GYRO_SIGNS;
 int converted=imu.gyro_mrad_s[IMU_YAW_AXIS]*gyro_signs[IMU_YAW_AXIS];
 assert(converted>260&&converted<270);
 assert(imu.accel_mm_s2[2]>9800&&imu.accel_mm_s2[2]<9815);
 assert(icm20948_ready(imu.sample_ms));
 assert(!icm20948_ready(imu.sample_ms+IMU_TIMEOUT_MS+1));
 int16_t wheels[4];body_reset();
 body_update(0,1,0,0,.01,wheels);
 assert(wheels[0]<0&&wheels[1]>0&&wheels[2]<0&&wheels[3]>0);
 float first=body.yaw_correction;
 body_update(0,1,0,.8,.01,wheels);assert(body.yaw_correction<first);
 body_reset();body_update(.2,0,0,0,.01,wheels);
 assert(wheels[0]>200&&wheels[0]==wheels[1]);
 body_update(0,0,0,.01,.01,wheels);
 assert(!body.yaw.integral&&!body.linear.integral&&!wheels[0]);
 for(unsigned n=0;n<2000;n++)body_update(.6,3.5,0,0,.01,wheels);
 for(unsigned i=0;i<4;i++)assert((unsigned)absi(wheels[i])<=MAX_MM_S);
 assert(body.yaw.integral<=BODY_W_CAP&&body.linear.integral<=BODY_V_CAP);
 bus_ok=0;icm20948_sample(imu.sample_ms+10,actual,1);assert(imu.failed);
 icm20948_init();assert(!icm20948_ready(0)&&!imu.initialized);
}
static void protocol_and_arbitration(void){
 reset();imu.calibrated=imu.have_sample=1;imu.sample_ms=0;
 body_cmd(200,0,0);control();
 assert(duty[0]<0&&duty[1]>0&&duty[2]>0&&duty[3]>0);
 assert(t1.CCR2==PWM_PERIOD&&t1.CCR1<PWM_PERIOD);
 assert(t1.CCR4==PWM_PERIOD&&t1.CCR3<PWM_PERIOD);
 reset();imu.calibrated=imu.have_sample=1;imu.sample_ms=0;
 body_cmd(200,1000,1);assert(!sources.nav_valid);
 uint8_t bad[8];p32(bad,nonce+1);p16(bad+4,200);p16(bad+6,1000);
 feed(BODY_COMMAND,bad,8,0);assert(!sources.nav_valid);
 body_cmd(MAX_BODY_MM_S+1,0,0);assert(!sources.nav_valid);
 body_cmd(MAX_BODY_MM_S,0,0);assert(sources.nav_valid&&sources.vx_mm_s==MAX_BODY_MM_S);
 body_cmd(200,1000,0);control();
 assert(active_source==SOURCE_BODY&&have_command&&targets[1]>targets[0]);
 manual_cmd(350);ms=10;control();
 assert(active_source==SOURCE_PWM&&duty[0]==560&&direct_mode);
 assert(t10.CCR1==1600&&t11.CCR1==1040);
 ms=20;body_cmd(100,0,0);control();assert(active_source==SOURCE_PWM);
 manual_cmd(0);ms=30;control();assert(active_source==SOURCE_BODY&&!direct_mode);
 ms=520;control();assert(!have_command&&active_source==SOURCE_STOP);
 reset();manual_cmd(350);control();ms=501;control();assert(!have_command&&!duty[0]);
 reset();imu.calibrated=imu.have_sample=1;body_cmd(200,0,0);control();
 bus_data_ready=0;ms=IMU_TIMEOUT_MS+1;control();assert(!have_command&&!fault&&!duty[0]);
 manual_cmd(350);control();assert(active_source==SOURCE_PWM&&duty[0]==560);
 uint8_t emergency[8]={0};feed(0x10,emergency,8,0);assert(!have_command&&!sources.nav_valid&&!sources.manual_valid);
 reset();imu.calibrated=imu.have_sample=1;body_cmd(200,0,0);control();
 gd.IDR=0;control();assert(!have_command&&!duty[0]);
}
static void bridge_and_saturation(void){
 reset();
 /* 四路闭环/开环前进方向及反向的实际CCR逐一相等；占空比取相同物理ticks。 */
 const int bridge_sign[4]={-1,-1,1,-1}; /* raw PWM API versus closed-loop bridge pin convention */
 const int common_cap=PWM_CAP<TEST_PWM_CAP?PWM_CAP:TEST_PWM_CAP;
 for(unsigned i=0;i<4;i++)for(int direction=-1;direction<=1;direction++){
  zero();pwm(i,direction*pwm_sign[i]*common_cap);
  uint32_t closed[8]={t10.CCR1,t11.CCR1,t9.CCR1,t9.CCR2,t1.CCR1,t1.CCR2,t1.CCR3,t1.CCR4};
  zero();test_pwm(i,direction*bridge_sign[i]*pwm_sign[i]*common_cap);
  uint32_t opened[8]={t10.CCR1,t11.CCR1,t9.CCR1,t9.CCR2,t1.CCR1,t1.CCR2,t1.CCR3,t1.CCR4};
  for(unsigned n=0;n<8;n++)assert(closed[n]==opened[n]);
 }
 reset();targets[0]=targets[2]=800;targets[1]=targets[3]=MAX_MM_S/2;
 wheel_control(.01f);
 assert(absi(duty[0])==PWM_CAP&&absi(duty[2])==PWM_CAP);
 assert(absi(duty[1])==PWM_CAP&&absi(duty[3])==PWM_CAP);
 for(unsigned i=0;i<4;i++)assert(integral[i]==0);
 /* 速度积分不能把改变方向后的微小负目标卡在正方向。 */
 zero();integral[0]=700;previous_target[0]=200;targets[0]=-20;
 targets[1]=targets[2]=targets[3]=0;wheel_control(.01f);
 assert(integral[0]<0&&duty[0]*pwm_sign[0]<0);
 reset();body_update(.4,.5,.4,0,.01,targets);
 float positive=body.yaw_correction;
 body_reset();body_update(.4,.5,.4,.8,.01,targets);
 assert(body.yaw_correction<positive); /* 实际偏航过快时IMU反馈减小转向力。 */
}
static void telemetry_and_legacy(void){
 reset();ms=40;control();telemetry();
 assert(last_tx[3]==BODY_TELEMETRY&&last_tx[4]==64&&last_tx_size==73);
 assert(previous_tx[3]==0x90&&previous_tx[4]==61);
 assert(crc(last_tx+2,last_tx_size-4)==rd16(last_tx+last_tx_size-2));
 assert(last_tx[7]==1&&last_tx[9]==0xea);
 assert(rd16(last_tx+7+56)==battery_mv);
 uint8_t req[4];p32(req,nonce);feed(BODY_INFO_REQUEST,req,4,0);
 assert(last_tx[3]==BODY_INFO_REPLY&&last_tx[4]==8&&rd32(last_tx+7)==nonce);
 feed(0x13,req,4,0);assert(last_tx[3]==0x83&&last_tx[4]==9);
 uint8_t speed[8];for(unsigned n=0;n<4;n++)p16(speed+2*n,200);
 feed(0x10,speed,8,0);control();assert(active_source==SOURCE_WHEEL&&targets[0]==200);
 ms=540;control();assert(!have_command);
 reset();uint8_t cal[9];p32(cal,nonce);cal[4]=2;p16(cal+5,CAL_PERMILLE);p16(cal+7,CAL_MS);
 feed(0x20,cal,9,0);control();assert(active_source==SOURCE_CAL&&duty[2]>0);
 ms=CAL_MS;control();assert(!have_command&&!duty[2]);
}
/* The selected main.c and its actual parser/control/PI/bridge run unchanged.
 * Trace input is synthetic, not a reconstruction of unrecorded 100Hz feedback. */
static FILE *yaw_trace;
static unsigned yaw_failures;
static void yaw_record(const char *name,unsigned step,int command_w){
 if(!yaw_trace)return;
 fprintf(yaw_trace,"%s,%u,%u,%d,%.9g,%.9g,%.9g,%.9g,%u,%u,%u",
  name,step,(unsigned)ms,command_w,body.measured_yaw,body.yaw.integral,
  body.linear.integral,body.yaw_correction,(unsigned)fault,
  (unsigned)fault_source,(unsigned)active_source);
 for(unsigned i=0;i<4;i++)fprintf(yaw_trace,",%d",targets[i]);
 for(unsigned i=0;i<4;i++)fprintf(yaw_trace,",%.9g",integral[i]);
 for(unsigned i=0;i<4;i++)fprintf(yaw_trace,",%d",duty[i]*pwm_sign[i]);
 for(unsigned i=0;i<4;i++)fprintf(yaw_trace,",%d",duty[i]);
 fprintf(yaw_trace,",%u,%u,%u,%u,%u,%u,%u,%u\n",
  t10.CCR1,t11.CCR1,t9.CCR1,t9.CCR2,t1.CCR1,t1.CCR2,t1.CCR3,t1.CCR4);
 for(unsigned i=0;i<4;i++){
  assert((unsigned)absi(targets[i])<=MAX_MM_S);
  assert((unsigned)absi(duty[i])<=PWM_CAP);
 }
}
static void yaw_ready(void){
 reset();body=(BodyControl){0};
 imu.calibrated=imu.have_sample=1;imu.sample_ms=0;
}
static void yaw_step(int vx,int wz){
 ms+=CONTROL_MS;control_interval_ms=CONTROL_MS;
 body_cmd(vx,wz,0);control();
}
static int yaw_direction_ok(int wz){
 for(unsigned i=0;i<4;i++){
  int side=(i==1||i==3)?1:-1;
  if((int32_t)targets[i]*wz*side<=0||
     (int32_t)(duty[i]*pwm_sign[i])*wz*side<=0)return 0;
 }
 return 1;
}
static void yaw_reverse(int first,int next){
 char name[64];snprintf(name,sizeof(name),"reverse_%d_%d",first,next);
 yaw_ready();
 for(unsigned n=0;n<200;n++){yaw_step(0,first);yaw_record(name,n,first);}
 assert(!fault&&body.yaw.integral*first>0);
 unsigned failures_before=yaw_failures;
 for(unsigned n=0;n<250;n++){
  yaw_step(0,next);yaw_record(name,n+200,next);
  if(n<10&&!yaw_direction_ok(next))yaw_failures++;
  if(fault)break; /* zero feedback may legitimately trip unchanged protection */
 }
 printf("ACCEPT %s first_100ms=%s\n",name,
        yaw_failures==failures_before?"PASS":"FAIL");
}
static void yaw_hold(int w){
 char name[64];snprintf(name,sizeof(name),"hold_%d",w);yaw_ready();
 for(unsigned n=0;n<400;n++){
  yaw_step(0,w);yaw_record(name,n,w);
  if(!n)assert(yaw_direction_ok(w));
  if(n==199)assert(body.yaw.integral*w>0);
  if(fault){assert(fault==4&&fault_source==5&&!have_command);break;}
 }
}
static void yaw_amplitude(int sign){
 char name[64];snprintf(name,sizeof(name),"same_amplitude_%d",sign);yaw_ready();
 for(unsigned n=0;n<300;n++){
  int w=sign*(n<100?52:(n<200?60:100));
  yaw_step(0,w);yaw_record(name,n,w);if(fault)break;
 }
}
static void yaw_periodic(int magnitude,unsigned period,int sign){
 char name[64];snprintf(name,sizeof(name),"alternate_%d_%u_%d",magnitude,period,sign);
 yaw_ready();
 for(unsigned n=0;n<800;n++){
  int w=((n/period)%2?-1:1)*sign*magnitude;
  yaw_step(0,w);yaw_record(name,n,w);if(fault)break;
 }
}
static void yaw_outer_step(float vx,float w,float measured_vx,float measured_w){
 ms+=CONTROL_MS;wheel_request_transition(vx,w);
 body_update(vx,w,measured_vx,measured_w,.01f,targets);
 for(unsigned i=0;i<4;i++)actual[i]=(int16_t)(filtered[i]=measured_vx*1000);
 wheel_control(.01f);
}
static void yaw_braking(int requested,int measured){
 char name[64];snprintf(name,sizeof(name),"brake_%d_%d",requested,measured);
 yaw_ready();
 for(unsigned n=0;n<30;n++){
  yaw_outer_step(0,requested*.001f,0,measured*.001f);
  yaw_record(name,n,requested);
  assert(measured>0?targets[1]<targets[0]:targets[1]>targets[0]);
 }
}
static void yaw_straight_and_arcs(void){
 yaw_ready();
 /* Natural linear-I history; no extreme integral is injected. */
 for(unsigned n=0;n<100;n++)yaw_outer_step(.2f,0,.1f,0);
 float linear_i=body.linear.integral;assert(linear_i>0);
 for(unsigned n=0;n<100;n++){
  yaw_outer_step(.2f,0,.2f,.02f);yaw_record("straight_w0",n,0);
  assert(targets[1]<targets[0]&&body.linear.integral==linear_i);
 }
 yaw_ready();
 for(unsigned n=0;n<400;n++){
  const int sequence[8]={1,0,-1,0,1,-1,0,0};int w=sequence[(n/10)%8];
  yaw_outer_step(.2f,w*.001f,.2f,.002f);yaw_record("micro_arc",n,w);
  assert(targets[0]>0&&targets[1]>0&&targets[2]>0&&targets[3]>0);
 }
 yaw_ready();
 for(unsigned n=0;n<300;n++){
  int w=n<100?60:(n<200?0:-60);
  yaw_outer_step(.2f,w*.001f,.2f,0);yaw_record("arc_cross_zero",n,w);
  for(unsigned i=0;i<4;i++)assert(targets[i]>0);
 }
 /* Micro pure turn: quantization is allowed, not promoted to motion. */
 yaw_ready();
 for(unsigned n=0;n<400;n++){
  int w=n<200?1:-1;yaw_step(0,w);yaw_record("micro_pure",n,w);
  for(unsigned i=0;i<4;i++)assert(!targets[i]&&!duty[i]);
  /* Reversal may reject old yaw I, but must not amplify a sub-resolution
   * request into motion or reset the unrelated PID history. */
  if(n==200)assert(body.yaw.started&&!body.linear.integral);
 }
}
static void yaw_reset_and_limits(void){
 yaw_ready();
 for(unsigned n=0;n<200;n++)yaw_step(0,60);
 yaw_step(0,-52);
 /* Subsequent wheel saturation must retain the corrected I, not old +I. */
 float corrected_i=body.yaw.integral;
 for(unsigned n=0;n<20;n++){
  body_update(.8f,-3.5f,0,0,.01f,targets);
  wheel_control(.01f);yaw_record("saturation",n,-3500);
  assert(body.yaw.integral==corrected_i);
 }
 yaw_ready();for(unsigned n=0;n<200;n++)yaw_step(0,60);
 yaw_step(0,0);yaw_record("all_zero",0,0);
 assert(!body.yaw.integral&&!body.linear.integral);
 for(unsigned i=0;i<4;i++)assert(!targets[i]&&!duty[i]&&!integral[i]);
 yaw_ready();for(unsigned n=0;n<100;n++)yaw_step(0,60);
 manual_cmd(350);ms+=CONTROL_MS;control();yaw_record("switch_pwm",0,0);
 assert(active_source==SOURCE_PWM&&!body.yaw.integral);
 manual_cmd(0);yaw_step(0,-60);yaw_record("switch_body",0,-60);
 assert(active_source==SOURCE_BODY&&yaw_direction_ok(-60));
 yaw_ready();yaw_step(0,60);bus_data_ready=0;
 for(unsigned n=0;n<=IMU_TIMEOUT_MS/CONTROL_MS;n++){
  yaw_step(0,60);yaw_record("imu_stale",n,60);
 }
 assert(!have_command&&!fault&&!body.yaw.integral);
 for(unsigned i=0;i<4;i++)assert(!duty[i]);
 yaw_ready();yaw_step(0,60);ms+=WATCHDOG_MS-1;control();
 yaw_record("watchdog_before",0,60);assert(have_command);
 ms++;control();yaw_record("watchdog_at",0,60);
 assert(!have_command&&!body.yaw.integral);
 for(unsigned i=0;i<4;i++)assert(!duty[i]);
}
static int yaw_suite(const char *output){
 yaw_trace=fopen(output,"w");assert(yaw_trace);
 fputs("case,step,ms,requested_w_mrad_s,measured_w_rad_s,yaw_I_m_s,linear_I_m_s,yaw_correction_m_s,fault,fault_source,source,target_A,target_B,target_C,target_D,inner_I_A,inner_I_B,inner_I_C,inner_I_D,duty_norm_A,duty_norm_B,duty_norm_C,duty_norm_D,duty_raw_A,duty_raw_B,duty_raw_C,duty_raw_D,CCR10_1,CCR11_1,CCR9_1,CCR9_2,CCR1_1,CCR1_2,CCR1_3,CCR1_4\n",yaw_trace);
 const int cases[8][2]={{60,-52},{-60,52},{60,-60},{-60,60},
                       {-60,54},{60,-54},{52,-52},{-52,52}};
 for(unsigned i=0;i<8;i++)yaw_reverse(cases[i][0],cases[i][1]);
 for(unsigned i=0;i<3;i++)for(int sign=-1;sign<=1;sign+=2){
  int w=(i==0?52:(i==1?60:100));yaw_hold(sign*w);
  yaw_periodic(w,50,sign);yaw_periodic(w,200,sign);
 }
 yaw_amplitude(-1);yaw_amplitude(1);
 yaw_braking(60,300);yaw_braking(-60,300);
 yaw_braking(-60,-300);yaw_braking(60,-300);
 yaw_straight_and_arcs();yaw_reset_and_limits();
 assert(!fclose(yaw_trace));yaw_trace=NULL;
 printf("YAW_REVERSAL_ACCEPTANCE failures=%u\n",yaw_failures);
 return yaw_failures?2:0;
}

/* Catch a stale opposite yaw I suppressing a real pure-turn reversal.
 * No motor/plant model: only actual firmware targets and timer commands. */
static unsigned reversal_checks, reversal_failures;
static void reversal_check(int ok,const char *name){
 reversal_checks++;
 if(!ok){reversal_failures++;printf("FAIL reversal %s\n",name);}
}
static void partial_reversal(int sign,unsigned history){
 yaw_ready();
 for(unsigned n=0;n<history;n++)yaw_step(0,sign*80);
 for(unsigned n=0;n<10;n++){
  yaw_step(0,-sign*80);
  /* Hand calculation at rest after rejecting old opposite I:
   * |wheel_ref|=(.08*.16 + .1*.08 + .2*.08*.01)*1000 -> 20 mm/s.
   * The new request must not be suppressed by its opposite history. */
  for(unsigned i=0;i<4;i++){
   int side=(i==1||i==3)?1:-1;
   reversal_check(targets[i]*(-sign)*side>=20,"partial_history_target");
   reversal_check(duty[i]*pwm_sign[i]*(-sign)*side>=80,"partial_history_output");
  }
 }
}
static void ramp_reversal(int sign){
 yaw_ready();for(unsigned n=0;n<120;n++)yaw_step(0,sign*80);
 for(unsigned n=0;n<5;n++)yaw_step(0,sign*28);
 yaw_step(0,-sign*24);
 for(unsigned i=0;i<4;i++){
  int side=(i==1||i==3)?1:-1;
  /* 24 mrad/s fresh request -> 6 mm/s, then final 80 -> >=20. */
  reversal_check(targets[i]*(-sign)*side>=6,"ramped_first_new_direction");
 }
 for(unsigned n=0;n<5;n++)yaw_step(0,-sign*24);
 yaw_step(0,-sign*80);
 for(unsigned i=0;i<4;i++){
  int side=(i==1||i==3)?1:-1;
  reversal_check(targets[i]*(-sign)*side>=20,"ramped_full_request");
 }
}
static void reversal_braking(int sign){
 yaw_ready();for(unsigned n=0;n<120;n++)yaw_step(0,sign*80);
 yaw_outer_step(0,-sign*.08f,0,sign*.30f);
 reversal_check(sign*(targets[1]-targets[0])<0,"brake_old_rotation");
 /* Also brake overspeed already in the newly requested direction.
  * The correction may legitimately oppose the target direction. */
 yaw_ready();for(unsigned n=0;n<120;n++)yaw_step(0,sign*80);
 yaw_outer_step(0,-sign*.08f,0,-sign*.30f);
 reversal_check(sign*(targets[1]-targets[0])>0,"brake_new_direction_overspeed");
}
static void retain_useful_yaw_integral(int sign){
 yaw_ready();
 for(unsigned n=0;n<100;n++)yaw_outer_step(0,sign*.08f,0,sign*.16f);
 float useful=body.yaw.integral;
 yaw_outer_step(0,-sign*.08f,0,0);
 reversal_check(body.yaw.integral*(-sign)>=useful*(-sign),
                "retain_integral_already_helping_new_direction");
}
static void no_repeated_reset(void){
 yaw_ready();for(unsigned n=0;n<120;n++)yaw_step(0,80);
 yaw_step(0,-80);
 for(unsigned n=0;n<100;n++)yaw_step(0,-80);
 reversal_check(targets[1]<=-36,"same_request_integral_can_build");
}

static void reversal_saturation_and_linear_history(int sign){
 yaw_ready();for(unsigned n=0;n<120;n++)yaw_step(0,sign*80);
 body_update(0,-sign*3.5f,0,0,.01f,targets);
 reversal_check(body.yaw.integral==0,"first_reversal_saturation_does_not_restore_old_I");
 for(unsigned i=0;i<4;i++)reversal_check((unsigned)absi(targets[i])<=MAX_MM_S,"reversal_wheel_limit");
 yaw_ready();
 for(unsigned n=0;n<100;n++)yaw_outer_step(.2f,0,.1f,0);
 for(unsigned n=0;n<120;n++)yaw_outer_step(0,sign*.08f,0,0);
 float retained=body.linear.integral;
 yaw_outer_step(0,-sign*.08f,0,0);
 reversal_check(body.linear.integral==retained&&retained==0,"pure_spin_discards_linear_history");
 reversal_check(sign*(targets[0]-targets[1])>=40,"yaw_differential_with_linear_history");
}

static int reversal_suite(void){
 const unsigned histories[]={20,60,100,120,180,200};
 for(int sign=-1;sign<=1;sign+=2){
  for(unsigned i=0;i<sizeof(histories)/sizeof(histories[0]);i++)
   partial_reversal(sign,histories[i]);
  ramp_reversal(sign);reversal_braking(sign);retain_useful_yaw_integral(sign);
  reversal_saturation_and_linear_history(sign);
 }
 no_repeated_reset();
 printf("YAW_PARTIAL_HISTORY checks=%u failures=%u\n",reversal_checks,reversal_failures);
 return reversal_failures?2:0;
}

static void output_limit_tests(void){
 for(unsigned percent=0;percent<=100;percent++){
  reset();uint8_t p[5];p32(p,nonce);p[4]=percent;feed(0x18,p,5,0);
  assert(output_pwm_cap==PWM_CAP*percent/100u);
  assert(last_tx[3]==0x85&&last_tx[4]==11&&rd32(last_tx+7)==nonce);
  assert(last_tx[11]==percent&&rd16(last_tx+12)==PWM_LIMIT_PERMILLE);
  assert(rd16(last_tx+14)==PWM_PERIOD&&rd16(last_tx+16)==output_pwm_cap);
  targets[0]=targets[2]=800;targets[1]=targets[3]=MAX_MM_S/2;wheel_control(.01f);
  for(unsigned i=0;i<4;i++)assert((unsigned)absi(duty[i])<=output_pwm_cap);
  if(percent)for(unsigned i=0;i<4;i++)assert(absi(duty[i])>=output_pwm_cap-1);
  else for(unsigned i=0;i<4;i++)assert(duty[i]==0);
  p[4]=101;feed(0x18,p,5,0);assert(output_pwm_cap==PWM_CAP*percent/100u);
  p[4]=100;p32(p,nonce+1);feed(0x18,p,5,0);assert(output_pwm_cap==PWM_CAP*percent/100u);
 }
 reset();uint8_t p[5];p32(p,nonce);p[4]=50;feed(0x18,p,5,0);
 assert(output_pwm_cap==PWM_CAP/2);
 manual_cmd(750);control();for(unsigned i=0;i<4;i++)assert((unsigned)absi(duty[i])<=PWM_CAP/2);
 ms=WATCHDOG_MS+1;control();assert(!have_command);for(unsigned i=0;i<4;i++)assert(!duty[i]);
 reset();p[4]=0;feed(0x18,p,5,0);manual_cmd(350);control();assert(!have_command);
 for(unsigned i=0;i<4;i++)assert(!duty[i]);
 reset();p[4]=100;feed(0x18,p,5,0);assert(output_pwm_cap==PWM_CAP);
 puts("OUTPUT_PERCENT_0_100_LOCAL_CAP_NONCE_WATCHDOG_PASS");
}

/* Real wheel PI and bridge, with synthetic feedback and register doubles.
 * These checks expose cross-wheel saturation coupling; they do not model torque. */
static unsigned load_checks,load_failures;
static void load_check(int ok,const char *name){
 load_checks++;
 if(!ok){if(load_failures<16)printf("FAIL wheel_load %s\n",name);load_failures++;}
}
static int close_float(float a,float b){return a-b<.0002f&&b-a<.0002f;}
static void load_independence(void){
 for(unsigned wheel=0;wheel<4;wheel++)for(int sign=-1;sign<=1;sign+=2){
  int16_t solo_duty[5];float solo_i[5];
  reset();targets[wheel]=sign*100;actual[wheel]=sign*20;filtered[wheel]=sign*20;
  for(unsigned n=0;n<5;n++){
   wheel_control(.01f);solo_duty[n]=duty[wheel];solo_i[n]=integral[wheel];
  }
  for(unsigned other=0;other<4;other++)if(other!=wheel)
   for(int other_sign=-1;other_sign<=1;other_sign+=2){
    reset();targets[wheel]=sign*100;actual[wheel]=sign*20;filtered[wheel]=sign*20;
    targets[other]=other_sign*800;
    for(unsigned n=0;n<5;n++){
     wheel_control(.01f);
     load_check(duty[wheel]==solo_duty[n],"other_saturation_cannot_reduce_duty");
     load_check(close_float(integral[wheel],solo_i[n]),"other_saturation_cannot_freeze_integral");
     load_check(duty[other]*pwm_sign[other]==other_sign*(int)PWM_CAP,"each_wheel_uses_own_cap");
     load_check(integral[other]==0&&!fault,"saturated_wheel_does_not_wind_up");
    }
   }
 }
}
static void load_unsaturated_trace(void){
 /* Hand-derived fixed-gain trajectory: target 100, KP=1, KI=4, FF=3.
  * Baseline and candidate must both retain this entirely unsaturated response. */
 const int measured[5]={0,40,120,100,200};
 const int wanted_duty[5]={404,366,285,305,201};
 const float wanted_i[5]={4,6.4f,5.6f,5.6f,1.6f};
 unsigned before=load_failures;
 for(unsigned wheel=0;wheel<4;wheel++)for(int sign=-1;sign<=1;sign+=2){
  reset();targets[wheel]=sign*100;
  for(unsigned n=0;n<5;n++){
   actual[wheel]=sign*measured[n];filtered[wheel]=sign*measured[n];wheel_control(.01f);
   load_check(duty[wheel]*pwm_sign[wheel]==sign*wanted_duty[n],"unsaturated_output_unchanged");
   load_check(close_float(integral[wheel],sign*wanted_i[n]),"unsaturated_integral_unchanged");
  }
 }
 printf("UNSATURATED_TRACE_404_366_285_305_201 %s\n",before==load_failures?"PASS":"FAIL");
}
static void load_unwind_and_reset(void){
 for(unsigned wheel=0;wheel<4;wheel++)for(int sign=-1;sign<=1;sign+=2){
  reset();targets[wheel]=previous_target[wheel]=sign*400;integral[wheel]=sign*1000;
  actual[wheel]=sign*450;filtered[wheel]=sign*450;
  for(unsigned n=0;n<5;n++){
   wheel_control(.01f);
   load_check(close_float(integral[wheel],sign*(998.f-2.f*n)),"saturated_output_allows_integral_unwind");
   load_check(duty[wheel]*pwm_sign[wheel]==sign*(int)PWM_CAP,"unwind_still_clamped_at_each_cap");
  }
  reset();targets[wheel]=previous_target[wheel]=sign*800;integral[wheel]=sign*100;
  wheel_control(.01f);
  load_check(integral[wheel]==sign*100,"outward_saturated_error_freezes_own_integral");
  reset();integral[wheel]=sign*700;previous_target[wheel]=sign*200;targets[wheel]=-sign*20;
  wheel_control(.01f);
  load_check(close_float(integral[wheel],-sign*.8f),"reversal_discards_old_integral");
  load_check(duty[wheel]*pwm_sign[wheel]==-sign*80,"reversal_output_has_new_sign");
  targets[wheel]=0;stalled[wheel]=opposite[wheel]=100;wheel_control(.01f);
  load_check(!duty[wheel]&&!integral[wheel]&&!previous_target[wheel]&&
             !stalled[wheel]&&!opposite[wheel],"zero_clears_wheel_state");
  load_check(!(t10.CCR1|t11.CCR1|t9.CCR1|t9.CCR2|t1.CCR1|t1.CCR2|t1.CCR3|t1.CCR4),
             "zero_is_coast_on_all_bridge_pins");
  reset();targets[wheel]=sign*100;filtered[wheel]=sign*1000;actual[wheel]=sign*1000;
  wheel_control(.01f);load_check(!duty[wheel],"wheel_sign_clamp_preserved");
 }
}
static void load_bridge_cap(void){
 const unsigned input1[4]={1,2,5,7},input2[4]={0,3,4,6};
 for(unsigned wheel=0;wheel<4;wheel++)for(int sign=-1;sign<=1;sign+=2){
  reset();pwm(wheel,sign*(PWM_CAP+100));
  uint32_t registers[8]={t10.CCR1,t11.CCR1,t9.CCR1,t9.CCR2,t1.CCR1,t1.CCR2,t1.CCR3,t1.CCR4};
  unsigned high=sign>0?input1[wheel]:input2[wheel];
  unsigned modulated=sign>0?input2[wheel]:input1[wheel];
  for(unsigned i=0;i<8;i++){
   uint32_t expected=i==high?PWM_PERIOD:(i==modulated?PWM_PERIOD-PWM_CAP:0);
   load_check(registers[i]==expected,"full_cap_reaches_correct_bridge_registers");
  }
  test_pwm(wheel,sign*(TEST_PWM_CAP+100));
  registers[0]=t10.CCR1;registers[1]=t11.CCR1;registers[2]=t9.CCR1;registers[3]=t9.CCR2;
  registers[4]=t1.CCR1;registers[5]=t1.CCR2;registers[6]=t1.CCR3;registers[7]=t1.CCR4;
  int difference=(int)registers[input1[wheel]]-(int)registers[input2[wheel]];
  load_check(absi(difference)==TEST_PWM_CAP,"legacy_direct_pwm_cap_unchanged");
 }
 printf("BRIDGE_CAP ticks=%u period=%u complementary=%u reported_permille=%u\n",
        PWM_CAP,PWM_PERIOD,PWM_PERIOD-PWM_CAP,PWM_LIMIT_PERMILLE);
}
static void load_protections(void){
 for(unsigned wheel=0;wheel<4;wheel++)for(int sign=-1;sign<=1;sign+=2){
  reset();targets[wheel]=sign*100;
  for(unsigned n=0;n<STALL_TIMEOUT_MS/CONTROL_MS;n++){wheel_control(.01f);load_check(!fault,"stall_allows_configured_duration");}
  wheel_control(.01f);
  load_check(fault==4&&first_fault==4&&fault_source==5,"stall_stops_after_configured_duration");
  load_check(!duty[wheel]&&!integral[wheel]&&!have_command,"stall_clears_output_and_integral");
  reset();targets[wheel]=sign*100;actual[wheel]=-sign*20;filtered[wheel]=-sign*20;
  for(unsigned n=0;n<DIRECTION_TIMEOUT_MS/CONTROL_MS;n++){wheel_control(.01f);load_check(!fault,"opposite_allows_configured_duration");}
  wheel_control(.01f);
  load_check(fault==4&&fault_source==5&&!duty[wheel],"opposite_stops_after_configured_duration");
 }
 /* Actual parser/control path retains watchdog, source arbitration, enable and IMU stops. */
 protocol_and_arbitration();
}
static void load_output_percent(void){
 for(unsigned percent=0;percent<=100;percent++){
  reset();targets[0]=100;wheel_control(.01f);
  uint8_t p[5];p32(p,nonce);p[4]=percent;feed(0x18,p,5,0);
  load_check(!duty[0]&&!integral[0]&&!targets[0],"cap_change_stops_old_motion");
  load_check(output_pwm_cap==PWM_CAP*percent/100u,"percent_applies_compiled_cap");
  load_check(last_tx[3]==0x85&&last_tx[4]==11&&rd32(last_tx+7)==nonce&&last_tx[11]==percent&&
             rd16(last_tx+12)==PWM_LIMIT_PERMILLE&&rd16(last_tx+14)==PWM_PERIOD&&
             rd16(last_tx+16)==output_pwm_cap,"percent_ack_preserves_integer_protocol");
  targets[0]=800;targets[1]=-800;wheel_control(.01f);
  load_check(absi(duty[0])==output_pwm_cap&&absi(duty[1])==output_pwm_cap,"percent_clamps_both_directions");
  p[4]=101;feed(0x18,p,5,0);
  load_check(output_pwm_cap==PWM_CAP*percent/100u,"invalid_percent_rejected");
  p[4]=100;p32(p,nonce+1);feed(0x18,p,5,0);
  load_check(output_pwm_cap==PWM_CAP*percent/100u,"wrong_nonce_rejected");
  if(!percent)load_check(!(t10.CCR1|t11.CCR1|t9.CCR1|t9.CCR2),"zero_percent_cannot_energize_bridge");
 }
 reset();uint8_t p[5];p32(p,nonce);p[4]=0;feed(0x18,p,5,0);manual_cmd(350);control();
 load_check(!have_command&&!duty[0],"zero_percent_blocks_direct_pwm");
 reset();manual_cmd(751);control();load_check(!have_command&&!duty[0],"legacy_751_permille_rejected");
}
static int wheel_load_suite(void){
 load_independence();load_unsaturated_trace();load_unwind_and_reset();load_bridge_cap();
 load_protections();load_output_percent();
 printf("WHEEL_LOAD checks=%u failures=%u cap_ticks=%u\n",load_checks,load_failures,PWM_CAP);
 return load_failures?2:0;
}

static void turn_response_tests(void){
 unsigned checks=0;
 for(int sign=-1;sign<=1;sign+=2){
  yaw_ready();
  for(unsigned n=0;n<100;n++)yaw_outer_step(sign*.2f,0,sign*.1f,0);
  assert(body.linear.integral*sign>0);
  /* 不论反馈还有多少平移，vx=0的纯转向不得携带前进补偿。 */
  yaw_outer_step(0,sign*.1f,sign*.03f,0);
  assert(body.linear.integral==0&&body.linear_correction==0);
  assert(targets[0]==-targets[1]&&targets[2]==-targets[3]);checks++;
  for(unsigned n=0;n<25;n++){
   yaw_outer_step(0,sign*.1f,sign*.03f,0);
   assert(targets[0]==-targets[1]);
  }
  /* 非零微速与正常圆弧都必须在首个控制周期执行新转向，不能沿旧I反转。 */
  const float vx_values[]={.002f,.2f,-.002f,-.2f};
  for(unsigned j=0;j<4;j++){
   float vx=vx_values[j];yaw_ready();
   for(unsigned n=0;n<100;n++)yaw_outer_step(vx,-sign*.4f,vx,0);
   assert(body.yaw.integral*sign<0);
   yaw_outer_step(vx,sign*.1f,vx,0);
   assert(sign*(targets[1]-targets[0])>0);checks++;
   assert(sign*(duty[1]*pwm_sign[1]-duty[0]*pwm_sign[0])>0);checks++;
   /* 角速度单独归零，线速度保持，不得残留上一段转向。 */
   yaw_outer_step(vx,0,vx,0);
   assert(body.yaw.integral==0&&body.yaw_correction==0);
   assert(targets[0]==targets[1]&&targets[2]==targets[3]);checks++;
   assert(duty[0]*pwm_sign[0]==duty[1]*pwm_sign[1]);checks++;
   /* 随后直行时若仍在偏转，IMU负反馈仍能给出反向纠偏。 */
   yaw_outer_step(vx,0,vx,sign*.2f);
   assert(sign*(targets[1]-targets[0])<0);checks++;
  }
  /* 所有轮端目标与PWM在全零命令的同一控制周期归零。 */
  yaw_ready();for(unsigned n=0;n<100;n++)yaw_step(0,sign*200);
  yaw_step(0,0);
  for(unsigned i=0;i<4;i++)assert(!targets[i]&&!duty[i]&&!integral[i]);
  assert(!body.linear.integral&&!body.yaw.integral);checks++;
 }
 /* 去掉转向差分I时保留共同前进I及两侧各自前后轮的负载差异。 */
 yaw_ready();body.target_vx=.2f;body.target_yaw=.4f;
 integral[0]=200;integral[1]=100;integral[2]=400;integral[3]=300;
 wheel_request_transition(.2f,0);
 assert(integral[0]==150&&integral[1]==150&&integral[2]==350&&integral[3]==350);checks++;
 body.target_yaw=0;wheel_request_transition(0,.1f);
 assert(integral[0]==-100&&integral[1]==-100&&integral[2]==100&&integral[3]==100);checks++;
 printf("TURN_START_STOP_MICRO_ARC checks=%u PASS\n",checks);
}

static void servo_tests(void){
 reset();servo_started=0;t12=(TIM_TypeDef){0};
 uint8_t pulse[2];
 uint32_t pb15_mode=gb.MODER&(3u<<30),pb15_af=gb.AFR[1]&(15u<<28);
 feed(0x30,0,0,0);assert(last_tx[3]==0x86&&!servo_started&&!t12.CR1);
 assert(!memcmp(last_tx+7,"B14P",4)&&last_tx[12]==14);
 for(unsigned i=0;i<4;i++)assert(!duty[i]);
 p16(pulse,2000);feed(0x31,pulse,2,1);assert(!servo_started);
 p16(pulse,499);feed(0x31,pulse,2,0);assert(!servo_started);
 p16(pulse,2000);feed(0x31,pulse,2,0);
 assert(t12.PSC==15&&t12.ARR==19999&&t12.CCR1==2000);
 assert(((gb.MODER>>28)&3)==2&&((gb.AFR[1]>>24)&15)==9);
 assert(last_tx[3]==0x87&&rd16(last_tx+7)==2000);
 assert(t12.CCMR1==0x68&&t12.CCER==1u&&!t12.CCR2);
 assert((gb.MODER&(3u<<30))==pb15_mode&&(gb.AFR[1]&(15u<<28))==pb15_af);
 ms=WATCHDOG_MS-1;servo_watchdog();assert(t12.CCR1==2000);
 ms++;servo_watchdog();assert(t12.CCR1==1500);
 p16(pulse,1000);feed(0x31,pulse,2,0);assert(t12.CCR1==1000);
 p16(pulse,1500);feed(0x31,pulse,2,0);assert(t12.CCR1==1500);
 const unsigned pulses[]={500,600,1000,1500,2000,2400,2500};
 for(unsigned i=0;i<sizeof(pulses)/sizeof(pulses[0]);i++){p16(pulse,pulses[i]);feed(0x31,pulse,2,0);assert(t12.CCR1==pulses[i]);}
 uint32_t last_refresh=servo_last_ms;ms++;p16(pulse,2501);feed(0x31,pulse,2,0);assert(t12.CCR1==2500&&servo_last_ms==last_refresh);
 for(unsigned i=0;i<4;i++)assert(!duty[i]);
 /* 模拟四轮闭环正在输出时插入舵机命令：轮端目标、积分、桥臂CCR不得改变。 */
 reset();for(unsigned i=0;i<4;i++)targets[i]=100+50*i;wheel_control(.01f);
 uint32_t before[8]={t10.CCR1,t11.CCR1,t9.CCR1,t9.CCR2,t1.CCR1,t1.CCR2,t1.CCR3,t1.CCR4};
 float integral_before[4];for(unsigned i=0;i<4;i++)integral_before[i]=integral[i];
 uint8_t configured_before=configured;unsigned source_before=active_source;
 p16(pulse,2000);feed(0x31,pulse,2,0);ms+=WATCHDOG_MS;servo_watchdog();
 uint32_t after[8]={t10.CCR1,t11.CCR1,t9.CCR1,t9.CCR2,t1.CCR1,t1.CCR2,t1.CCR3,t1.CCR4};
 assert(!memcmp(before,after,sizeof before));
 assert(configured==configured_before&&active_source==source_before);
 for(unsigned i=0;i<4;i++)assert(targets[i]==(int16_t)(100+50*i)&&integral[i]==integral_before[i]);
 puts("PB14_ACTIVE_FOUR_WHEELS_TIMERS_TARGETS_PI_UNCHANGED_PASS");
 puts("PB14_CAPABILITY_PWM_CRC_LIMIT_WATCHDOG_NO_WHEEL_PASS");
}
static void navigation_recovery_tests(void){
 reset();
 imu.calibrated=imu.have_sample=1;
 body_cmd(200,0,0);control();assert(have_command);
 uint8_t request[5];p32(request,nonce);request[4]=1;
 feed(0x19,request,5,0);assert(!fault&&have_command);
 request[4]=2;feed(0x19,request,5,0);request[4]=1;
 feed(0x19,request,5,1);assert(!fault&&have_command);
 feed(0x19,request,5,0);
 assert(fault==32&&!have_command&&!t10.CCR1&&!t11.CCR1);
 request[4]=0;
 feed(0x19,request,5,0);assert(fault==32);
 for(unsigned n=0;n<30;n++){ms+=CONTROL_MS;control();}
 uint8_t wrong[5];memcpy(wrong,request,sizeof wrong);p32(wrong,nonce+1);
 feed(0x19,wrong,5,0);assert(fault==32);
 feed(0x19,request,5,0);assert(!fault&&!have_command);
 control();assert(!have_command&&!duty[0]);
 body_cmd(200,0,0);control();assert(have_command);
 ms+=WATCHDOG_MS;control();assert(fault==16&&!have_command);
 for(unsigned n=0;n<30;n++){ms+=CONTROL_MS;control();}
 latch_fault(2,4,0);feed(0x19,request,5,0);assert(fault==(16|2));
 reset();imu.calibrated=imu.have_sample=1;
 body_cmd(200,0,0);control();assert(have_command);
 ms+=WATCHDOG_MS;control();assert(!fault&&!have_command);
 request[4]=2;feed(0x19,request,5,0);
 body_cmd(0,0,0);control();
 ms+=WATCHDOG_MS;control();assert(!fault&&!have_command);
 puts("NAVIGATION_LINK_LOCK_AND_RECOVERY_PASS");
}

int main(int argc,char **argv){
 if(argc==2&&!strcmp(argv[1],"--wheel-load-suite"))return wheel_load_suite();
 assert(crc((const uint8_t*)"123456789",9)==0x29b1);
 servo_tests();
 navigation_recovery_tests();
 imu_and_pid();protocol_and_arbitration();telemetry_and_legacy();bridge_and_saturation();
 puts("BODY_PID_IMU_CONVERSION_POLARITY_ARBITRATION_WATCHDOG_PROTOCOL_PASS");
 output_limit_tests();
 turn_response_tests();
 if(wheel_load_suite())return 2;
 if(argc==2&&!strcmp(argv[1],"--reversal-regression"))return reversal_suite();
 if(argc==3&&!strcmp(argv[1],"--yaw-suite"))return yaw_suite(argv[2]);
 return reversal_suite();
}
