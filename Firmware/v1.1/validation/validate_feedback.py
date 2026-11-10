"""实际手推日志重放+实际C车体控制器的符号响应模型，无硬件访问。"""
from pathlib import Path
import sys,json,copy,math,ctypes,subprocess,hashlib
import yaml
HERE=Path(__file__).resolve().parent;VERSION=HERE.parent;ROOT=VERSION.parents[2]
sys.path.insert(0,str(ROOT/'Test'))
from diagnose_straight import analyze
meta=json.loads((HERE/'input_manifest.json').read_text());log=Path(meta['raw_log'])
assert hashlib.sha256(log.read_bytes()).hexdigest()==meta['sha256']
rows=[json.loads(line) for line in log.read_text().splitlines()];before_cfg=rows[0]['host_config'];before_fw=rows[0]['firmware_config'];rows=rows[1:]
after_cfg=yaml.safe_load((VERSION/'host_bundle/Hardware/config.yaml').read_text());after_fw=yaml.safe_load((VERSION/'src/config.yaml').read_text())
before=analyze(rows,before_cfg,before_fw);replayed=copy.deepcopy(rows)
for row in replayed:
    if 'wheel_mm_s' in row:row['wheel_mm_s']=[-x for x in row['wheel_mm_s']]
    if 'vx' in row:row['vx']=-row['vx']
after=analyze(replayed,after_cfg,after_fw)
assert before['yaw_correlation']<-.9 and after['yaw_correlation']>.9
assert all(x>0 for x in after['forward_signed_distance_m'])
assert after['manual_yaw_degrees']==before['manual_yaw_degrees']
# 使用实际balance.c，不用另写一套PID。模型中的轮速跟踪理想化，只检查反馈符号稳定性。
lib=HERE/'body_model.so';src=VERSION/'src'
subprocess.run(['gcc','-shared','-fPIC','-O2','-I'+str(src/'build'),'-I'+str(src/'BALANCE'),str(src/'BALANCE/balance.c'),'-o',str(lib)],check=True)
class PID(ctypes.Structure):_fields_=[('integral',ctypes.c_float),('previous_error',ctypes.c_float),('started',ctypes.c_uint8)]
class Body(ctypes.Structure):_fields_=[('linear',PID),('yaw',PID)]+[(x,ctypes.c_float) for x in ['measured_vx','measured_yaw','linear_correction','yaw_correction','target_vx','target_yaw']]
dll=ctypes.CDLL(str(lib));dll.body_update.argtypes=[ctypes.c_float]*5+[ctypes.POINTER(ctypes.c_int16)]
state=Body.in_dll(dll,'body');lever=(after_fw['hardware']['track_width_m']+after_fw['hardware']['wheelbase_m'])/2
results={}
for name,polarity,command in [('old_mapping',-1,-.2),('corrected_mapping',1,.2)]:
    ctypes.memset(ctypes.addressof(state),0,ctypes.sizeof(state));wheels=(ctypes.c_int16*4)();rate=.05;heading=0.;trace=[]
    for n in range(600):
        dll.body_update(command,0.,command,rate,.01,wheels)
        diff=(wheels[0]+wheels[2]-wheels[1]-wheels[3])/4000.
        rate+=(polarity*diff/lever+.1-rate)*(.01/.15);heading+=rate*.01
        trace.append([round(n*.01,2),rate,heading,*wheels])
    (HERE/(name+'_model.json')).write_text(json.dumps(trace))
    results[name]={'final_rate_rad_s':rate,'heading_deg':heading*180/math.pi,'ideal_wheel_tracking_model':True}
assert abs(results['corrected_mapping']['final_rate_rad_s'])<.01
assert abs(results['old_mapping']['final_rate_rad_s'])>.5
report={'raw_log':str(log),'forward_before_m':before['forward_signed_distance_m'],'forward_after_m':after['forward_signed_distance_m'],'yaw_correlation_before':before['yaw_correlation'],'yaw_correlation_after':after['yaw_correlation'],'manual_yaw_degrees':after['manual_yaw_degrees'],'actual_C_controller_model':results,'pid_changed':False,'imu_sign_changed':False,'hardware_motion_performed':False,'verified':True}
(HERE/'feedback_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False,indent=2))
