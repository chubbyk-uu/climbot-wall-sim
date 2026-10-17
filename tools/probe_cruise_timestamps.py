#!/usr/bin/env python3
import argparse,ast,importlib.util,json,os,signal,subprocess,time
from pathlib import Path
parser=argparse.ArgumentParser(description='Collect controlled ROS cruise and odometry-delay traces; no Gazebo physics.')
parser.add_argument('--output',type=Path,required=True)
args_cli=parser.parse_args()
root=Path(__file__).resolve().parents[1];path=root/'src/climbot_control/test/test_coverage_executor.py'
spec=importlib.util.spec_from_file_location('cruise_probe_source',path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
func=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='generate_test_description')
call=next(n for n in ast.walk(func) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='Node')
params=ast.literal_eval(next(k.value for k in call.keywords if k.arg=='parameters'))[0]
args=[str(root/'install/climbot_control/lib/climbot_control/line_tracker_node'),'--ros-args']
for k,v in params.items():args+=['-p',f'{k}:={str(v).lower() if isinstance(v,bool) else v}']
out=args_cli.output
if out.exists():raise ValueError('output directory already exists')
out.mkdir(parents=True)
records=[]
log=Path('/tmp')/f'climbot_cruise_control_{time.time_ns()}.log'
with log.open('w') as stream:
 process=subprocess.Popen(args,stdout=stream,stderr=subprocess.STDOUT)
 try:
  for mode in ['normal','delayed_odometry']:
   case=m.TestCoverageExecutor('test_capture_heartbeat_does_not_modulate_scan_speed');trace=[];state={'dropped':0,'begun':False}
   original_pub=case._publish_odometry
   def pub(x,y,yaw,linear=0.,angular=0.):
    skip=False
    if mode=='delayed_odometry' and .8<x<1.0 and state['dropped']<6:
     state['dropped']+=1;skip=True
    trace.append({'sim_s':case.sim_time,'x':x,'command_mps':linear,'odom_held':skip})
    if not skip:original_pub(x,y,yaw,linear,angular)
   case._publish_odometry=pub
   case.setUp()
   try:
    original_send=case.client.send_goal_async
    feedback=[]
    def send(goal,*args,**kwargs):
     def receive(message):
      f=message.feedback;feedback.append({'sim_s':case.sim_time,'along_m':f.along_track_error,'schedule_lag_s':f.schedule_lag_s,'command_mps':case._current_command()[0]});case._feedback_callback(message)
     kwargs['feedback_callback']=receive
     return original_send(goal,*args,**kwargs)
    case.client.send_goal_async=send
    failure=None
    try:case.test_capture_heartbeat_does_not_modulate_scan_speed()
    except AssertionError as error:failure=str(error)
    samples=[r['command_mps'] for r in trace if .6<r['x']<2.5]
    record={'mode':mode,'original_assertion_failure':failure,'maximum_command_mps':max(samples),'dropped_odom_steps':state['dropped'],'maximum_schedule_lag_s':max(r['schedule_lag_s'] for r in feedback)}
    (out/(mode+'.json')).write_text(json.dumps({'summary':record,'trace':trace,'feedback':feedback},indent=2)+'\n');records.append(record)
   finally:case.tearDown()
 finally:
  process.send_signal(signal.SIGINT)
  process.wait(timeout=10)
(out/'execution.json').write_text(json.dumps({'controller_exit_code':process.returncode,'controller_log_name':log.name,'ros_domain_id':os.environ.get('ROS_DOMAIN_ID')},indent=2)+'\n')
(out/'parameters.json').write_text(json.dumps(params,indent=2)+'\n')
(out/'summary.json').write_text(json.dumps(records,indent=2)+'\n');print(json.dumps(records))
