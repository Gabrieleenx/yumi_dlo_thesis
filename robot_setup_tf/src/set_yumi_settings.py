#!/usr/bin/env python3

import rospy
import rosservice

def myhook():
      print("shutdown begin")
      rospy.wait_for_service('/yumi/rws/sm_addin/stop_egm')
      rosservice.call_service('/yumi/rws/sm_addin/stop_egm', {})

      rospy.sleep(0.5)

      rospy.wait_for_service('/yumi/rws/stop_rapid')
      rosservice.call_service('/yumi/rws/stop_rapid', {})
      rospy.sleep(0.5)

      print("shutdown commplete")



if __name__ == "__main__":
      rospy.wait_for_service('/yumi/rws/stop_rapid')
      rosservice.call_service('/yumi/rws/stop_rapid', {})
      rospy.sleep(0.4)
      rospy.wait_for_service('/yumi/rws/pp_to_main')
      rosservice.call_service('/yumi/rws/pp_to_main', {})
      rospy.sleep(0.4)
      rospy.wait_for_service('/yumi/rws/start_rapid')
      rosservice.call_service('/yumi/rws/start_rapid', {})
      rospy.sleep(0.4)


      setup_uc = dict(use_filtering=True, comm_timeout=1.0)
      xyz = dict(x=0.0, y=0.0, z=0.0)
      quat = dict(q1=1.0, q2=0.0, q3=0.0, q4=0.0)
      tframe = dict(trans=xyz, rot=quat)
      tload = dict(mass=0.001, cog=dict(x=0.0, y=0.0, z=0.001), aom=quat, ix=0.0, iy=0.0, iz=0.0)
      tool = dict(robhold=True, tframe=tframe, tload=tload)
      wobj = dict(robhold=False, ufprog=True,  ufmec='', uframe=dict(trans=xyz, rot=quat), oframe=dict(trans=xyz, rot=quat))
      correction_frame=dict(trans=xyz, rot=quat)
      sensor_frame=dict(trans=xyz, rot=quat)
      activate = dict(tool=tool, wobj=wobj, correction_frame=correction_frame, sensor_frame=sensor_frame,\
      cond_min_max=0.0, lp_filter=20.0, sample_rate=4, max_speed_deviation=90.0)
      run = dict(cond_time=60.0, ramp_in_time=1.0, offset=dict(trans=xyz, rot=quat), pos_corr_gain=0.0)
      stop = dict(ramp_out_time=1.0)
      settings = dict(allow_egm_motions=True, use_presync=False, setup_uc=setup_uc, activate=activate, run=run, stop=stop)

      rospy.wait_for_service('/yumi/rws/sm_addin/set_egm_settings')
      rosservice.call_service('/yumi/rws/sm_addin/set_egm_settings', dict(task="T_ROB_L", settings=settings))
      rospy.sleep(0.4)   
      rospy.wait_for_service('/yumi/rws/sm_addin/set_egm_settings')
      rosservice.call_service('/yumi/rws/sm_addin/set_egm_settings', dict(task="T_ROB_R", settings=settings))
      rospy.sleep(0.4)  


      rospy.wait_for_service('/yumi/rws/sm_addin/start_egm_joint')
      rosservice.call_service('/yumi/rws/sm_addin/start_egm_joint', {})
      rospy.sleep(0.4)

      rospy.wait_for_service('/yumi/egm/controller_manager/switch_controller')
      print(rosservice.call_service('/yumi/egm/controller_manager/switch_controller', dict(start_controllers=['joint_group_velocity_controller'],\
            stop_controllers=[''],\
            strictness=1,\
            start_asap=False,\
            timeout=0.0)))
      
      
      rospy.on_shutdown(myhook)

      rospy.spin()




