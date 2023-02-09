import numpy as np 
import time 
import math

from throttle import calculate_thr

import sys, os

file_path = os_path.dirname(os_path.realpath(__file__))

mass = 809 # vehicle mass in kg
u_s = 1.0  # coefficent of friction
g = 9.807  # gravity

f_max=mass*g*u_s



if __name__ == '__main__':
    try:
        with rti.open_connector(config_name="VRXParticipantLibrary::ControllerParticipant", url=file_path + "/../config/indy_ds.xml") as connector:
            # subscriber
            vehicle_state = connector.get_input("VRXSubscriber::VehicleStateReader")
            simWait = connector.get_input("VRXSubscriber::simWaitReader")

            # publishers
            acc_control = connector.get_output("VRXPublisher::AccelWriter")
            steer_control = connector.get_output("VRXPublisher::SteerWriter")
            simDone = connector.getOutput("VRXPublisher::SimDoneWriter")

            # wait for subscribers
            acc_control.wait_for_subscriptions()
            steer_control.wait_for_subscriptions()

            # read the waypoints
            waypoints = genfromtxt(file_path + "/../trajectories/test1.csv" , delimiter=',')

            # PD controller error parameters
            prev_error_steer = 0
            error_steer = 0

            v_int = 0
            steer_int = 0

            wp_count = 0
            target = waypoints[wp_count]
            print_ctr = 0
           

            #Steering Parameters
            KP_STEER = 0.09#0.155#0.175#0.125#0.05#0.045
            KD_STEER = 0.12#0.3 0.15
            KI_STEER = 0.0001*0


            #Throttle Parameters
            KP_GAS = 17
            KI_GAS = 1
            
            # Distance at which the target waypoint is updated
            DISTANCE_THRESHOLD = 35#30 #m
            
            
            # Previous time
            time_prev = time.time()-1

            # previous wp init to starting location
            previous = [-303.717277,-1857.490347] 

            
            # Read data from the input, transform it and write it into the output
            simDone.write()
            lap_number=0
            total_laps=2
            print("Waiting for data...")
            
            lap_start=0

            while True:
                simWait.read()
               
                vehicle_state.wait() # Wait for data in the input
                vehicle_state.take()
                
                for sample in vehicle_state.samples.valid_data_iter:
                    
                    vehicle_data = sample.get_dictionary()
                    rpm=(vehicle_data['EngineSpeed']*60)/(2*np.pi)
                    gear = vehicle_data['GearEngaged']

                    cur_x = vehicle_data['cdgPos_x']
                    cur_y = vehicle_data['cdgPos_y']
                    cur_yaw = np.rad2deg(vehicle_data['cdgPos_heading'])

                    v_x = vehicle_data['cdgSpeed_x']
                    v_y = vehicle_data['cdgSpeed_y']
                    v_z = vehicle_data['cdgSpeed_z']
                    sim_time=vehicle_data['TimeOfUpdate']

                # time update
                time_start = time.time()
                dt = time_start - time_prev

                dt=0.040



                ## THROTTLE CONTROL
                v = (90/25)*math.sqrt(v_x**2 + v_y**2+ v_z**2)
                v_target = 313.0 #target[2] 312
                # GAS
                v_int = v_int + (v_target - v)*dt
                
                if (v_int*KI_GAS) > 0.5*f_max:
                    v_int=(0.5*f_max/KI_GAS)
                if (v_int*KI_GAS) < -0.5*f_max:
                    v_int=(-0.5*f_max/KI_GAS)
                force_desired = KP_GAS * (v_target-v) + KI_GAS * (v_int)
                if(force_desired > f_max):
                    force_desired =  f_max
                if(force_desired < -f_max):
                    force_desired = -f_max

                correctionTh=calculate_thr(rpm, force_desired,gear)



                ## STEERING CONTROL 
                desired_yaw = np.rad2deg(math.atan2(target[1]-cur_y, target[0]-cur_x))
                error_steer = desired_yaw - cur_yaw # current_yaw is already in degrees. 
                
                
                if error_steer > 180:
                    error_steer = -360 + error_steer
                elif error_steer < -180:
                    error_steer = 360 + error_steer

               
                
                steer_der = (error_steer - prev_error_steer)#/dt
                correctionSt = KP_STEER * error_steer - KD_STEER * steer_der + KI_STEER * steer_int
                distance_error =  np.linalg.norm([target[0]-cur_x, target[1]-cur_y])

                if (distance_error < DISTANCE_THRESHOLD  and wp_count < len(waypoints)-1):
                    wp_count += 10
                    if wp_count > 2030:
                        wp_count=0
                        lap_number=lap_number+1
                        print('------Lap Time----------:', (sim_time-lap_start))
                        lap_start=sim_time
                        
                    previous = target
                    target = waypoints[wp_count]
                elif wp_count == len(waypoints)-1:
                    break

                acc_control.instance.set_number("AcceleratorAdditive", correctionTh)
                
                
                if lap_number == total_laps:
                    acc_control.instance.set_number("AcceleratorAdditive", 0.0)
                    acc_control.instance.set_number("BrakeAdditive", 400.0)
                    #acc_control.instance.set_number("BrakeMultiplicative", 1.0)
                
                
                steer_control.instance.set_number("AdditiveSteeringWheelAngle", correctionSt)
                acc_control.write()
                steer_control.write()
                simDone.write()

                prev_error_steer = error_steer
                time_prev = time_start

                # prints output for every 25 iteration

                
                
                if print_ctr % 10 == 0:
                    print("Err: ", str(round(error_steer,2)), " Str: ", str(round(correctionSt,2)), " Vel: ", str(round(v,2)), " RPM: ", str(round(rpm,2)), " Th: ", str(round(correctionTh,2)), "Gear: ", gear, "f_d:", str(round(force_desired,2)))
                    #print ("Total WP:", len(waypoints), "Current X: ", cur_x, "Current Y: ", cur_y, "Current Yaw: ", cur_yaw, "WP:", wp_count, "Lap:", lap_number)
                    print_ctr = 0
                print_ctr+=1

    except KeyboardInterrupt:
        print('Interrupted')
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)
    finally:
        pass