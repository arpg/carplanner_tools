import sys
import numpy as np
# import sophus as sp
# from scipy import stats
import matplotlib.pyplot as plt
from matplotlib import cm
# from mpl_toolkits import mplot3d
import rosbag
from collections import OrderedDict as odict

from tf_bag import BagTfTransformer

# import bagpy
# from bagpy import bagreader
# import pandas as pd

def SecondOrderBezier1D(ti,p0,p1,p2):
    # dim = len(p0)
    # assert(dim==1)
    # assert(dim==len(p1))
    # assert(dim==len(p2))
    
    # print(t)
    # print(p0)
    # print(p1)
    # print(p2)

    return (1-ti)*((1-ti)*p0+ti*p1)+ti*((1-ti)*p1+ti*p2)

def SecondOrderBezier2D(t,p0,p1,p2):
    dim = len(p0)
    assert(dim==2)
    assert(dim==len(p1))
    assert(dim==len(p2))

    # print(t)
    # print(p0)
    # print(p1)
    # print(p2)

    return [(SecondOrderBezier1D(ti,p0[0],p1[0],p2[0]), SecondOrderBezier1D(ti,p0[1],p1[1],p2[1])) for ti in t]

def PathFromWaypoints(wp_start,wp_end,num_pts=100):
    assert(len(wp_start)==len(wp_end))
    dim = len(wp_start)
    assert(dim==2)

    # a0 = wp_start[0]*wp_end[0]**2 - wp_start[0]**2*wp_end[0] + wp_start[0]*wp_end[1] - wp_end[0]*wp_start[1]
    # a0 = a0 / (wp_start[0] - wp_end[0])
    # a1 = wp_start[0]**2 - wp_end[0]**2 + wp_start[1] - wp_end[1]
    # a1 = a1 / (wp_start[0] - wp_end[0])

    t = list(np.linspace(0, 1, num_pts))
    # print(t)
    p0 = wp_start
    # print(p0)
    if wp_start[0]==wp_end[0]:
        p1 = (wp_start[0],(wp_end[1]-wp_start[1])/2+wp_start[1])
    elif wp_start[1]==wp_end[1]:
        p1 = ((wp_end[0]-wp_start[0])/2+wp_start[0],wp_start[1])
    elif np.sign(wp_end[0])==np.sign(wp_start[1]):
        p1 = (wp_end[0],wp_start[1])
    else:
        p1 = (wp_start[0],wp_end[1])
    # print(p1)
    p2 = wp_end
    # print(p2)
    b = SecondOrderBezier2D(t,p0,p1,p2)

    return b[0:-1]

def main():
    bagfile = sys.argv[1]
    if bagfile == None:
        print('Error: Invalid bag file provided.')

    odom_dict = odict()
    path_dict = odict()

    # list_of_topics = ['/tf','/gui/ctrl_path']
    bag = rosbag.Bag(bagfile)
    bag_transformer = BagTfTransformer(bag)
    for topic, msg, time in bag.read_messages(topics=['/gui/ctrl_path']):
        time_s = time.secs+time.nsecs/1e9
        path_dict[time_s] = msg
        translation, quaternion = bag_transformer.lookupTransform("map", "base_link", time)
        odom_dict[time_s] = translation
    bag.close()

    times = [list(path_dict.keys())[i] for i in range(len(odom_dict))]
    odoms = [(list(odom_dict.values())[i][0], list(odom_dict.values())[i][1], list(odom_dict.values())[i][2]) for i in range(len(odom_dict))]
    # starts = [(list(path_dict.values())[i].markers[0].points[0].x, list(path_dict.values())[i].markers[0].points[0].y, list(path_dict.values())[i].markers[0].points[0].z) for i in range(len(odom_dict))]
    # ends = [(list(path_dict.values())[i].markers[0].points[-1].x, list(path_dict.values())[i].markers[0].points[-1].y, list(path_dict.values())[i].markers[0].points[-1].z) for i in range(len(odom_dict))]
    paths = [([list(path_dict.values())[i].markers[0].points[j].x for j in range(len(list(path_dict.values())[i].markers[0].points))], [list(path_dict.values())[i].markers[0].points[j].y for j in range(len(list(path_dict.values())[i].markers[0].points))], [list(path_dict.values())[i].markers[0].points[j].z for j in range(len(list(path_dict.values())[i].markers[0].points))]) for i in range(len(odom_dict))]
    # paths = [(, , ) for i in range(len(odom_dict))]
    # paths = []
    # for ipath in range(len(odom_dict)):
    #     paths[ipath] = []
    #     for ipoint in range(len(list(path_dict.values())[ipath].markers[0].points)-1):
    #         paths[ipath][ipoint] = (list(path_dict.values())[ipath].markers[0].points[ipoint].x, list(path_dict.values())[ipath].markers[0].points[ipoint].y, list(path_dict.values())[ipath].markers[0].points[ipoint].z)
    norms = [float(list(path_dict.values())[i].markers[1].text) for i in range(len(odom_dict))]

    ## calculate derivative values
    durations = [(time-min(times)) for time in times]

    dtimes = [0]
    dtimes.extend( [durations[i+1]-durations[i] for i in range(len(durations)-1)] ) 

    dodoms = [(0,0)]
    dodoms.extend( [(odoms[i+1][0]-odoms[i][0],odoms[i+1][1]-odoms[i][1]) for i in range(len(odoms)-1)] ) 

    ddists = [0]
    ddists.extend( [(dodoms[i+1][0]**2+dodoms[i+1][1]**2)**0.5 for i in range(len(dodoms)-1)] )

    vels = [(0,0)]
    vels.extend( [(dodoms[i+1][0]/dtimes[i+1],dodoms[i+1][1]/dtimes[i+1]) for i in range(len(dodoms)-1)] )

    speeds = [(v[0]**2+v[1]**2)**0.5 for v in vels]

    control_plan_freqs = [0]
    control_plan_freqs.extend( [1/(times[i+1]-times[i]) for i in range(len(times)-1)] )

    ## crop and sort starts/norms to relevant time window for scatter plotting
    crop_index = [i for i in range(len(durations)) if durations[i]>47 and durations[i]<90]
    durations_crop = [durations[i] for i in crop_index]
    # starts_crop = [starts[i] for i in crop_index]
    # ends_crop = [ends[i] for i in crop_index]
    norms_crop = [norms[i] for i in crop_index]
    paths_crop = [paths[i] for i in crop_index]
    odoms_crop = [odoms[i] for i in crop_index]
    control_plan_freqs_crop = [control_plan_freqs[i] for i in crop_index]

    sort_index = [i[0] for i in sorted(enumerate(norms_crop), key=lambda x:x[1])] # sort by norm value
    # sort_index.reverse()
    # sort_index = list(range(len(norms_crop)))[::-1] # reverse order
    # starts_sorted = [starts_crop[i] for i in sort_index]
    # ends_sorted = [ends_crop[i] for i in sort_index]
    norms_sorted = [norms_crop[i] for i in sort_index]
    paths_sorted = [paths_crop[i] for i in sort_index]
    odoms_sorted = [odoms_crop[i] for i in sort_index]
    control_plan_freqs_sorted = [control_plan_freqs_crop[i] for i in sort_index]

    ## define waypoints for scatter plotting
    wpoffset = (0,-2)
    wps = [(0,2),(3,2),(5,0),(3,-2),(0,-2),(-3,-2),(-5,0),(-3,2)]
    wp_path = []
    [wp_path.extend(PathFromWaypoints(wps[i],wps[i+1])) for i in range(len(wps)-1)]
    wp_path.extend(PathFromWaypoints(wps[-1],wps[0]))
    wp_path.append(wp_path[0])

    # print(wp_path)

    near_path = []
    rmse = []
    for iodom in range(len(odoms)):
        min_e = 1000
        min_ei = None
        for iwp in range(len(wp_path)):
            e = ((odoms[iodom][0]-wp_path[iwp][0]-wpoffset[0])**2+(odoms[iodom][1]-wp_path[iwp][1]-wpoffset[1])**2)**0.5
            if e < min_e:
                min_e = e
                min_ei = iwp
        near_path.append((wp_path[min_ei][0]+wpoffset[0],wp_path[min_ei][1]+wpoffset[1]))
        rmse.append(min_e)
    near_path_crop = [near_path[i] for i in crop_index]
    rmse_crop = [rmse[i] for i in crop_index]

    speed_target = 2
    speed_error = [speeds[i]-speed_target for i in range(len(speeds))]
    speed_error_crop = [speed_error[i] for i in crop_index]

    ## scatter plot of trajectory and norms
    fig = plt.figure()
    norm_cmmap = plt.scatter([start[0] for start in odoms_sorted], [start[1] for start in odoms_sorted], c=norms_sorted, cmap='RdYlGn_r')
    plt.close(fig)

    fig = plt.figure()
    # norm_cmap = plt.scatter([], [], c=norms_sorted, cmap='RdYlGn_r') # 'list' object has no attribute 'shape'
    norm_cmap = plt.cm.RdYlGn_r(list(np.linspace(0,1,len(norms_sorted))))
    # norm_cmap = cm.get_cmap('RdYlGn_r', len(norms_sorted)) # LinearSegmentedColormap doesn't support indexing
    # norm_cmap = cm.ScalarMappable(cmap=cm.RdYlGn_r) # LinearSegmentedColormap doesn't support indexing
    # norm_cmap.set_array(norms_sorted) # ScalarMappable doesn't support indexing
    for ipath in range(len(paths_sorted)):
        plt.plot([point for point in paths_sorted[ipath][0]], [point for point in paths_sorted[ipath][1]], color=norm_cmap[ipath])
    plt.plot([odom[0] for odom in odoms_crop], [odom[1] for odom in odoms_crop], 'k:')
    # plt.plot([near_odom[0] for near_odom in near_path_crop], [near_odom[1] for near_odom in near_path_crop], 'k:')
    plt.plot([wp_pt[0]+wpoffset[0] for wp_pt in wp_path], [wp_pt[1]+wpoffset[1] for wp_pt in wp_path], 'k')
    plt.scatter([wp[0]+wpoffset[0] for wp in wps], [wp[1]+wpoffset[1] for wp in wps], c='k')
    # for iodom in range(len(near_path_crop)):
    #     plt.plot([near_path_crop[iodom][0],odoms_crop[iodom][0]], [near_path_crop[iodom][1],odoms_crop[iodom][1]], 'b')
    fig.colorbar(norm_cmmap)
    plt.gca().invert_xaxis()
    plt.gca().invert_yaxis()
    plt.xlabel('x (meters)')
    plt.ylabel('y (meters)')
    plt.gca().set_aspect('equal')

    ## plot of norms over time
    fig = plt.figure()
    plt.plot(durations_crop,norms_crop,'k')
    plt.plot(durations_crop,[0 for i in range(len(durations_crop))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('path norm')

    # ## plot of speeds over time
    # fig = plt.figure()
    # plt.plot(durations,speeds)
    # plt.xlabel('time (seconds)')
    # plt.ylabel('speed (m/s)')

    # ## plot odom and near_path elements
    # fig = plt.figure()
    # plt.plot(durations_crop,[odom[0] for odom in odoms_crop],'r:')
    # plt.plot(durations_crop,[near_point[0] for near_point in near_path_crop],'r')
    # plt.plot(durations_crop,[odom[1] for odom in odoms_crop],'b:')
    # plt.plot(durations_crop,[near_point[1] for near_point in near_path_crop],'b')
    # plt.xlabel('time (seconds)')
    # plt.ylabel('x ')

    ## plot of control planning frequencies
    fig = plt.figure()
    plt.plot(durations_crop,control_plan_freqs_crop,'k')
    plt.plot(durations_crop,[0 for i in range(len(durations_crop))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('control plan frequency (Hz)')

    ## plot of speed error over time
    fig = plt.figure()
    plt.plot(durations_crop,speed_error_crop,'k')
    plt.plot(durations_crop,[0 for i in range(len(durations_crop))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('velocity error (2 m/s target)')

    ## plot of rmse over time
    fig = plt.figure()
    plt.plot(durations_crop,rmse_crop,'k')
    plt.plot(durations_crop,[0 for i in range(len(durations_crop))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('RMSE (meters)')

    # ## plot of time derivative over time
    # fig = plt.figure()
    # plt.plot(odurations,odtimes,'.')
    # plt.xlabel('time (seconds)')
    # plt.ylabel('\deltat (seconds)')

    # ## plot of distance derivative over time
    # fig = plt.figure()
    # plt.plot(odurations,oddists,'.')
    # plt.xlabel('time (seconds)')
    # plt.ylabel('\deltad (meters)')

    plt.draw()
    plt.show()


if __name__=="__main__":
    main()
