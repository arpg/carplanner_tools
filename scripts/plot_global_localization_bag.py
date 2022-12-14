import sys
import math
import numpy as np
# import sophus as sp
# from scipy import stats
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.animation as animation
from numpy import average
# from mpl_toolkits import mplot3d
import rosbag
import rospy
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
    return EllipticalPathFromWaypoints(wp_start, wp_end, num_pts)

def EllipticalPathFromWaypoints(wp_start,wp_end,num_pts=100):
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

# def CircularPathFromWaypoints(wp_start,wp_end,num_pts=100):
#     assert(len(wp_start)==len(wp_end))
#     dim = len(wp_start)
#     assert(dim==2)

#     # a0 = wp_start[0]*wp_end[0]**2 - wp_start[0]**2*wp_end[0] + wp_start[0]*wp_end[1] - wp_end[0]*wp_start[1]
#     # a0 = a0 / (wp_start[0] - wp_end[0])
#     # a1 = wp_start[0]**2 - wp_end[0]**2 + wp_start[1] - wp_end[1]
#     # a1 = a1 / (wp_start[0] - wp_end[0])

#     t = list(np.linspace(0, 1, num_pts))
#     # print(t)
#     p0 = wp_start
#     # print(p0)
#     p1 = ((wp_end[0]-wp_start[0])/2+wp_start[0],(wp_end[1]-wp_start[1])/2+wp_start[1])
#     # print(p1)
#     p2 = wp_end
#     # print(p2)
#     b = SecondOrderBezier2D(t,p0,p1,p2)

#     return b[0:-1]

def DictTime(dict):
    return list(dict.keys())

def TimeToDuration(time):
    return [(instant-min(time)) for instant in time]

def DictDuration(dict):
    return TimeToDuration(DictTime(dict))

def DictValues(dict):
    return list(dict.values())

def RollingAverage(list,kernel_size=2):
    assert(len(list)>=kernel_size)
    assert(kernel_size>=1)
    avg = [sum(list[i:i+kernel_size])/kernel_size for i in range(len(list)-kernel_size)]
    return avg

def GetMsgDictFromBag(bag, topics):
    dict = odict()
    for topic, msg, time in bag.read_messages(topics):
        time_s = time.secs+time.nsecs/1e9
        dict[time_s] = msg
    return dict

def GetPathDictFromBag(bag):
    return GetMsgDictFromBag(bag, ['/gui/ctrl_path'])

def GetMapDictFromBag(bag):
    return GetMsgDictFromBag(bag, ['/input_terrain_mesh'])

def GetCpuDictFromBag(bag):
    return GetMsgDictFromBag(bag, ['/cpu_stats_publisher/cpu_usage'])

def GetTranslationDictFromBag(bag, topics_for_time, parent_id="map", child_id="base_link"):
    bag_transformer = BagTfTransformer(bag)
    dict = odict()
    for topic, msg, time in bag.read_messages(topics_for_time):
        time_s = time.secs+time.nsecs/1e9
        translation, quaternion = bag_transformer.lookupTransform(parent_id, child_id, time)
        dict[time_s] = translation
    return dict

def GetTranslationRangeDictFromBag(bag, time, parent_id="map", child_id="base_link"):
    bag_transformer = BagTfTransformer(bag)
    dict = odict()
    for t in time:
        stamp = rospy.Time(int(math.floor(t)),int(1e9*(t-math.floor(t))))
        translation, quaternion = bag_transformer.lookupTransform(parent_id, child_id, stamp)
        dict[t] = translation
    return dict

def GetPathAndTranslationDictFromBag(bag, parent_id="map", child_id="base_link"):
    bag_transformer = BagTfTransformer(bag)
    path_dict = odict()
    tran_dict = odict()
    for topic, msg, time in bag.read_messages('/gui/ctrl_path'):
        time_s = time.secs+time.nsecs/1e9
        path_dict[time_s] = msg
        translation, quaternion = bag_transformer.lookupTransform(parent_id, child_id, time)
        tran_dict[time_s] = translation
    return path_dict, tran_dict

def PlotPathDataFromBagFile(bagfile, experiment_id, crop=False, save_fig=True, extension='.png'):
    print('Plotting path data...')

    wpoffset = (0,-2,0)

    bag = rosbag.Bag(bagfile)
    path_dict, tran_dict = GetPathAndTranslationDictFromBag(bag)
    bag.close()

    time = DictTime(path_dict)
    path_msgs = DictValues(path_dict)
    tran_msgs = DictValues(path_dict)

    print('Found: '+str(len(path_msgs))+' path msgs from '+str(time[0])+' - '+str(time[-1])+' s.')

    trans = [(tran_msgs[i][0]-wpoffset[0], tran_msgs[i][1]-wpoffset[1], tran_msgs[i][2]-wpoffset[2]) for i in range(len(tran_msgs))]
    # starts = [(path_msgs[i].markers[0].points[0].x, path_msgs[i].markers[0].points[0].y, path_msgs[i].markers[0].points[0].z) for i in range(len(path_msgs))]
    # ends = [(path_msgs[i].markers[0].points[-1].x, path_msgs[i].markers[0].points[-1].y, path_msgs[i].markers[0].points[-1].z) for i in range(len(path_msgs))]
    paths = [([path_msgs[i].markers[0].points[j].x-wpoffset[0] for j in range(len(path_msgs[i].markers[0].points))], [path_msgs[i].markers[0].points[j].y-wpoffset[1] for j in range(len(path_msgs[i].markers[0].points))], [path_msgs[i].markers[0].points[j].z-wpoffset[2] for j in range(len(path_msgs[i].markers[0].points))]) for i in range(len(path_msgs))]
    norms = [float(path_msgs[i].markers[1].text) for i in range(len(path_msgs))]

    ## calculate derivative values
    duration = TimeToDuration(time)

    dtime = [0]
    dtime.extend( [duration[i+1]-duration[i] for i in range(len(duration)-1)] ) 

    dtrans = [(0,0)]
    dtrans.extend( [(trans[i+1][0]-trans[i][0],trans[i+1][1]-trans[i][1]) for i in range(len(trans)-1)] ) 

    ddists = [0]
    ddists.extend( [(dtrans[i+1][0]**2+dtrans[i+1][1]**2)**0.5 for i in range(len(dtrans)-1)] )

    vels = [(0,0)]
    vels.extend( [(dtrans[i+1][0]/dtime[i+1],dtrans[i+1][1]/dtime[i+1]) for i in range(len(dtrans)-1)] )

    speeds = [(v[0]**2+v[1]**2)**0.5 for v in vels]

    control_plan_freqs = [0]
    control_plan_freqs.extend( [1/dtime[i+1] for i in range(len(dtime)-1)] )

    ## crop and sort starts/norms to relevant time window for scatter plotting
    if crop:
        # crop_index = [i for i in range(len(durations)) if durations[i]>255]
        crop_index = [i for i in range(len(duration))]

        time = [time[i] for i in crop_index]
        duration = [duration[i] for i in crop_index]
        # starts_crop = [starts[i] for i in crop_index]
        # ends_crop = [ends[i] for i in crop_index]
        paths = [paths[i] for i in crop_index]
        trans = [trans[i] for i in crop_index]
        norms = [norms[i] for i in crop_index]
        speeds = [speeds[i] for i in crop_index]
        control_plan_freqs = [control_plan_freqs[i] for i in crop_index]

    ## print stats
    print('average norm: '+str(np.average(norms)))
    print('average speed: '+str(np.average(speeds)))
    print('average control frequency: '+str(np.average([control_plan_freqs[i] for i in range(len(control_plan_freqs)) if control_plan_freqs[i]<40])))

    sort_index = [i[0] for i in sorted(enumerate(norms), key=lambda x:x[1])] # sort by norm value
    # sort_index.reverse()
    # sort_index = list(range(len(norms_crop)))[::-1] # reverse order
    # time_sorted = [time[i] for i in sort_index]
    # starts_sorted = [starts_crop[i] for i in sort_index]
    # ends_sorted = [ends_crop[i] for i in sort_index]
    paths_sorted = [paths[i] for i in sort_index]
    trans_sorted = [trans[i] for i in sort_index]
    norms_sorted = [norms[i] for i in sort_index]
    speeds_sorted = [speeds[i] for i in sort_index]
    control_plan_freqs_sorted = [control_plan_freqs[i] for i in sort_index]

    # update whole duration after crop so that's where zero is
    # whole_duration = [whole_duration[i]-(times_crop[0]-whole_duration[0]) for i in range(len(whole_duration))]

    ## plot of path norms over duration
    fig = plt.figure()
    plt.plot(duration,norms,'k')
    plt.plot(duration,[0 for i in range(len(duration))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('path norm')
    if save_fig:
        plt.savefig('norm_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    ## plot of speed over duration
    fig = plt.figure()
    plt.plot(duration,speeds,'k')
    plt.plot(duration,[0 for i in range(len(duration))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('velocity (meters/second)')
    if save_fig:
        plt.savefig('velocity_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    ## plot of control planning frequencies over duration
    fig = plt.figure()
    kernel_size = 2
    this_duration = duration
    this_value = control_plan_freqs
    this_duration = duration[:-kernel_size]
    this_value = RollingAverage(control_plan_freqs,kernel_size)
    print(this_duration)
    print(this_value)
    plt.plot(this_duration,this_value,'k')
    plt.plot(this_duration,[0 for i in range(len(this_duration))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('control plan frequency (Hz)')
    plt.ylim([0,max([1.5*freq for freq in this_value if freq<40])])
    if save_fig:
        plt.savefig('controlfreq_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    ## define waypoints for scatter plotting
    ## ellipse
    # wpoffset = (0,-2)
    # wps = [(0,2),(3,2),(5,0),(3,-2),(0,-2),(-3,-2),(-5,0),(-3,2)] 
    ## circle
    wps = [(0,1.5),(1.06,1.06),(1.5,0),(1.06,-1.06),(0,-1.5),(-1.06,-1.06),(-1.5,0),(-1.06,1.06)]
    radius = 1.5
    wp_path = [(radius*math.sin(theta),radius*math.cos(theta)) for theta in list(np.linspace(0, 2*math.pi, 800))]

    ## find rmse
    near_path = []
    rmse = []
    for iodom in range(len(trans)):
        min_e = 1000
        min_ei = None
        for iwp in range(len(wp_path)):
            e = ((trans[iodom][0]-wp_path[iwp][0])**2+(trans[iodom][1]-wp_path[iwp][1])**2)**0.5
            if e < min_e:
                min_e = e
                min_ei = iwp
        near_path.append((wp_path[min_ei][0],wp_path[min_ei][1]))
        rmse.append(min_e)

    print('average rmse: '+str(np.average(rmse)))

    ## plot of rmse over duration
    fig = plt.figure()
    plt.plot(duration,rmse,'k')
    plt.plot(duration,[0 for i in range(len(duration))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('RMSE (meters)')
    if save_fig:
        plt.savefig('rmse_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    # ## plot odom and near_path elements over duration
    # fig = plt.figure()
    # plt.plot(durations_crop,[odom[0] for odom in odoms_crop],'r:')
    # plt.plot(durations_crop,[near_point[0] for near_point in near_path_crop],'r')
    # plt.plot(durations_crop,[odom[1] for odom in odoms_crop],'b:')
    # plt.plot(durations_crop,[near_point[1] for near_point in near_path_crop],'b')
    # plt.xlabel('time (seconds)')
    # plt.ylabel('x ')

    ## generate colormap for scatter plot of paths
    fig = plt.figure()
    norm_cmmap = plt.scatter([], [], c=norms_sorted, cmap='RdYlGn_r')
    plt.close(fig)

    ## scatter plot of trans, paths, and norms in x,y space
    fig = plt.figure()
    # ax = plt.axes(xlim=(,),ylim=(,))
    norm_cmap = plt.cm.RdYlGn_r(list(np.linspace(0,1,len(norms_sorted))))
    [plt.plot([point for point in paths_sorted[ipath][0]], [point for point in paths_sorted[ipath][1]], color=norm_cmap[ipath], zorder=80) for ipath in range(len(paths_sorted))] # [paths, dims, coords]
    plt.plot([odom[0] for odom in trans], [odom[1] for odom in trans], 'k:', zorder=90)
    # plt.plot([near_odom[0] for near_odom in near_path_crop], [near_odom[1] for near_odom in near_path_crop], 'k:')
    wp_path_plot = plt.plot([wp_pt[0] for wp_pt in wp_path], [wp_pt[1] for wp_pt in wp_path], 'k', zorder=99)
    wp_plot = plt.scatter([wp[0] for wp in wps], [wp[1] for wp in wps], c='k', zorder=100)
    fig.colorbar(norm_cmmap)
    plt.gca().invert_xaxis()
    plt.gca().invert_yaxis()
    plt.xlabel('x (meters)')
    plt.ylabel('y (meters)')
    plt.gca().set_aspect('equal')
    if save_fig:
        plt.savefig('paths_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    print('Done plotting path data.')
    return 
    
    ## setup animation of scatter plot
    # path_plots = [plt.plot([], [])[0] for _ in range(len(paths))]
    # artists = path_plots# + list(odom_plot) #things to animate
    # def init():
    #     for path_plot in path_plots:
    #         path_plot.set_data([], [])
    #     # for odom in trans:
    #     #     odom.set_height(0)
    #     return artists
    # ## create and save animation of paths
    # def animate(i):
    #     #animate lines
    #     for ipath,path_plot in enumerate(path_plots[:i]):
    #         path_plot.set_data([path[0] for path in paths[ipath]], [path[1] for path in paths[ipath]])
    #         path_plot.set_color(norm_cmap[ipath])
    #     #animate rectangles
    #     # for j,odom in enumerate(odoms_crop):
    #     #     odom.set_height(i/(j+1))
    #     return artists
    #     # [path_plots[:ipath].set_visible(True) for ipath in range(i)]
    #     # [path_plots[i+1:].set_visible(False)]
    #     # odom_plot.set_xdata([odom[0] for odom in odoms_crop[:i]])
    #     # odom_plot.set_ydata([odom[1] for odom in odoms_crop[:i]])
    #     # return odom_plot,
    #     # return path_plots[:]
    # ani = animation.FuncAnimation(fig, animate, init_func=init, frames=len(trans), interval=20, blit=True) # , fargs=()
    # # ani = animation.FuncAnimation(fig, animate, interval=20, blit=True, save_count=50)
    # writer = animation.FFMpegWriter(fps=15, metadata=dict(artist='Me'), bitrate=1800)
    # ani.save("movie.mp4", writer=writer)

def PlotGlobalLocalizationDataFromBagFile(bagfile, experiment_id):
    print('Plotting global localization data...')

    save_fig = True
    extension = '.png'
    crop = True
    offset = (0,-2,0)

    bag = rosbag.Bag(bagfile)
    cpu_dict = GetCpuDictFromBag(bag)
    cpu_time = DictTime(cpu_dict)
    dtime = 1.0/100.0
    time = [cpu_time[0]+i*dtime for i in range(int(round((cpu_time[-1]-cpu_time[0])/dtime)))]
    tran_dict = GetTranslationRangeDictFromBag(bag, time, 'map', 'base_link')
    gtran_dict = GetTranslationRangeDictFromBag(bag, time, 'map', 'base_link/vicon')
    bag.close()

    tran_msgs = DictValues(tran_dict)
    gtran_msgs = DictValues(gtran_dict)

    print('Found: '+str(len(tran_msgs))+' tf msgs from '+str(time[0])+' - '+str(time[-1])+' s.')

    trans = [(tran_msgs[i][0]-offset[0], tran_msgs[i][1]-offset[1], tran_msgs[i][2]-offset[2]) for i in range(len(tran_msgs))]
    gtrans = [(gtran_msgs[i][0]-offset[0], gtran_msgs[i][1]-offset[1], gtran_msgs[i][2]-offset[2]) for i in range(len(tran_msgs))]

    ## calculate derivative values
    duration = TimeToDuration(time)

    ## crop and sort starts/norms to relevant time window for scatter plotting
    if crop:
        crop_index = [i for i in range(len(duration)) if duration[i]>35]
        # crop_index = [i for i in range(len(duration))]

        time = [time[i] for i in crop_index]
        duration = [duration[i] for i in crop_index]
        trans = [trans[i] for i in crop_index]
        gtrans = [gtrans[i] for i in crop_index]

    dtime = [0]
    dtime.extend( [duration[i+1]-duration[i] for i in range(len(duration)-1)] ) 

    dtrans = [(0,0)]
    dtrans.extend( [(trans[i+1][0]-trans[i][0],trans[i+1][1]-trans[i][1]) for i in range(len(trans)-1)] ) 

    ddists = [0]
    ddists.extend( [(dtrans[i+1][0]**2+dtrans[i+1][1]**2)**0.5 for i in range(len(dtrans)-1)] )

    vels = [(0,0)]
    vels.extend( [(dtrans[i+1][0]/dtime[i+1],dtrans[i+1][1]/dtime[i+1]) for i in range(len(dtrans)-1)] )

    speeds = [(v[0]**2+v[1]**2)**0.5 for v in vels]

    ## print stats
    print('average speed: '+str(np.average(speeds)))

    ## plot of speed over duration
    fig = plt.figure()
    plt.plot(duration,speeds,'k')
    plt.plot(duration,[0 for i in range(len(duration))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('velocity (meters/second)')
    if save_fig:
        plt.savefig('velocity_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    # ## define waypoints for scatter plotting
    # ## ellipse
    # # wpoffset = (0,-2)
    # # wps = [(0,2),(3,2),(5,0),(3,-2),(0,-2),(-3,-2),(-5,0),(-3,2)] 
    # ## circle
    # wps = [(0,1.5),(1.06,1.06),(1.5,0),(1.06,-1.06),(0,-1.5),(-1.06,-1.06),(-1.5,0),(-1.06,1.06)]
    # radius = 1.5
    # wp_path = [(radius*math.sin(theta),radius*math.cos(theta)) for theta in list(np.linspace(0, 2*math.pi, 800))]

    ## find rmse between odometry and ground truth
    near_path = []
    lrmse = [((trans[i][0]-gtrans[i][0])**2+(trans[i][1]-gtrans[i][1])**2)**0.5 for i in range(len(time))]

    print('average localization rmse: '+str(np.average(lrmse)))

    ## plot of rmse over duration
    fig = plt.figure()
    plt.plot(duration,lrmse,'k')
    plt.plot(duration,[0 for i in range(len(duration))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('RMSE (meters)')
    if save_fig:
        plt.savefig('localization-rmse_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    ## plot odom and near_path elements over duration
    fig = plt.figure()
    plt.plot(duration,[odom[0] for odom in trans],'r:')
    plt.plot(duration,[near_point[0] for near_point in gtrans],'r')
    plt.plot(duration,[odom[1] for odom in trans],'b:')
    plt.plot(duration,[near_point[1] for near_point in gtrans],'b')
    plt.xlabel('time (seconds)')
    plt.ylabel('x (red), y (blue)')
    if save_fig:
        plt.savefig('xy_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    ## scatter plot of trans, paths, and norms in x,y space
    fig = plt.figure()
    # ax = plt.axes(xlim=(,),ylim=(,))
    plt.plot([odom[0] for odom in trans], [odom[1] for odom in trans], 'k', zorder=90)
    plt.plot([near_odom[0] for near_odom in gtrans], [near_odom[1] for near_odom in gtrans], 'k:')
    # wp_path_plot = plt.plot([wp_pt[0] for wp_pt in wp_path], [wp_pt[1] for wp_pt in wp_path], 'k', zorder=99)
    # wp_plot = plt.scatter([wp[0] for wp in wps], [wp[1] for wp in wps], c='k', zorder=100)
    plt.gca().invert_xaxis()
    plt.gca().invert_yaxis()
    plt.xlabel('x (meters)')
    plt.ylabel('y (meters)')
    plt.gca().set_aspect('equal')
    if save_fig:
        plt.savefig('odoms_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    print('Done plotting tf data.')
    return 

def PlotCpuDataFromBagFile(bagfile, experiment_id, crop=False, save_fig=True, extension='.png'):
    print('Plotting cpu data...')

    bag = rosbag.Bag(bagfile)
    dict = GetCpuDictFromBag(bag)
    bag.close()

    time = DictTime(dict)
    cpu = [msg.data for msg in DictValues(dict)]

    print('Found: '+str(len(cpu))+' cpu msgs from '+str(time[0])+' - '+str(time[-1])+' s.')

    ## calculate derivative values
    duration = TimeToDuration(time)

    ## crop and sort starts/norms to relevant time window for scatter plotting
    if crop:
        # crop_index = [i for i in range(len(durations)) if durations[i]>255]
        crop_index = [i for i in range(len(duration))]

        time = [time[i] for i in crop_index]
        duration = [duration[i] for i in crop_index]
        cpu = [cpu[i] for i in crop_index]

    ## remove nans
    duration = [duration[i] for i in range(len(duration)) if not math.isnan(cpu[i])]
    cpu = [cpu[i] for i in range(len(duration)) if not math.isnan(cpu[i])]

    print('average cpu usage: '+str(np.average(cpu)))

    ## plot of cpu usage over duration
    fig = plt.figure()
    kernel_size = 200
    this_duration = [duration[i] for i in range(len(duration)) if not math.isnan(cpu[i])]
    this_value = [cpu[i] for i in range(len(cpu)) if not math.isnan(cpu[i])]
    this_duration = this_duration[:-kernel_size]
    this_value = RollingAverage(this_value,kernel_size)
    plt.plot(this_duration,this_value,'k')
    plt.plot(this_duration,[0 for i in range(len(this_duration))],'k:')
    plt.plot(this_duration,[100 for i in range(len(this_duration))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('cpu usage (%)')
    if save_fig:
        plt.savefig('cpu_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    print('Done plotting cpu data.')
    return 

def PlotMapDataFromBagFile(bagfile, experiment_id, crop=False, save_fig=True, extension='.png'):
    print('Plotting map data...')

    bag = rosbag.Bag(bagfile)
    dict = GetMapDictFromBag(bag)
    bag.close()

    time = DictTime(dict)
    map_lens = [len(msg.mesh.triangles) for msg in DictValues(dict)]

    print('Found: '+str(len(map_lens))+' map msgs from '+str(time[0])+' - '+str(time[-1])+' s.')

    ## calculate derivative values
    duration = TimeToDuration(time)

    ## crop and sort starts/norms to relevant time window for scatter plotting
    if crop:
        # crop_index = [i for i in range(len(durations)) if durations[i]>255]
        crop_index = [i for i in range(len(duration))]

        time = [time[i] for i in crop_index]
        duration = [duration[i] for i in crop_index]
        map_lens = [map_lens[i] for i in crop_index]

    print('average map size: '+str(np.average(map_lens)))

    ## plot of number of mesh triangles over duration of whole experiment
    fig = plt.figure()
    plt.plot(duration,map_lens,'k')
    plt.plot(duration,[0 for i in range(len(map_lens))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('map size (number of triangles)')
    if save_fig:
        plt.savefig('map_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    print('Done plotting map data.')
    return 

def main():
    bagfile = sys.argv[1]
    if bagfile == None:
        print('Error: Invalid bag file provided.')
    experiment_id = sys.argv[2]
    if experiment_id == None:
        print('Error: Invalid experiment id provided.')

    print('Processing Experiment '+experiment_id+'...')

    # try:
    #     PlotPathDataFromBagFile(bagfile, experiment_id, crop, save_fig, extension)
    # except Exception as e:
    #     print('Could not plot path data -- '+str(e))

    # try:
    PlotGlobalLocalizationDataFromBagFile(bagfile, experiment_id)
    # except Exception as e:
    #     print('Could not plot global localization data -- '+str(e))

    # try:
    #     PlotCpuDataFromBagFile(bagfile, experiment_id, crop, save_fig, extension)
    # except Exception as e:
    #     print('Could not cpu path data -- '+str(e))

    # try:
    #     PlotMapDataFromBagFile(bagfile, experiment_id, crop, save_fig, extension)
    # except Exception as e:
    #     print('Could not map path data -- '+str(e))

    print('Done processing Experiment '+experiment_id+'.')
    return

    odom_dict = odict()
    path_dict = odict()
    cpu_dict = odict()
    map_dict = odict()

    # list_of_topics = ['/tf','/gui/ctrl_path']
    bag = rosbag.Bag(bagfile)
    bag_transformer = BagTfTransformer(bag)
    for topic, msg, time in bag.read_messages(topics=['/gui/ctrl_path','/cpu_stats_publisher/cpu_usage','/input_terrain_mesh']):
        time_s = time.secs+time.nsecs/1e9
        if topic=='/gui/ctrl_path':
            path_dict[time_s] = msg
            translation, quaternion = bag_transformer.lookupTransform("map", "base_link", time)
            odom_dict[time_s] = translation
        elif topic=='/cpu_stats_publisher/cpu_usage':
            cpu_dict[time_s] = msg.data
        elif topic=='/input_terrain_mesh':
            map_dict[time_s] = len(msg.mesh.triangles)
    bag.close()

    whole_time = DictTime(path_dict)+DictTime(cpu_dict)+DictTime(map_dict)
    whole_time = list(set(whole_time))
    whole_time.sort()
    print('Time range: '+str(whole_time[0])+' - '+str(whole_time[-1])+' s.')
    # whole_time_range = (min(DictTime(path_dict)+DictTime(cpu_dict)+DictTime(map_dict)), max(DictTime(path_dict)+DictTime(cpu_dict)+DictTime(map_dict)))
    # whole_time_zero = DictTime(path_dict)[0]
    # def ValuePaddedForTimeRange(values, values_time, pad_time):
    #     pad_values = [float('NAN')]*len(pad_time)
    #     for value in pad_values:
            
    #     new_values = [values[i] for i in range(len(values)) if values_time[i]]
    #     return [value[i]-(times_crop[0]-this_whole_time[0]) for i in range(len(this_whole_duration))]
    # this_whole_time = DictTime(cpu_dict)
    # this_whole_duration = TimeToDuration(this_whole_time)
    # this_whole_duration = [this_whole_duration[i]-(times_crop[0]-this_whole_time[0]) for i in range(len(this_whole_duration))] 
    whole_duration = TimeToDuration(whole_time)
    print('Duration: '+str(whole_duration[-1])+' s.')

    print('Found: '+str(len(path_dict))+' path msgs from '+str(DictTime(path_dict)[0])+' - '+str(DictTime(path_dict)[-1])+' s.')
    print('Found: '+str(len(odom_dict))+' odom msgs from '+str(DictTime(odom_dict)[0])+' - '+str(DictTime(odom_dict)[-1])+' s.')
    print('Found: '+str(len(cpu_dict))+' cpu msgs from '+str(DictTime(cpu_dict)[0])+' - '+str(DictTime(cpu_dict)[-1])+' s.')
    print('Found: '+str(len(map_dict))+' map msgs from '+str(DictTime(map_dict)[0])+' - '+str(DictTime(map_dict)[-1])+' s.')

    times = [list(path_dict.keys())[i] for i in range(len(odom_dict))]
    odoms = [(list(odom_dict.values())[i][0]-wpoffset[0], list(odom_dict.values())[i][1]-wpoffset[1], list(odom_dict.values())[i][2]-wpoffset[2]) for i in range(len(odom_dict))]
    # starts = [(list(path_dict.values())[i].markers[0].points[0].x, list(path_dict.values())[i].markers[0].points[0].y, list(path_dict.values())[i].markers[0].points[0].z) for i in range(len(odom_dict))]
    # ends = [(list(path_dict.values())[i].markers[0].points[-1].x, list(path_dict.values())[i].markers[0].points[-1].y, list(path_dict.values())[i].markers[0].points[-1].z) for i in range(len(odom_dict))]
    paths = [([list(path_dict.values())[i].markers[0].points[j].x-wpoffset[0] for j in range(len(list(path_dict.values())[i].markers[0].points))], [list(path_dict.values())[i].markers[0].points[j].y-wpoffset[1] for j in range(len(list(path_dict.values())[i].markers[0].points))], [list(path_dict.values())[i].markers[0].points[j].z-wpoffset[2] for j in range(len(list(path_dict.values())[i].markers[0].points))]) for i in range(len(odom_dict))]
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

    ## plot of control planning frequencies over duration of whole experiment
    fig = plt.figure()
    kernel_size = 2
    this_duration = durations
    this_value = control_plan_freqs
    # this_duration = durations[:-kernel_size]
    # this_value = RollingAverage(control_plan_freqs,kernel_size)
    print(this_duration)
    print(this_value)
    plt.plot(this_duration,this_value,'k')
    plt.plot(this_duration,[0 for i in range(len(this_duration))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('control plan frequency (Hz)')
    plt.ylim([0,max([1.5*freq for freq in this_value if freq<40])])
    if save_fig:
        plt.savefig('controlfreq-whole_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    ## plot of cpu usage over duration of whole experiment
    fig = plt.figure()
    dict = cpu_dict
    kernel_size = 200
    this_time = DictTime(dict)
    this_duration = TimeToDuration(this_time)
    this_value = DictValues(dict)
    this_duration = [this_duration[i] for i in range(len(this_duration)) if not math.isnan(this_value[i])]
    this_value = [this_value[i] for i in range(len(this_value)) if not math.isnan(this_value[i])]
    this_duration = this_duration[:-kernel_size]
    this_value = RollingAverage(this_value,kernel_size)
    plt.plot(this_duration,this_value,'k')
    plt.plot(this_duration,[0 for i in range(len(this_duration))],'k:')
    plt.plot(this_duration,[100 for i in range(len(this_duration))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('cpu usage (%)')
    if save_fig:
        plt.savefig('cpu-whole_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    ## plot of number of mesh triangles over duration of whole experiment
    fig = plt.figure()
    dict = map_dict
    this_time = DictTime(dict)
    this_duration = TimeToDuration(this_time)
    this_value = DictValues(dict)
    plt.plot(this_duration,this_value,'k')
    plt.plot(this_duration,[0 for i in range(len(this_duration))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('map size (number of triangles)')
    if save_fig:
        plt.savefig('map-whole_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    ## crop and sort starts/norms to relevant time window for scatter plotting
    # crop_index = [i for i in range(len(durations)) if durations[i]>255]
    crop_index = [i for i in range(len(durations))]
    times_crop = [times[i] for i in crop_index]
    durations_crop = [durations[i] for i in crop_index]
    # starts_crop = [starts[i] for i in crop_index]
    # ends_crop = [ends[i] for i in crop_index]
    norms_crop = [norms[i] for i in crop_index]
    paths_crop = [paths[i] for i in crop_index]
    odoms_crop = [odoms[i] for i in crop_index]
    speeds_crop = [speeds[i] for i in crop_index]
    control_plan_freqs_crop = [control_plan_freqs[i] for i in crop_index]

    sort_index = [i[0] for i in sorted(enumerate(norms_crop), key=lambda x:x[1])] # sort by norm value
    times_sorted = [times_crop[i] for i in sort_index]
    # sort_index.reverse()
    # sort_index = list(range(len(norms_crop)))[::-1] # reverse order
    # starts_sorted = [starts_crop[i] for i in sort_index]
    # ends_sorted = [ends_crop[i] for i in sort_index]
    norms_sorted = [norms_crop[i] for i in sort_index]
    paths_sorted = [paths_crop[i] for i in sort_index]
    odoms_sorted = [odoms_crop[i] for i in sort_index]
    speeds_sorted = [speeds_crop[i] for i in sort_index]
    control_plan_freqs_sorted = [control_plan_freqs_crop[i] for i in sort_index]

    # update whole duration after crop so that's where zero is
    # whole_duration = [whole_duration[i]-(times_crop[0]-whole_duration[0]) for i in range(len(whole_duration))]

    ## print stats
    print('average norm: '+str(np.average(norms_crop)))
    print('average speed: '+str(np.average(speeds_crop)))
    print('average control frequency: '+str(np.average([control_plan_freqs_crop[i] for i in range(len(control_plan_freqs_crop)) if control_plan_freqs_crop[i]<40])))

    ## plot of control planning frequencies over duration of control
    fig = plt.figure()
    kernel_size = 2
    this_duration = durations_crop[:-kernel_size]
    this_value = RollingAverage(control_plan_freqs_crop,kernel_size)
    plt.plot(this_duration,this_value,'k')
    plt.plot(this_duration,[0 for i in range(len(this_duration))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('control plan frequency (Hz)')
    plt.ylim([0,max([1.5*freq for freq in this_value if freq<40])])
    if save_fig:
        plt.savefig('controlfreq_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    ## plot of cpu usage over duration of control
    fig = plt.figure()
    dict = cpu_dict
    kernel_size = 200
    this_time = DictTime(dict)
    # print('cpu time range: '+str(this_time[0])+' to '+str(this_time[-1]))
    this_crop = [i for i in range(len(this_time)) if this_time[i]>=times_crop[0] and this_time[i]<=times_crop[-1]]
    # this_time = [this_time[i] for i in range(len(this_time)) if this_time[i]>=times_crop[0] and this_time[i]<=times_crop[-1]]
    this_time = [this_time[i] for i in this_crop]
    this_duration = TimeToDuration(this_time)
    this_value = DictValues(dict)
    this_value = [this_value[i] for i in this_crop]
    this_time = [this_time[i] for i in range(len(this_time)) if not math.isnan(this_value[i])]
    # this_duration = [this_duration[i]-this_duration[0] for i in range(len(this_duration)) if not math.isnan(this_value[i]) and this_time[i]>=times_crop[0] and this_time[i]<=times_crop[-1]]
    this_duration = [this_duration[i] for i in range(len(this_duration)) if not math.isnan(this_value[i])]
    # this_value = [this_value[i] for i in range(len(this_value)) if not math.isnan(this_value[i]) and this_time[i]>=times_crop[0] and this_time[i]<=times_crop[-1]]
    this_value = [this_value[i] for i in range(len(this_value)) if not math.isnan(this_value[i])]
    # this_duration = [this_duration[i] for i in range(len(this_duration))]
    print('average cpu usage: '+str(np.average(this_value)))
    this_time = this_time[:-kernel_size]
    this_duration = this_duration[:-kernel_size]
    this_value = RollingAverage(this_value,kernel_size)
    plt.plot(this_duration,this_value,'k')
    plt.plot(this_duration,[0 for i in range(len(this_duration))],'k:')
    plt.plot(this_duration,[100 for i in range(len(this_duration))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('cpu usage (%)')
    if save_fig:
        plt.savefig('cpu_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    ## plot of number of mesh triangles over duration of control
    fig = plt.figure()
    dict = map_dict
    this_time = DictTime(dict)
    this_crop = [i for i in range(len(this_time)) if this_time[i]>=times_crop[0] and this_time[i]<=times_crop[-1]]
    this_time = [this_time[i] for i in this_crop]
    this_duration = TimeToDuration(this_time)
    this_value = DictValues(dict)
    this_value = [this_value[i] for i in this_crop]
    plt.plot(this_duration,this_value,'k')
    plt.plot(this_duration,[0 for i in range(len(this_duration))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('map size (number of triangles)')
    if save_fig:
        plt.savefig('map_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    ## define waypoints for scatter plotting
    ## ellipse
    # wpoffset = (0,-2)
    # wps = [(0,2),(3,2),(5,0),(3,-2),(0,-2),(-3,-2),(-5,0),(-3,2)] 
    ## circle
    wps = [(0,1.5),(1.06,1.06),(1.5,0),(1.06,-1.06),(0,-1.5),(-1.06,-1.06),(-1.5,0),(-1.06,1.06)]
    radius = 1.5
    wp_path = [(radius*math.sin(theta),radius*math.cos(theta)) for theta in list(np.linspace(0, 2*math.pi, 800))]
    # [wp_path.extend(CircularPathFromWaypoints(wps[i],wps[i+1])) for i in range(len(wps)-1)]
    # wp_path.extend(CircularPathFromWaypoints(wps[-1],wps[0]))
    # wp_path.append(wp_path[0])

    # print(wp_path)

    near_path = []
    rmse = []
    for iodom in range(len(odoms)):
        min_e = 1000
        min_ei = None
        for iwp in range(len(wp_path)):
            e = ((odoms[iodom][0]-wp_path[iwp][0])**2+(odoms[iodom][1]-wp_path[iwp][1])**2)**0.5
            if e < min_e:
                min_e = e
                min_ei = iwp
        near_path.append((wp_path[min_ei][0],wp_path[min_ei][1]))
        rmse.append(min_e)
    near_path_crop = [near_path[i] for i in crop_index]
    rmse_crop = [rmse[i] for i in crop_index]

    print('average rmse: '+str(np.average(rmse_crop)))

    speed_target = 2
    speed_error = [speeds[i]-speed_target for i in range(len(speeds))]
    speed_error_crop = [speed_error[i] for i in crop_index]

    ## scatter plot of trajectory and norms over duration of control
    fig = plt.figure()
    norm_cmmap = plt.scatter([start[0] for start in odoms_sorted], [start[1] for start in odoms_sorted], c=norms_sorted, cmap='RdYlGn_r')
    plt.close(fig)

    fig = plt.figure()
    # ax = plt.axes(xlim=(,),ylim=(,))
    norm_cmap = plt.cm.RdYlGn_r(list(np.linspace(0,1,len(norms_sorted))))
    [plt.plot([point for point in paths_sorted[ipath][0]], [point for point in paths_sorted[ipath][1]], color=norm_cmap[ipath], zorder=80) for ipath in range(len(paths_sorted))] # [paths, dims, coords]
    plt.plot([odom[0] for odom in odoms_crop], [odom[1] for odom in odoms_crop], 'k:', zorder=90)
    # plt.plot([near_odom[0] for near_odom in near_path_crop], [near_odom[1] for near_odom in near_path_crop], 'k:')
    wp_path_plot = plt.plot([wp_pt[0] for wp_pt in wp_path], [wp_pt[1] for wp_pt in wp_path], 'k', zorder=99)
    wp_plot = plt.scatter([wp[0] for wp in wps], [wp[1] for wp in wps], c='k', zorder=100)
    # for iodom in range(len(near_path_crop)):
    #     plt.plot([near_path_crop[iodom][0],odoms_crop[iodom][0]], [near_path_crop[iodom][1],odoms_crop[iodom][1]], 'b')
    fig.colorbar(norm_cmmap)
    plt.gca().invert_xaxis()
    plt.gca().invert_yaxis()
    plt.xlabel('x (meters)')
    plt.ylabel('y (meters)')
    plt.gca().set_aspect('equal')
    if save_fig:
        plt.savefig('paths_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)
    
    path_plots = [plt.plot([], [])[0] for _ in range(len(paths_crop))]
    artists = path_plots# + list(odom_plot) #things to animate
    def init():
        for path_plot in path_plots:
            path_plot.set_data([], [])
        # for odom in odoms_crop:
        #     odom.set_height(0)
        return artists
    ## create and save animation of paths
    def animate(i):
        #animate lines
        for ipath,path_plot in enumerate(path_plots[:i]):
            path_plot.set_data([path[0] for path in paths_crop[ipath]], [path[1] for path in paths_crop[ipath]])
            path_plot.set_color(norm_cmap[ipath])
        #animate rectangles
        # for j,odom in enumerate(odoms_crop):
        #     odom.set_height(i/(j+1))
        return artists
        # [path_plots[:ipath].set_visible(True) for ipath in range(i)]
        # [path_plots[i+1:].set_visible(False)]
        # odom_plot.set_xdata([odom[0] for odom in odoms_crop[:i]])
        # odom_plot.set_ydata([odom[1] for odom in odoms_crop[:i]])
        # return odom_plot,
        # return path_plots[:]
    ani = animation.FuncAnimation(fig, animate, init_func=init, frames=len(odoms_crop), interval=20, blit=True) # , fargs=()
    # ani = animation.FuncAnimation(fig, animate, interval=20, blit=True, save_count=50)
    writer = animation.FFMpegWriter(fps=15, metadata=dict(artist='Me'), bitrate=1800)
    ani.save("movie.mp4", writer=writer)

    ## plot of norms over duration of control
    fig = plt.figure()
    plt.plot(durations_crop,norms_crop,'k')
    plt.plot(durations_crop,[0 for i in range(len(durations_crop))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('path norm')
    if save_fig:
        plt.savefig('norm_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    # ## plot of speeds over duration of control
    # fig = plt.figure()
    # plt.plot(durations,speeds)
    # plt.xlabel('time (seconds)')
    # plt.ylabel('speed (m/s)')

    # ## plot odom and near_path elements over duration of control
    # fig = plt.figure()
    # plt.plot(durations_crop,[odom[0] for odom in odoms_crop],'r:')
    # plt.plot(durations_crop,[near_point[0] for near_point in near_path_crop],'r')
    # plt.plot(durations_crop,[odom[1] for odom in odoms_crop],'b:')
    # plt.plot(durations_crop,[near_point[1] for near_point in near_path_crop],'b')
    # plt.xlabel('time (seconds)')
    # plt.ylabel('x ')

    ## plot of speed over duration of control
    fig = plt.figure()
    plt.plot(durations_crop,speeds_crop,'k')
    plt.plot(durations_crop,[0 for i in range(len(durations_crop))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('velocity (meters/second)')
    if save_fig:
        plt.savefig('velocity_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    ## plot of rmse over duration of control
    fig = plt.figure()
    plt.plot(durations_crop,rmse_crop,'k')
    plt.plot(durations_crop,[0 for i in range(len(durations_crop))],'k:')
    plt.xlabel('time (seconds)')
    plt.ylabel('RMSE (meters)')
    if save_fig:
        plt.savefig('rmse_'+experiment_id+extension,bbox_inches='tight')
        plt.close(fig)

    # ## plot of time derivative over duration of control
    # fig = plt.figure()
    # plt.plot(odurations,odtimes,'.')
    # plt.xlabel('time (seconds)')
    # plt.ylabel('\deltat (seconds)')

    # ## plot of distance derivative over duration of control
    # fig = plt.figure()
    # plt.plot(odurations,oddists,'.')
    # plt.xlabel('time (seconds)')
    # plt.ylabel('\deltad (meters)')

    plt.draw()
    plt.show()
    # plt.close('all')

    print('Done processing Experiment '+experiment_id+'.')


if __name__=="__main__":
    main()
