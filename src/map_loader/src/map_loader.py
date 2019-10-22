#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import rospy
import sys
import os

import xml.etree.ElementTree as ET

import numpy as np

from std_msgs.msg import String
from sk_msgs.msg import Waypoint
from sk_msgs.msg import WaypointArray
from geometry_msgs.msg import Point

from pprint import pprint as pp

wpArr = WaypointArray()

def core():
    rospy.init_node('map_loader')
    wpArr_pub = rospy.Publisher('/global_waypoints_latlon',WaypointArray,queue_size=10)
    rate = rospy.Rate(1)

    importOsmFile()
    print 'Importing osm file succeed!'

    while not rospy.is_shutdown():
        wpArr_pub.publish(wpArr)
        rate.sleep()


def importOsmFile():
    bCfg_ImportDebug = True

    script_dir = os.path.dirname(__file__)
    #rel_path = 'osmfiles/final_map_2.osm' # final
    #rel_path = 'osmfiles/preliminary_map.osm' # preliminary
    rel_path = 'osmfiles/konkuk_wp_20191810_124227_route_gen.osm'
    InputOsmPath=os.path.join(script_dir, rel_path)

    if bCfg_ImportDebug:
        print('[1. OSM Import] Import file: ', InputOsmPath)

    tree_import = ET.parse(InputOsmPath)
    root_import = tree_import.getroot()

    # Filtering and error check
    if bCfg_ImportDebug:
        print('[1. OSM Import] Filtering and error check')

    NumOfNode = 0
    NumOfWay = 0
    NumOfDelElement = 0

    for element_1st in root_import:
        # Counting
        if element_1st.tag == 'node':
            NumOfNode = NumOfNode + 1
        elif element_1st.tag == 'way':
            NumOfWay = NumOfWay + 1

        # Get child attributes
        strActionValue = element_1st.attrib.get('action')

        # Delete the attributes of actions
        if strActionValue != None:
            del element_1st.attrib['action']
            NumOfDelElement = NumOfDelElement + 1

        # Delete the element with action = delete
        if strActionValue=='delete':
            root_import.remove(element_1st)
            if bCfg_ImportDebug == True:
                print(element_1st.tag, element_1st.attrib, ' are deleted')

    # Debug display
    if bCfg_ImportDebug == True:
        print('NumOfNode: ', NumOfNode)
        print('NumOfWay: ', NumOfWay)
        print('NumOfDelElement: ', NumOfDelElement)

    if NumOfWay > 1 and NumOfNode <= 2:
        sys.exit("[1. OSM Import] This version cannot handle multiple way osm!")


    # Construction of Node dictionary
    if bCfg_ImportDebug:
        print('[1. OSM Import] Construction of node dictionary and way list')

    DictNodes = dict()

    for node_element in root_import.iter('node'):
        wp = Waypoint()
        tmp_node_id = node_element.attrib['id']
        tmp_node_lat = node_element.attrib['lat']
        tmp_node_lon = node_element.attrib['lon']
        DictNodes[tmp_node_id]=[tmp_node_lat, tmp_node_lon]

        wp.id = tmp_node_id
        wp.frame_id = 'world'
        wp.point.y = float(tmp_node_lat)
        wp.point.x = float(tmp_node_lon)

        #print 'node : ', tmp_node_id, tmp_node_lat, tmp_node_lon
        for tag in node_element.iter('tag'):
            if tag.attrib['k']=='heading':
                wp.heading=float(tag.attrib['v'])
            elif tag.attrib['k']=='speed':
                wp.speed=float(tag.attrib['v'])

            #print 'tag : ', tag.attrib

        wpArr.wp.append(wp)

    # Construction of lists for calculation
    ListLatLon = []
    for way_element in root_import.iter('way'):
        for nodes_in_way in way_element.iter('nd'):
            tmp_node_in_way_attrib_ref = DictNodes[nodes_in_way.attrib['ref']]
            ListLatLon.append([float(tmp_node_in_way_attrib_ref[0]), float(tmp_node_in_way_attrib_ref[1])])


if __name__ == '__main__':
    try:
        core()
    except rospy.ROSInterruptException:
        pass
