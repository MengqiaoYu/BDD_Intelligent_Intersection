import os
import II # Rename API as II
from os import listdir
import numpy as np
import numpy.linalg as la
import json
import overpy
import pyproj as proj
from shapely.geometry import Polygon, Point
from math import asin, cos, pi, sin, radians, sqrt, atan2
from extract_trajectory import extract_coordinate, extract_speed
from extract_name import is_intersection
import logging
import csv
import shutil
from collections import defaultdict

def deg2rad(angle):
    return angle * pi / 180

def rad2deg(angle):
    return angle * 180 / pi

def pointRadialDistance(lat1, lon1, bearing, distance):
    """
    Return final coordinates (lat2,lon2) [in degrees] given initial coordinates
    (lat1,lon1) [in degrees] and a bearing [in degrees] and distance [in km]
    """
    global rEarth
    rEarth = 6371.01 # Earth's average radius in km
    epsilon = 0.000001 # threshold for floating-point equality

    rlat1 = deg2rad(lat1)
    rlon1 = deg2rad(lon1)
    rbearing = deg2rad(bearing)
    rdistance = distance / rEarth # normalize linear distance to radian angle

    rlat = asin( sin(rlat1) * cos(rdistance) + cos(rlat1) * sin(rdistance) * cos(rbearing) )

    if cos(rlat) == 0 or abs(cos(rlat)) < epsilon: # Endpoint a pole
        rlon=rlon1
    else:
        rlon = ( (rlon1 - asin( sin(rbearing)* sin(rdistance) / cos(rlat) ) + pi ) % (2*pi) ) - pi

    lat = rad2deg(rlat)
    lon = rad2deg(rlon)
    return (lat, lon)

def angle_2vector(x, y):
    """
    Returns the angle in radians between vectors 'v1' and 'v2'
    """
    cosang = np.dot(x, y)
    sinang = la.norm(np.cross(x, y))
    return np.arctan2(sinang, cosang) * 180 / np.pi

def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance between two points
    return the distance in km.
    """
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * asin(sqrt(a))

    dis = rEarth * c
    return dis # in km

def find_area_intersections(coordinate, speed = 15):
    """
    coordinate: center
    return the intersections in the bounding box based on current location
    return type: get a list of Node objects representing each intersection:
    [<overpy.Node id=42800986 lat=40.7681152 lon=-73.9038760>,
     <overpy.Node id=42865836 lat=40.7689047 lon=-73.9069026>]
    """

    # Parameter 1
    radius = 0.25
    ## If use speed to define radius, uncomment the following two lines
    # acc = 5 # m/s^2
    # radius = max(0.2, (speed * 1.60934) ** 2 / (2 * 1000 * acc)) # in km
    center_lat = coordinate[0]
    center_lon = coordinate[1]
    north = pointRadialDistance(center_lat, center_lon, 0, radius)
    west = pointRadialDistance(center_lat, center_lon, 90, radius)
    south = pointRadialDistance(center_lat, center_lon, 180, radius)
    east = pointRadialDistance(center_lat, center_lon, 270, radius)

    # Use overpass api to extract intersection coordinates in the bounding box
    api = overpy.Overpass()
    query_result = api.query("""
    [bbox: """ + str (south[0]) + "," + str(west[1]) + "," + str(north[0]) + ',' + str(east[1]) + """];
    way[highway~"^(motorway|trunk|primary|secondary|tertiary|(motorway|trunk|primary|secondary)_link)$"]->.major;
    way[highway~"^(motorway|trunk|primary|secondary|tertiary|(motorway|trunk|primary|secondary)_link)$"]->.minor;
    node(w.major)(w.minor)[highway=traffic_signals];
    out; """)
    ret = query_result.get_nodes()
    if len(ret) == 0:
        return None
    else:
        return ret

def find_next_intersection(coor_curr, coor_next, coor_intersections):
    """
    find next intersection along the trajectory based on current loc and 1 sec later.
    return the intersection (lat, lon)
    return -1, if the vehicle stops
    return none if cannot find next intersection in current scope
    """
    # setup your projections
    crs_wgs = proj.Proj(init='epsg:3857') # assuming you're using WGS84 geographic
    crs_bng = proj.Proj(init='epsg:27700') # use a locally appropriate projected CRS

    # then cast your geographic coordinate pair to the projected system
    x_curr, y_curr = proj.transform(crs_wgs, crs_bng, coor_curr[1], coor_curr[0])
    x_next, y_next = proj.transform(crs_wgs, crs_bng, coor_next[1], coor_next[0])

    # Stopping/waiting --> unstable GPS
    if haversine(coor_curr[1], coor_curr[0], coor_next[1], coor_next[0]) <= 0.0009:
        # Parameter 2
        # 2mph
        return -1

    # Find next intersection based on distance and angle
    # Parameter 3
    angle_max = 60
    dist_max = 1
    for item in coor_intersections:
        inx_lat, inx_lon = float(item.lat), float(item.lon)
        x_inx, y_inx = proj.transform(crs_wgs, crs_bng, inx_lon, inx_lat)
        v_1 = np.array([x_next - x_curr, y_next - y_curr])
        v_2 = np.array([x_inx - x_curr, y_inx - y_curr])
        dis2inx = haversine(coor_curr[1], coor_curr[0], inx_lon, inx_lat)
        # Parameter 4
        if dis2inx <= 0.03:
            return (inx_lat, inx_lon)
        if angle_2vector(v_1, v_2) < angle_max and dis2inx < dist_max:
            dist_max = dis2inx
            target_inx = (inx_lat, inx_lon)
            continue
    if dist_max == 1:
        return None
    return target_inx

def build_guideway_polygon(guideway_data):
    """
    given a guideway info: ['direction', 'origin_lane', 'destination_lane',
    'left_border', 'median', 'right_border', 'id', 'type']
    return the Polygon geometry
    """
    if 'reduced_left_border' in guideway_data.keys():
        polygon_sequence = guideway_data['reduced_left_border'] + guideway_data['reduced_right_border'][::-1]
    else:
        polygon_sequence = guideway_data['left_border'] + guideway_data['right_border'][::-1]
    return Polygon(polygon_sequence)

def which_guideway(coor, guideway_all):
    """
    return the guideway id of current location
    """
    for i, guideway in enumerate(guideway_all):
        if build_guideway_polygon(guideway).contains(Point(coor[1], coor[0])):
            return i
    return None

def which_conflict_zone(ins_name, coor_curr, zone = "one"):
    """
    (street1, street2, city, state)
    calculate conflict zone(s) and save the figures
    """
    # city_ins_name = ins_name[2] + ins_name[3] + "USA"
    # TODO: Download bounding box of the intersection given radius from osm.
    osm_file = "/Users/MengqiaoYu/Desktop/BDD currently/VideoData_ForAlex/Val_data_extract/Howard_5th.osm"

    # city = II.get_data(city_ins_name = city_ins_name)
    city = II.get_data(file_name=osm_file)

    x_section_addr = (ins_name[0], ins_name[1])
    x_section = II.get_intersection(x_section_addr, city, crop_radius=200.0)
    guideways = II.get_guideways(x_section)
    # print ("There are %d guideways at this intersection." % len(guideways))

    # Default: only show the conflict zone(s) related with current guideway.
    if zone == "one":
        guidway_id = which_guideway(coor_curr, guideways)
        if guidway_id is None:
            print ("No guideway found for this location!")
            return None
        conflictzone = II.get_conflict_zones(guideways[guidway_id], guideways)
        fig_CZ = II.get_conflict_zone_image(conflictzone, x_section)
        return fig_CZ
    # If want to show all conflict zones at the intersection, set zone = "all".
    else:
        fig_set = []
        for i in range(len(guideways)):
            conflictzones = II.get_conflict_zones(guideways[i], guideways)
            fig_CZ = II.get_conflict_zone_image(conflictzones, x_section)
            fig_set.append(fig_CZ)
        return fig_set

def analyze_II():
    dir_val = '/Users/MengqiaoYu/Desktop/BDD currently/VideoData_ForAlex/Val_data/'
    # dir_used = '/Users/MengqiaoYu/Desktop/BDD currently/VideoData_ForAlex/Val_usedData/'
    dir_saved = '/Users/MengqiaoYu/Desktop/BDD currently/VideoData_ForAlex/Val_guideway/'
    # miss_file = [] # Store the file name with no intersection info
    for f in listdir(dir_val): # total  2500 files
        print ("Now is the file %s." % f)
        if f.startswith('.'):
            continue
        with open(dir_val + str(os.path.splitext(f)[0]) + '.json') as json_data:
            data_curr = json.load(json_data) # .json file
        route_coor = extract_coordinate(data_curr) # Get the coordinates
        route_speed = extract_speed(data_curr)
        assert len(route_coor) == len(route_speed), "Missing attributes in " + f

        # For each data point (1s), check its next intersection along the route;
        # If it is too far away, will return None.
        None_count = 0
        # TODO: add edge cases using None_count
        for i in range(len(route_coor)):
            print ("For location %d." % (i + 1))
            coor_curr = route_coor[i]
            speed_curr = route_speed[i]
            coor_next = route_coor[i + 1]
            inx_area = find_area_intersections(coor_curr, speed_curr)
            # print ("There are %d intersections near here." % len(inx_area))
            if inx_area is None:
                None_count += 1
                continue
            inx_next_coor = find_next_intersection(coor_curr, coor_next, inx_area)
            if inx_next_coor is None:
                None_count += 1
                print ("Next intersection is not in scope!")
                continue
            inx_next_name = is_intersection(inx_next_coor) #[street1, street2, city, state]
            print ("The next intersection is %s." % inx_next_name)
            cz_next = which_conflict_zone(inx_next_name, coor_curr)
            print ('Save the conflict zone fig.')
            cz_next.savefig(dir_saved + str(os.path.splitext(f)[0]) + '_' + str(i + 1) + '.png')

def analyze_SF():
    """
    Identify signalized intersections in San Francisco, for which we have video feed and GPS traces in BDD 100k.
    Sort them in the descending order by number of traces per intersection.
    Extract traces for these intersections in the form of a Python dictionary of the following form: {(street 1, street 2): <list of traces>}, where the tuple (street1, street 2) identifies the intersection, and each trace in the list contains a name of the video file in addition to the sequence of (lon, lat) coordinates.
    """
    # Change here: The folder where stores all the validation files in BDD 100K
    dir_val = '/Users/MengqiaoYu/Desktop/BDD currently/VideoData_ForAlex/Val_data/'
    # Change here: The folder where stores the files have been processed
    dir_used = '/Users/MengqiaoYu/Desktop/BDD currently/VideoData_ForAlex/Val_usedData/'

    inx_count_dict = {}
    inx_traj_dict = {}
    inx_file_dict = {}
    logger = logging.getLogger()
    logging.basicConfig(format='%(asctime)s %(message)s', level=logging.INFO)

    for f in listdir(dir_val): # total  2500 files

        ### Read the file
        if f.startswith('.'):
            continue
        with open(dir_val + str(os.path.splitext(f)[0]) + '.json') as json_data:
            data_curr = json.load(json_data) # .json file
        logger.info("Now is the file %s." % f)

        ### Get the coordinates and speeds, deal with edge case
        route_coor = extract_coordinate(data_curr)
        route_speed = extract_speed(data_curr)
        shutil.move(dir_val + str(f), dir_used + str(f))
        # If the video duration is shorter than 5 seconds, jump to next file.
        if len(route_coor) <= 5:
            logger.info("There is no effective trajectory in the file %s." %f)
            continue
        assert len(route_coor) == len(route_speed), "Missing attributes in " + f
        # If the trajectory is not around SF, give a warning and jump to next file
        bound_left = [-123.0137, 37.6040]
        bound_right = [-122.3549, 37.8324]
        if route_coor[0][0] < 37.6040 or route_coor[0][1] > 37.8324 \
            or route_coor[0][1] > -122.3549 or route_coor[0][1] < -123.0137:
            logger.info("This trajectory is not in SF.")
            continue

        ### For each data point (1s), check its next intersection along the route;
        none_count = 0
        stopping_count = 0
        inx_name_set = []
        for i in range(0, len(route_coor) - 1, 2):
            logger.debug("For location %d." % (i + 1))

            coor_curr = route_coor[i]
            logger.debug(coor_curr)
            speed_curr = route_speed[i]
            coor_next = route_coor[i + 1]

            # Find intersections near current location
            inx_area = find_area_intersections(coor_curr, speed_curr)
            if inx_area is None:
                none_count += 1
                continue
            logger.debug("There are %d intersections near here." % len(inx_area))

            # Find next intersection based on some rules
            inx_next_coor = find_next_intersection(coor_curr, coor_next, inx_area)
            # If it is too far away, will return None; o/w, find inx name.
            if inx_next_coor is None:
                none_count += 1
                logger.debug("Next intersection is not in scope!")
                continue
            if inx_next_coor == -1:
                stopping_count += 1
                continue
            inx_next_name = is_intersection(inx_next_coor) #[street1, street2, city, state]
            if inx_next_name is None:
                none_count += 1
                logger.debug("Next intersection doesn't have a name!")
                continue
            logger.debug("The next intersection is %s." % inx_next_name)

            # Store the intersection names along the route (easily duplicate) if in SF; \
            # o/w, jump to next file
            if inx_next_name[2] != "San Francisco":
                logger.info("This trajectory is not in SF.")
                break
            inx_name_set.append(inx_next_name)

        # If the route is not in SF, jump to next file
        if i < range(0, len(route_coor) - 1, 2)[-1]:
            continue

        # If the route is too far away, will return None and give warning.
        if none_count == len(range(0, len(route_coor) - 1, 2)):
            logger.info("The trajectory in file %s is far away from intersections" %f)
            continue
        if stopping_count == len(range(0, len(route_coor) - 1, 2)):
            logger.info("The trajectory in file %s is stopping." %f)
            continue

        ## Find the unique intersection names along this route
        inx_name_set = [(x[0], x[1]) for x in set(tuple(x) for x in inx_name_set)]
        for inx_name in inx_name_set:
            if inx_name not in inx_count_dict:
                inx_count_dict[inx_name] = 1
                inx_traj_dict[inx_name] = [route_coor]
                inx_file_dict[inx_name] = [f]

            else:
                inx_count_dict[inx_name] += 1
                inx_traj_dict[inx_name].append(route_coor)
                inx_file_dict[inx_name].append(f)

    with open('/Users/MengqiaoYu/Desktop/BDD currently/VideoData_ForAlex/dict_count_0724.csv', 'w') as csv_file:
        writer = csv.writer(csv_file)
        for key, value in inx_count_dict.items():
           writer.writerow([key, value])
    with open('/Users/MengqiaoYu/Desktop/BDD currently/VideoData_ForAlex/dict_traj_0724.csv', 'w') as csv_file:
        writer = csv.writer(csv_file)
        for key, value in inx_traj_dict.items():
           writer.writerow([key, value])
    # with open('/Users/MengqiaoYu/Desktop/BDD currently/VideoData_ForAlex/dict_file_0724.csv', 'w') as csv_file:
    #     writer = csv.writer(csv_file)
    #     for key, value in inx_file_dict.items():
    #        writer.writerow([key, value])
