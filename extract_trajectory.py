import json
from os import listdir
import os.path
from gmplot import gmplot

def extract_coordinate(json_data):
    """
    json_data is a json file
    return a sequence of coordinates along the trajectory
    """
    loc_all = json_data["locations"]
    coordinate_route = []
    for loc in loc_all:
        coordinate_route.append((loc["latitude"],loc["longitude"]))
    return coordinate_route

def extract_speed(json_data):
    """
    json_data is a json file
    return a list of speed along the trajectory (mph)
    """
    loc_all = json_data["locations"]
    speed_route = []
    for loc in loc_all:
        speed_route.append(loc["speed"])
    return  speed_route

def is_color(speed):
    if speed <= 5:
        return 'red'
    elif speed <= 10:
        return 'magenta'
    elif speed <= 15:
        return  'pink'
    elif speed <= 25:
        return  'darkgreen'
    else:
        return 'darkblue'

def main():
    gmap = gmplot.GoogleMapPlotter.from_geocode("San Francisco") # can also use (40.7543, -73.9484, 13) # Base: NewYork
    iter_num = 0
    dir_val = '/Users/MengqiaoYu/Desktop/BDD currently/VideoData_ForAlex/Val_data/'
    dir_used = '/Users/MengqiaoYu/Desktop/BDD currently/VideoData_ForAlex/Val_usedData/'

    print ("------------Start Extracting Trajectories for Each Video--------")

    for f in listdir(dir_val): # total  2500 files
        if f.startswith('.'):
            continue

        iter_num += 1
        with open(dir_val + str(os.path.splitext(f)[0]) + '.json') as json_data:
            data_curr = json.load(json_data)
        route_coor = extract_trajectory(data_curr) # Get the coordinates
        route_speed = extract_speed(data_curr) # Get the speed
        assert len(route_coor) == len(route_speed), "Missing attributes in " + f

        ### If want to save the trajectory data, uncomment the following codes
        # with open('/Users/MengqiaoYu/Desktop/BDD currently/VideoData_ForAlex/Results_route/route_'+ os.path.splitext(f)[0] + '.csv', 'wb') as outputfile:
        #     writer = csv.writer(outputfile)
        #     header = ["latitude", 'longitude']
        #     writer.writerow(header)
        #     writer.writerows(route_coor)
        # shutil.move(dir_val + str(f), dir_used + str(f))

        ### Visualization of all the trajectories in google map
        try:
            _lats, _lons = zip(*route_coor)
            for i in range(len(_lats) - 1):
              gmap.plot(_lats[i:i+2], _lons[i:i+2], is_color(route_speed[i]), edge_width = 3)
              ### If only show the trajectory without speed map, use the following line
              # gmap.plot(_lats, _lons, 'darkblue', edge_width = 3)
        except Exception:
            print ("Alert: no trajectory info in file " + f)

        if iter_num % 500 == 0:
            print ("Processed " + str(iter_num) + " files.")

    gmap.draw("trajectory_with_speed_map.html")

    print ("------------------------The END--------------------------------")