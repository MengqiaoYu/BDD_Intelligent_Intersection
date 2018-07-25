import requests
import csv
from os import listdir
import os.path
import shutil

def clean_name(streetName):
    """
    Return the full name of addresses
    """
    streetName_split = streetName.split()
    lastStr = streetName_split[-1]

    if lastStr == "St":
        streetName_split[-1] = "Street"
    if lastStr == "Ave":
        streetName_split[-1] = "Avenue"
    if lastStr == "Blvd":
        streetName_split[-1] = "Boulevard"
    if lastStr == "Rd":
        streetName_split[-1] = "Road"
    if lastStr == "Dr":
        streetName_split[-1] = "Drive"
    if lastStr == "Pkwy":
        streetName_split[-1] = "Parkway"
    if lastStr == "Hwy":
        streetName_split[-1] = "Highway"
    if lastStr == "Ct":
        streetName_split[-1] = "Court"
    if lastStr == "Pl":
        streetName_split[-1] = "Place"

    return ' '.join(streetName_split)

def is_nearest_intersection(data):
    """return
    (1) all unique intersections with clean name format and
    (2) files that don't have location information
    """

    loc_all = data["locations"]
    coordinate_intersection = []
    for loc in loc_all:
        # Use api to extract intersection info for each location
        # Important: extract nearest intersection using geonames api
        response = requests.get("http://api.geonames.org/findNearestIntersectionJSON?lat="
                                + str(loc["latitude"])
                                + "&lng="
                                + str(loc["longitude"])
                                + "&username=mengqiao")

        if response.status_code == 200:
            # Extract data if we can recieve response.
            try:
                response = response.json()["intersection"]
                # First street name
                street1 = clean_name(response["street1"])
                # Second street name
                street2 = clean_name(response["street2"])
                # City name
                cityName = response["placename"]
                # County name
                countyName = response["adminName2"]
                if street1 != street2:
                    # We don't consider this location if the two street names are same
                    if len(cityName) != 0:
                        # We use county name instead of city name if the field of city name is null.
                        coordinate_intersection.append([street1, street2, cityName])
                    else:
                        coordinate_intersection.append([street1, street2, countyName])
            except Exception:
                # If there are some other issues, return None
                return None
        else:
        # If there is no response, return None
            return None
    intersection_set1 = [list(x) for x in set(tuple(x) for x in coordinate_intersection)]
    intersection_set2 = [] # store the unique intersection
    for item in intersection_set1:
        # Find unique intersections since the two street names can be reversed.
        if (item not in intersection_set2) and ([item[1], item[0], item[2]] not in intersection_set2):
            intersection_set2.append(item)
    # Return unique intersections along the route
    return intersection_set2

def is_intersection(coordinate):
    """
    given (lat, lon)
    return the name of the intersection [streetname1, streetname2, city, fullstatename]
    """
    response = requests.get("http://api.geonames.org/findNearestIntersectionJSON?lat="
                            + str(coordinate[0])
                            + "&lng="
                            + str(coordinate[1])
                            + "&username=mengqiao")

    if response.status_code == 200:
        # Extract data if we can recieve response.
        try:
            response = response.json()["intersection"]
            # First street name
            street1 = clean_name(response["street1"])
            # Second street name
            street2 = clean_name(response["street2"])
            # City name
            cityName = response["placename"]
            # County name
            countyName = response["adminName2"]
            # State name
            stateName = response["adminName1"]

            if street1 != street2:
                # We don't consider this location if the two street names are same
                if len(cityName) != 0:
                    # We use county name instead of city name if the field of city name is null.
                    return sorted([street1, street2]) + [cityName, stateName]
                    # Sort street 1 and street 1 alphabetically to avoid swapped name
                else:
                    return sorted([street1, street2]) + [countyName, stateName]
        except Exception:
            # If there are some other issues, return None
            return None
    else:
    # If there is no response, return None
        return None

def main():
    intersection_all = []
    file_miss_all = []
    file_used_all = []
    iter_num = 0
    dir_val = '/Users/MengqiaoYu/Desktop/Val_data/'
    dir_used = '/Users/MengqiaoYu/Desktop/Val_usedData/'

    for f in listdir(dir_val):

        if f.startswith('.'):
        # Skip the hidden file in the folder
            continue

        iter_num += 1
        result = is_nearest_intersection(dir_val, os.path.splitext(f)[0]) # Get the results

        # Save the results to two lists and move the processed file to another folder.
        # Sometimes api can be disconnected and this step enables us to continue
        # processing without starting from the beginning.
        if result is not None:
            result.append(f)
            intersection_all.append(result)
        else:
            file_miss_all.append(f)
        file_used_all.append(f)

        # Give some process info
        if iter_num % 100 == 0:
            print ("This is " + str(iter_num) + "th file.")
            with open('/Users/MengqiaoYu/Desktop/Val_data_extract/first_'+str(iter_num) + 'outfile_summary.csv', 'wb') as f:
                writer = csv.writer(f)
                header = ["street1", 'street2', 'city', 'filename']
                writer.writerow(header)
                writer.writerows(intersection_all)

            with open('/Users/MengqiaoYu/Desktop/Val_data_extract/first_'+str(iter_num) + 'files_noLocation.csv', 'wb') as f_2:
                writer = csv.writer(f_2)
                header = ["filename"]
                writer.writerow(header)
                writer.writerows(file_miss_all)
            for f in file_used_all:
                shutil.move(dir_val + str(f), dir_used + str(f))
            file_used_all = []

if __name__ == '__main__':
    main()