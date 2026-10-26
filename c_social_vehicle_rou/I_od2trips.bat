od2trips -n 1_traffic_zone.taz.xml -d 2_design_social_vehicle.od.txt --timeline 0:1,3600:2,7200:5,10800:4,14400:2,18000:0 --end 18000 --vtype Social_Vehicle -o 3_design_social_vehicle.odtrips.xml
od2trips -n 1_traffic_zone.taz.xml -d 2_random_social_vehicle.od.txt --timeline 0:1,3600:2,7200:5,10800:4,14400:2,18000:0 --end 18000 --vtype Social_Vehicle -o 3_random_social_vehicle.odtrips.xml
exit