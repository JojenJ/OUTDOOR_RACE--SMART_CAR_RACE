# OUTDOOR_RACE--SMART_CAR_RACE

This project is mainly created for 19th SMART CAR RACE -- outdoor race.

The complete codes won't be provided since the most of it was provided by supplier, which we do a little changes on encoder and imu. Codes in src folder is for competition mission, embodying our work.

Codes
mainly written in python, and was all intergrated into one file only for dealing with the problem of cpu occupation, that can't be effectively solve in the conditon of multiple files.

which involes slam--gmapping, path planning, car controller--stanley, rador message processing, CV--yolov5, ROS


the ultimate test of this project is on the competition, where the car finished the all the mission and sucessfully ran on the track, clocking 37s, that rank the top ten on the nation final.
the maximum speed we tested is 35s, which mainly restricted by slam stability.
