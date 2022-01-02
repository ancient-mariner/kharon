
-- historical artifact. sync must still be manually created
create_udp_sync("udp_sync")   

------------------------------------------------
-- specify where config and log data is stored

-- where device data is located
-- on a Pi, this will be /opt/kharon/data/dev/
set_device_dir("/opt/kharon/data/dev/")

-- environment describes alignment of devices to ship-centric space
set_environment("/opt/kharon/data/env/eagle/")  

-- 'default' cruise speed. this will be used for DR and collision avoidance
--    when GPS and instruments are out
cruise_speed_kts = 3.0


------------------------------------------------------------------------
------------------------------------------------------------------------
-- should be rare to need to edit this content

set_world_map_folder("/opt/kharon/mapping/master/")


------------------------------------------------------------------------
------------------------------------------------------------------------
-- should not need to edit content below. 
-- if editing required, try to create configuration variables that 
--    can be set above

set_cruise_speed_kts(cruise_speed_kts)


-- create IMU receiver
create_imu_receiver("eagle_imu", "eagle", "log")

-- create attitude module, subscribe it to IMU
create_attitude("attitude", "log")

subscribe_b_to_a("eagle_imu", "attitude")

--create_gps_receiver("gps", "log")
create_gps_receiver("eagle_gps", "log")

--priority rules
--   1  producer data is always used when available
--   2  producer data used when available, with 1/2 weight of P1
--   3  producer data used when P1 is not available, with same weight as P2
--   4  do-not-use
-- order: gyr, acc, mag (same column order as 's2')
set_imu_priority("eagle_imu",    1, 1, 1)


-- driver needed for autopilot, regardless of whether collision
--    avoidance is running
create_driver("driver")
subscribe_b_to_a("attitude", "driver")
subscribe_b_to_a("eagle_gps", "driver")
--set_destination("-123.07", "48.965", "150")     -- point roberts

