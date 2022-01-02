#include "logger.h"


#include "../world_map.c"
//#include "../path_map.c"
//#include "../zoutput.c"


const world_coordinate_type TEST = { .lon=238.49131, .lat=48.74467 };

const world_coordinate_type SQUAL = { .lon=-122.51667f, .lat=48.7417 };
const world_coordinate_type CHUCK = { .lon=-122.49750, .lat=48.6810 };
//const world_coordinate_type CHUCK = { .lon=-122.49861, .lat=48.6810 };
//const world_coordinate_type CHUCK = { .lon=-122.4989f, .lat=48.6867f };

const world_coordinate_type POINT_BOB = { .lon=-123.07f, .lat=48.965f };
const world_coordinate_type SUCIA = { .lon=-122.87f, .lat=48.757f };
const world_coordinate_type FH = { .lon=-122.993f, .lat=48.547f };
const world_coordinate_type SYDNEY = { .lon=-123.374f, .lat=48.641f };
const world_coordinate_type BHAM = { .lon=-122.523f, .lat=48.744f };
const world_coordinate_type ANACORTES = { .lon=-122.582f, .lat=48.517f };
const world_coordinate_type PT_TOWNSEND = { .lon=-122.765f, .lat=48.093f };
const world_coordinate_type PT_ANGELES = { .lon=-123.454f, .lat=48.148f };
const world_coordinate_type VICTORIA = { .lon=-123.444f, .lat=48.412f };
const world_coordinate_type BALLARD = { .lon=-122.4265, .lat=47.6822 };

int main(int argc, char **argv)
{
   (void) argc;
   (void) argv;
   set_log_dir_string("/tmp/");
   image_size_type size = { .x=720, .y=720 };
   world_coordinate_type vessel_pos = BHAM;
   set_world_map_folder_name("/opt/kharon/mapping/master/");
   path_map_type *path_map = create_path_map(size);
   if (init_beacon_list() != 0) {
      printf("Failed to init beacon list. Put on crash helmet\n");
   }
   //
//   trace_routes(path_map, PT_TOWNSEND);
//   write_path_map(path_map, "x_weights_port_townsend.pnm");
//   write_direction_map(path_map, "x_direction_port_townsend.pnm");
//   trace_routes(path_map, ANACORTES);
//   write_path_map(path_map, "x_weights_anacortes.pnm");
//   write_direction_map(path_map, "x_direction_anacortes.pnm");
//   trace_routes(path_map, BHAM);
//   write_path_map(path_map, "x_weights_bellingham.pnm");
//   write_direction_map(path_map, "x_direction_bellingham.pnm");
//   trace_routes(path_map, FH);
//   write_path_map(path_map, "x_weights_friday_harbor.pnm");
//   write_direction_map(path_map, "x_direction_friday_harbor.pnm");
//   trace_routes(path_map, SUCIA);
//   write_path_map(path_map, "x_weights_sucia.pnm");
//   write_direction_map(path_map, "x_direction_sucia.pnm");
//   trace_routes(path_map, POINT_BOB);
//   write_path_map(path_map, "x_weights_point_roberts.pnm");
//   write_direction_map(path_map, "x_direction_point_roberts.pnm");
//   trace_routes(path_map, SQUAL);
//   write_path_map(path_map, "x_weights_squal.pnm");
//   write_direction_map(path_map, "x_direction_squal.pnm");
   //
//   trace_route_initial(path_map, CHUCK, vessel_pos);
//   write_path_map(path_map, "x_weights_chuck.pnm");
//   write_direction_map(path_map, "x_direction_chuck.pnm");
   trace_route_initial(path_map, BALLARD, vessel_pos);
   write_path_map(path_map, "x_weights_ballard.pnm");
   write_direction_map(path_map, "x_direction_ballard.pnm");
   write_depth_map(path_map, "x_region.pnm");
for (uint32_t i=0; i<path_map->num_beacons; i++) {
   map_beacon_reference_type *ref = &path_map->beacon_ref[i];
   if (ref->path_weight > 0.0f) {
      printf("%d  %.3f,%.3f   dist %.1fm   wt %.1f\n", ref->index, ref->coords.akn_x, ref->coords.akn_y, (double) ref->center_dist_met, (double) ref->path_weight);
   }
}
   return 0;
}

