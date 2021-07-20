//
// Created by 徐豪 on 2018/4/22.
//
#include <fstream>
#include "configure/global_variables.h"
#include "core/LayerGraph.h"

int glob_vars::current_input = -1;
model glob_vars::current_model;
string glob_vars::current_debug_dir;
std::ofstream glob_vars::config_file;
string glob_vars::sketch_path;

SketchGraph*  glob_vars::sketchGraph  = NULL;
GuidingGraph* glob_vars::guidingGraph = NULL;
BrickFactory *glob_vars::brickFactory = NULL;

vector<model> config::model_names = {
        {"default",                "input_from_user",
                false,false,false, 0, 0.9995, {1.0,   100,    50    ,100,  0.03,    0.00,    0     , 10     , 10}
//                                          w_dev  w_simp  w_rigid w_gap  w_bound  w_lsymm  w_colld  w_conn  ph_ratio
        },
        {"line",              "0_line/sketch/line_4.obj",
                false,true,false, 0, 0.999, {1.0,   100,    50     ,100,  0.03,   0.00,    10     , 10      ,  10}
//                                    w_dev  w_simp  w_rigid w_gap w_bound w_lsymm  w_colld  w_conn     ph_ratio
        },
        {"cube",              "0_cube/sketch/cube_11.obj",
                false,true,true, 0, 0.999,  {1.0,   100,    50     ,0,  0.03,   0.00,    0     , 10       , 10 }
//                                         w_dev  w_simp  w_rigid w_gap w_bound w_lsymm  w_colld  w_conn     ph_ratio
        },
        {"siggraph",          "0_siggraph/sketch/2.obj",
                false,false,false,0, 0.9998,  {1,   100,    0.0     ,100,  0.03,   0.00,    10      ,10      , 10 }
        },
        {"robotarm",          "0_robotarm/sketch/robotarm_simplified3_rh.obj",
                true,true,false,0, 0.999,  {100,   0.1,    0.1     ,100,  0.03,   0.00,    10     ,10        , 10 }
        },
        {"lifter_remove_3",            "0_lifter/sketch/lifter_rh.obj",
                true,true,true, 10, 0.99992, {0,    100,   0.1     ,100,  0.03,   0.00,    10     ,10         , 10 }
        },
        {"flyingkite",        "0_flyingkite/sketch/fly_rh.obj",
                false,true,false, 2, 0.999, {10,  100,      0      ,100,  0.03,     20,    20     ,10        , 10 }
        },
        {"bookshelf",         "0_bookshelf/sketch/bookshelf_2021.obj",
                false,true,false,3 , 0.99995,   {0.0,    100,    100     ,100,  0.03,    0.00,    100     ,10         , 10 }
        },
        {"small_crossbow",    "0_crossbow_small/sketch/smallcrossbow_rh.obj",
                false,true,false,3 , 0.99995, {50,    100,    0     ,100,  0.03,   0.00,    0     ,0         , 10 }
        },
        {"normalbow",         "0_normalbow/sketch/normalbow.obj",
                false,true, true, 10, 0.9997, {100,    0.1,    200     , 100,  0.03,   10,    10     ,10         , 10 }
        },
        {"plane",             "0_plane/sketch/plane_extend_tail.obj",
                false,true,false,6, 0.9998,  {0,   100,    50     ,100,  0.03,   0.00,    100     ,10         , 10 }
        },
        {"crossbow",          "0_crossbow/sketch/old/withoutbelt_rh.obj",
                false,true,false,0, 0.999, {1.0,   100,    50     ,100,  0.03,   0.00,    10     ,10            , 10 }
        },
        {"3Dprinter",         "0_3D_printer/sketch/3Dprinter_rh.obj",
                true,true,false ,0, 0.99997, {100.0,   0.1,    100     ,100,  0.03,   10.0,    10      ,10       , 10 }
        },
        {"bridge", "0_bridge/sketch/bridge_width_modify_rh.obj",
                true,true,false ,0, 0.99997, {1.0,   100,    200     ,100,  3,   30,    100      ,10         , 10 }
        },
        {"tokyotower",        "0_tokyotower/sketch/tower_simplified_rh.obj",
                false,true,false,5, 0.99996, {0.0,   100.0,    0.0  ,100,   20,    50,     10     , 10      , 10 }
//                                          w_dev  w_simp  w_rigid  w_gap w_bound  w_lsymm w_colld  w_conn  ph_ratio
        },
        {"tokyo_temple",      "0_tokyo_temple/sketch/bottom_simple_rh.obj",
                true,true,false, 0, 0.9998, {100,   0.1,    0.1     ,100,   0.03,    50,    10      ,10         , 10 }
        },
        {"spacesation",       "0_spacestation/sketch/spacestation_small_symmetry_simplified2_rh.obj",
                true, true,false, 2, 0.99997, {100,  0.0,    0.1     ,100,  0.03,   0.00,    100     ,10         , 10 }
        },
        {"temple_head",       "0_tokyo_temple/sketch/top_simple2_rh.obj",
                true,true,false , 0, 0.9998, {100,   0.1,    0.1     ,100,   0.03,    50,    10      ,10         , 10 }
        },
        {"triangle",       "0_experiments/triangle/triangle_2d_10.obj",
                false,true,false, 0, 0.9998, {0.1,   100,    0.1      ,50,   1,    10,    10      ,10         , 10 }
        },
        {"tian",       "0_experiments/tian/tian_10.obj",
                false,false,false, 0, 0.9996, {100,   0.1,    0.1      ,100,   0.03,    50,    10      ,10         , 10 }
        },
        {"triangle_tile",       "0_experiments/triangle_tile/triangle2.obj",
                        false,true,false, 0, 0.99997, {0,  100,    0.1      ,100,    1,      10,      0        ,10   ,  10 }
                //                                    w_dev  w_simp  w_rigid   w_gap   w_bound w_lsymm  w_colld  w_conn  ph_ratio
        },
        {"wind",       "0_experiments/wind/wind_10_tile1.obj",
         false,false,false, 1, 0.99995, {0.1,   100,    0        ,100,    1,      0,      0        ,0   ,  0 }
                //                      w_dev  w_simp  w_rigid   w_gap   w_bound w_lsymm  w_colld  w_conn  ph_ratio
        },
        {"castle",       "0_disneycastle/disney_castle_simp_v2_rh.obj",
            false,false,false, 3, 0.99997, {0,   100,    100      ,100,    1,      10,      0        ,10   ,  10 }
                //                         w_dev  w_simp  w_rigid   w_gap   w_bound w_lsymm  w_colld  w_conn  ph_ratio
        },
        {"fan",       "0_fan/fan.obj",
            false,false,true, 1, 0.9998, {0,   100,    300      ,100,    1,      10,      0        ,10   ,  10 }
                //                         w_dev  w_simp  w_rigid   w_gap   w_bound w_lsymm  w_colld  w_conn  ph_ratio
        },
        {"fail_1",       "0_fail/ball_simp_simp.obj",
            false,false,false, 1, 0.9998, {0,   100,    300      ,100,    1,      10,      0        ,10   ,  10 }
                //                         w_dev  w_simp  w_rigid   w_gap   w_bound w_lsymm  w_colld  w_conn  ph_ratio
        },
        {"fail_2",       "0_fail/squares.obj",
            false,false,false, 1, 0.9998, {0,   100,    300      ,100,    1,      10,      0        ,10   ,  10 }
                //                         w_dev  w_simp  w_rigid   w_gap   w_bound w_lsymm  w_colld  w_conn  ph_ratio
        },
        {"user_ruihui",       "0_user_study/ruihui_car.obj",
            false,false,false, 1, 0.9999, {0,   100,    0      ,100,    1,      10,      10        ,10   ,  10 }
                //                         w_dev  w_simp  w_rigid   w_gap   w_bound w_lsymm  w_colld  w_conn  ph_ratio
        },
        {"cartoon_car",       "5_other_models/cartoon_car/sketch/cartoon_sf_rh.obj",
            false,true,false, 1, 0.99996, {0,   100,    0      ,100,    1,      10,      10        ,0   ,  10 }
                //                        w_dev  w_simp  w_rigid   w_gap   w_bound w_lsymm  w_colld  w_conn  ph_ratio
        },
        {"bicycle",       "0_user_study/bicycle.obj",
            false,true,false, 1, 0.99996, {0,   100,     0      ,100,    0,      10,      0        ,0   ,  10 }
                //                        w_dev  w_simp w_rigid   w_gap   w_bound w_lsymm  w_colld  w_conn  ph_ratio
        },
        {"BOC",       "0_user_study/BOC_part1_rh.obj",
            false,false,false, 1, 0.99996, {100,   0,     0      ,100,    0,      10,      0        ,0   ,  10 }
                //                        w_dev  w_simp w_rigid   w_gap   w_bound w_lsymm  w_colld  w_conn  ph_ratio
        },
        {"glass",       "0_user_study/xy_glass2_rh.obj",
            false,false,true, 12, 0.99996, {0,   100,     0      ,100,    0,      10,      0        ,0   ,  10 }
                //                        w_dev  w_simp w_rigid   w_gap   w_bound w_lsymm  w_colld  w_conn  ph_ratio
        },
        {"wall-e",       "0_user_study/walle_tx_x2_rh.obj",
            false,true,false, 9, 0.99997, {0,   100,     150      ,100,    0,      10,       5        ,0   ,  10 }
                //                        w_dev  w_simp w_rigid   w_gap   w_bound w_lsymm  w_colld  w_conn  ph_ratio
        },
        {"wheel",       "0_user_study/wheel.obj",
            false,false,true, 1, 0.99996, {0,   100,     0      ,100,    0,      10,      0        ,0   ,  10 }
                //                        w_dev  w_simp w_rigid   w_gap   w_bound w_lsymm  w_colld  w_conn  ph_ratio
        },
        {"bird",       "0_bird/mahou_rh.obj",
            false,true,false, 5, 0.99995, {0,   100,     150      ,100,    0,      10,      0        ,0   ,  10 }
                //                        w_dev  w_simp w_rigid   w_gap   w_bound w_lsymm  w_colld  w_conn  ph_ratio
        }

};