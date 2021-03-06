/***********************************************************************
* This file is part of kharon <https://github.com/ancient-mariner/kharon>.
* Copyright (C) 2019-2022 Keith Godfrey
*
* kharon is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, version 3.
*
* kharon is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with kharon.  If not, see <http://www.gnu.org/licenses/>.
***********************************************************************/

struct distribution_map {
    uint8_t nw, ne, sw, se;
};

const struct distribution_map map_[64] = {
   { .nw=64, .ne=0, .sw=0, .se=0 }, // 0,0
   { .nw=56, .ne=8, .sw=0, .se=0 }, // 1,0
   { .nw=48, .ne=16, .sw=0, .se=0 }, // 2,0
   { .nw=40, .ne=24, .sw=0, .se=0 }, // 3,0
   { .nw=32, .ne=32, .sw=0, .se=0 }, // 4,0
   { .nw=24, .ne=40, .sw=0, .se=0 }, // 5,0
   { .nw=16, .ne=48, .sw=0, .se=0 }, // 6,0
   { .nw=8, .ne=56, .sw=0, .se=0 }, // 7,0
   { .nw=56, .ne=0, .sw=8, .se=0 }, // 0,1
   { .nw=49, .ne=7, .sw=7, .se=1 }, // 1,1
   { .nw=42, .ne=14, .sw=6, .se=2 }, // 2,1
   { .nw=35, .ne=21, .sw=5, .se=3 }, // 3,1
   { .nw=28, .ne=28, .sw=4, .se=4 }, // 4,1
   { .nw=21, .ne=35, .sw=3, .se=5 }, // 5,1
   { .nw=14, .ne=42, .sw=2, .se=6 }, // 6,1
   { .nw=7, .ne=49, .sw=1, .se=7 }, // 7,1
   { .nw=48, .ne=0, .sw=16, .se=0 }, // 0,2
   { .nw=42, .ne=6, .sw=14, .se=2 }, // 1,2
   { .nw=36, .ne=12, .sw=12, .se=4 }, // 2,2
   { .nw=30, .ne=18, .sw=10, .se=6 }, // 3,2
   { .nw=24, .ne=24, .sw=8, .se=8 }, // 4,2
   { .nw=18, .ne=30, .sw=6, .se=10 }, // 5,2
   { .nw=12, .ne=36, .sw=4, .se=12 }, // 6,2
   { .nw=6, .ne=42, .sw=2, .se=14 }, // 7,2
   { .nw=40, .ne=0, .sw=24, .se=0 }, // 0,3
   { .nw=35, .ne=5, .sw=21, .se=3 }, // 1,3
   { .nw=30, .ne=10, .sw=18, .se=6 }, // 2,3
   { .nw=25, .ne=15, .sw=15, .se=9 }, // 3,3
   { .nw=20, .ne=20, .sw=12, .se=12 }, // 4,3
   { .nw=15, .ne=25, .sw=9, .se=15 }, // 5,3
   { .nw=10, .ne=30, .sw=6, .se=18 }, // 6,3
   { .nw=5, .ne=35, .sw=3, .se=21 }, // 7,3
   { .nw=32, .ne=0, .sw=32, .se=0 }, // 0,4
   { .nw=28, .ne=4, .sw=28, .se=4 }, // 1,4
   { .nw=24, .ne=8, .sw=24, .se=8 }, // 2,4
   { .nw=20, .ne=12, .sw=20, .se=12 }, // 3,4
   { .nw=16, .ne=16, .sw=16, .se=16 }, // 4,4
   { .nw=12, .ne=20, .sw=12, .se=20 }, // 5,4
   { .nw=8, .ne=24, .sw=8, .se=24 }, // 6,4
   { .nw=4, .ne=28, .sw=4, .se=28 }, // 7,4
   { .nw=24, .ne=0, .sw=40, .se=0 }, // 0,5
   { .nw=21, .ne=3, .sw=35, .se=5 }, // 1,5
   { .nw=18, .ne=6, .sw=30, .se=10 }, // 2,5
   { .nw=15, .ne=9, .sw=25, .se=15 }, // 3,5
   { .nw=12, .ne=12, .sw=20, .se=20 }, // 4,5
   { .nw=9, .ne=15, .sw=15, .se=25 }, // 5,5
   { .nw=6, .ne=18, .sw=10, .se=30 }, // 6,5
   { .nw=3, .ne=21, .sw=5, .se=35 }, // 7,5
   { .nw=16, .ne=0, .sw=48, .se=0 }, // 0,6
   { .nw=14, .ne=2, .sw=42, .se=6 }, // 1,6
   { .nw=12, .ne=4, .sw=36, .se=12 }, // 2,6
   { .nw=10, .ne=6, .sw=30, .se=18 }, // 3,6
   { .nw=8, .ne=8, .sw=24, .se=24 }, // 4,6
   { .nw=6, .ne=10, .sw=18, .se=30 }, // 5,6
   { .nw=4, .ne=12, .sw=12, .se=36 }, // 6,6
   { .nw=2, .ne=14, .sw=6, .se=42 }, // 7,6
   { .nw=8, .ne=0, .sw=56, .se=0 }, // 0,7
   { .nw=7, .ne=1, .sw=49, .se=7 }, // 1,7
   { .nw=6, .ne=2, .sw=42, .se=14 }, // 2,7
   { .nw=5, .ne=3, .sw=35, .se=21 }, // 3,7
   { .nw=4, .ne=4, .sw=28, .se=28 }, // 4,7
   { .nw=3, .ne=5, .sw=21, .se=35 }, // 5,7
   { .nw=2, .ne=6, .sw=14, .se=42 }, // 6,7
   { .nw=1, .ne=7, .sw=7, .se=49 }, // 7,7
};
