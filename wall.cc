/////////////////////////////////
// File: wall.cc
// Desc: dynamic wall for a petri dish
// Created: 2011-10-17
// Author: Stephen Makonin <smakonin@makonin.com>
// License: GPL
/////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stage.hh"
using namespace Stg;

const bool verbose = false;
const meters_t r = 49.5;
const meters_t d = 2 * r;
const meters_t C = M_PI * d;
const meters_t wlen = 2.68;
const radians_t degree_incro = ceil(C) / 360.0 * wlen;

static radians_t degree = 0.0;

// Stage calls this when the model starts up
extern "C" int Init( Model* mod )
{
    radians_t bearing = dtor(degree);
    mod->SetPose(Pose(r * cos(bearing), r * sin(bearing), 0, bearing));    
    degree += degree_incro;
}
