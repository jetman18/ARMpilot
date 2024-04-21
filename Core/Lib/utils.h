#ifndef __NAVIGATION__
#define __NAVIGATION__

#define sq(x) ((x)*(x))
#define MSTOSEC(x) (x*0.001f)
#define sign(X) (X > 0 ? 1 : -1)

static float range360(float deg){
    if(deg < 0)
        deg = 360 + deg;
    else if (deg > 359)
        deg = deg - 360;
    return deg;
}
static float range180(float val){
    if(val > 180)
        val = val - 360;
    else if (val < -180)
        val = val + 360;
    return val;
}

static int zeroSet(){
    return 0L;
}

#define Feq2Sec(F) (1.0f/F)

#endif