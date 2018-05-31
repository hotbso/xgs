#include <float.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <time.h>
#include <sys/param.h>
#include "XPLMPlugin.h"
#include "XPLMPlanes.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPLMNavigation.h"

#include <acfutils/airportdb.h>
#include <acfutils/dr.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <Carbon/Carbon.h>
#else
#include <stdlib.h>
#include <GL/gl.h>
#endif

#define VERSION "3.0b2"

static float gameLoopCallback(float inElapsedSinceLastCall,
                float inElapsedTimeSinceLastFlightLoop, int inCounter,    
                void *inRefcon);
                
#define MS_2_FPM 196.850
#define M_2_FT 3.2808
#define STATE_LAND 1
#define STATE_AIR 2

#define WINDOW_HEIGHT 125
#define STD_WINDOW_WIDTH 180

static XPLMWindowID gWindow = NULL;
static XPLMDataRef vertSpeedRef, gearKoofRef, flightTimeRef;
static XPLMDataRef craftNumRef, icaoRef;
static XPLMDataRef gForceRef;
static dr_t lat_dr, lon_dr, y_agl_dr, hdg_dr;

static char landMsg[7][100];
static int lastState;
static float landingSpeed = 0.0f;
static float lastVSpeed = 0.0f;
static float landingG = 0.0f;
static float lastG = 0.0f;
static float remainingShowTime = 0.0f;
static float remainingUpdateTime = 0.0f;

static int winPosX = 20;
static int winPosY = 600;
static int lastMouseX, lastMouseY;
static int windowCloseRequest = 0;
static XPLMMenuID xgsMenu = NULL;
static int enableLogItem;
static int logEnabled = 0;

static int logThisLanding = 0;
static char logAirportId[50];
static char logAirportName[300];
static char logAircraftNum[50];
static char logAircraftIcao[40];
static time_t landingTime;


typedef struct rating_ { float limit; char txt[100]; } rating_t;
static rating_t std_rating[] = {
	{0.5, "excellent landing"},
	{1.0, "good landing"},
	{1.5, "acceptable landing"},
	{2.0, "hard landing"},
	{2.5, "you are fired!!!"},
	{3.0, "anybody survived?"},
	{FLT_MAX, "R.I.P."},
};

#define NRATING 10
static rating_t acf_rating[NRATING];

static rating_t *rating = std_rating;
static int window_width = STD_WINDOW_WIDTH;

static char xpdir[512] = { 0 };
const char *psep;

static airportdb_t airportdb;
static list_t *near_airports;
static float arpt_last_reload;

/* will be set on transitioning into the rwy_bbox, if set then reloading is paused */
static const runway_t *landing_rwy;
static int landing_rwy_end;
static double landing_cross_height;
static double landing_dist = -1;
static double landing_cl_delta, landing_cl_angle;


static FILE* getConfigFile(char *mode)
{
    char path[512];
    
    XPLMGetPrefsPath(path);
    XPLMExtractFileAndPath(path);
    strcat(path, psep);
    strcat(path, "xgs.prf");
    return fopen(path, mode);
}


static void saveConfig()
{
    FILE *f;
    
    f = getConfigFile("w");
    if (! f)
        return;
    
    fprintf(f, "%i %i %i", winPosX, winPosY, logEnabled);
    
    fclose(f);
}


static void loadConfig()
{
    FILE *f;
    
    f = getConfigFile("r");
    if (! f)
        return;
    
    fscanf(f, "%i %i %i", &winPosX, &winPosY, &logEnabled);

    fclose(f);
}



static void updateLogItemState()
{
    XPLMCheckMenuItem(xgsMenu, enableLogItem, 
        logEnabled ? xplm_Menu_Checked : xplm_Menu_Unchecked);
}


static void xgsMenuCallback(void *menuRef, void *param)
{
    logEnabled = ! logEnabled;
    updateLogItemState();
}


PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc)
{
    XPLMMenuID pluginsMenu;
    int subMenuItem;
	
 	/* Always use Unix-native paths on the Mac! */
	XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
	log_init(XPLMDebugString, "xgs");
	
    psep = XPLMGetDirectorySeparator();
	XPLMGetSystemPath(xpdir);
	
    loadConfig();
    strcpy(outName, "Landing Speed " VERSION);
    strcpy(outSig, "babichev.landspeed - hotbso");
    strcpy(outDesc, "A plugin that shows vertical landing speed.");

    vertSpeedRef = XPLMFindDataRef("sim/flightmodel/position/vh_ind");
    gearKoofRef = XPLMFindDataRef("sim/flightmodel/forces/faxil_gear");
    flightTimeRef = XPLMFindDataRef("sim/time/total_flight_time_sec");
	icaoRef = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    craftNumRef = XPLMFindDataRef("sim/aircraft/view/acf_tailnum");
    gForceRef = XPLMFindDataRef("sim/flightmodel2/misc/gforce_normal");

	dr_find(&lat_dr, "sim/flightmodel/position/latitude");
    dr_find(&lon_dr, "sim/flightmodel/position/longitude");
    dr_find(&y_agl_dr, "sim/flightmodel/position/y_agl");
	dr_find(&hdg_dr, "sim/flightmodel/position/true_psi");

    memset(landMsg, 0, sizeof(landMsg));
    XPLMRegisterFlightLoopCallback(gameLoopCallback, 0.05f, NULL);
    
    pluginsMenu = XPLMFindPluginsMenu();
    subMenuItem = XPLMAppendMenuItem(pluginsMenu, "Landing Speed", NULL, 1);
    xgsMenu = XPLMCreateMenu("Landing Speed", pluginsMenu, subMenuItem, 
                xgsMenuCallback, NULL);     
    enableLogItem = XPLMAppendMenuItem(xgsMenu, "Enable Log", NULL, 1);
    updateLogItemState();
    
	char cache_path[512];
	sprintf(cache_path, "%s%sOutput%scaches%sXGS.cache", xpdir, psep, psep, psep);
	airportdb_create(&airportdb, xpdir, cache_path);
	if (!recreate_cache(&airportdb)) {
		logMsg("recreate_cache failed\n");
		goto error;
	}
	
    return 1;
	
  error:
	return 0;
}


static void trim(char *str)
{
    int len = strlen(str);
    len--;
    while (0 < len) {
        if (('\r' == str[len]) || ('\n' == str[len])) {
            str[len] = 0;
            len--;
        } else
            return;
    }
}


static void writeLandingToLog()
{
    FILE *f;
    char buf[512];
    
    logThisLanding = 0;
    XPLMGetSystemPath(buf);
    strcat(buf, "landing.log");

    f = fopen(buf, "a");
    if (! f) return;

    strcpy(buf, ctime(&landingTime));
    trim(buf);
    fprintf(f, "%s %s %s %s '%s' %.3f m/s %.0f fpm %.3f G %s\n", buf, logAircraftIcao, logAircraftNum,
                logAirportId, logAirportName, landingSpeed, 
                landingSpeed * 60.0f * 3.2808f, landingG, 
                landMsg[0]);

    fclose(f);
}


static void closeEventWindow()
{
    if (logThisLanding)
        writeLandingToLog();
    
    if (gWindow) {
        XPLMDestroyWindow(gWindow);
        gWindow = NULL;
    }
    
    landingSpeed = 0.0f;
    landingG = 0.0f;
    memset(landMsg, 0, sizeof(landMsg));
    remainingShowTime = 0.0f;
	
	landing_rwy = NULL;
	landing_dist = -1;
}

PLUGIN_API void	XPluginStop(void)
{
    closeEventWindow();
    saveConfig();
}


PLUGIN_API void XPluginDisable(void)
{
}


PLUGIN_API int XPluginEnable(void)
{
	return 1;
}


PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho,
                long inMessage,	void *inParam)
{
	switch (inMessage) {
		case XPLM_MSG_PLANE_LOADED:
			if (inParam == 0) {
				char acf_path[512];
				char acf_file[256];
				
				rating = std_rating;
				window_width = STD_WINDOW_WIDTH;
				
				XPLMGetNthAircraftModel(XPLM_USER_AIRCRAFT, acf_file, acf_path);
				
				char *s = strrchr(acf_path, psep[0]);
				if (NULL != s) {
					strcpy(s+1, "xgs_rating.cfg");
					logMsg("trying config file %s\n", acf_path);
					
					FILE *f = fopen(acf_path, "r");
					
					if (f) {
						char line[200];
						
						int i = 0;
						int firstLine = 1;
						
						while (fgets(line, sizeof line, f) && i < NRATING) {
							if (line[0] == '#') continue;
							line[sizeof(line) -1 ] = '\0';
							trim(line);
							if ('\0' == line[0])
								continue;
							
							if (firstLine) {
								firstLine = 0;
								if (0 == strcmp(line, "V30")) {
									continue;	/* the only version currently supported */
								} else {
									logMsg("Config file does not start with version number\n");
									break;
								}
							}
							
							char *s2 = NULL;
							char *s1 = strchr(line, ';');
							if (s1) {
								*s1++ = '\0';
								s2 = strchr(s1, ';');
							}
							if (NULL == s1 || NULL == s2) {
								logMsg("ill formed line -> %s\n", line);
								break;
							}
							
							s2++;
							
							float v_ms = atof(line);
							float v_fpm = atof(s1);
							logMsg("%f, %f, <%s>\n", v_ms, v_fpm, s2);
							
							s2 = strncpy(acf_rating[i].txt, s2, sizeof(acf_rating[i].txt));
							acf_rating[i].txt[ sizeof(acf_rating[i].txt) -1 ] = '\0';
												
							if (v_ms > 0) {
								acf_rating[i].limit = v_ms;
							} else if (v_fpm > 0) {
								acf_rating[i].limit = v_fpm / MS_2_FPM;
							} else {
								acf_rating[i].limit = FLT_MAX;
								break;
							}
							i++;
						}
						
						if (i < NRATING && FLT_MAX == acf_rating[i].limit) {
							rating = acf_rating;
						} else {
							logMsg("Invalid config file\n");
						}
						
						fclose(f);
					}
					
					
				}
			
					
			}
		break;
	}
}

void drawWindowCallback(XPLMWindowID inWindowID, void *inRefcon)
{
    int i;

    if (0.0f < remainingShowTime) {
        int left, top, right, bottom;
        float color[] = { 1.0, 1.0, 1.0 }; 	/* RGB White */
            
        XPLMGetWindowGeometry(inWindowID, &left, &top, &right, &bottom);
        XPLMDrawTranslucentDarkBox(left, top, right, bottom);
        for (i = 0; i < 7; i++) {
			if ('\0' == landMsg[i][0])
				break;
			
            XPLMDrawString(color, left + 5, top - 20 - i*15, 
                    landMsg[i], NULL, xplmFont_Basic);
        }
		
        glDisable(GL_TEXTURE_2D);
        glColor3f(0.7f, 0.7f, 0.7f);
        glBegin(GL_LINES);
          glVertex2i(right - 1, top - 1);
          glVertex2i(right - 7, top - 7);
          glVertex2i(right - 7, top - 1);
          glVertex2i(right - 1, top - 7);
        glEnd();
        glEnable(GL_TEXTURE_2D);
    }
}                                   

static int mouseCallback(XPLMWindowID inWindowID, int x, int y,    
                   XPLMMouseStatus inMouse, void *inRefcon)
{
    if (windowCloseRequest)
        return 1;
    
    switch (inMouse) {
        case xplm_MouseDown:
            if ((x >= winPosX + window_width - 8) && (x <= winPosX + window_width) && 
                        (y <= winPosY) && (y >= winPosY - 8))
                windowCloseRequest = 1;
            else {
                lastMouseX = x;
                lastMouseY = y;
            }
            break;
            
        case xplm_MouseDrag:
            winPosX += x - lastMouseX;
            winPosY += y - lastMouseY;
            XPLMSetWindowGeometry(gWindow, winPosX, winPosY, 
                    winPosX + window_width, winPosY - WINDOW_HEIGHT);
            lastMouseX = x;
            lastMouseY = y;
            break;
            
        case xplm_MouseUp:
            break;
    }

    return 1;
}

static void keyboardCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags,    
                   char inVirtualKey, void *inRefcon, int losingFocus)
{
}

static int getCurrentState()
{
    return 0.0f != XPLMGetDataf(gearKoofRef) ? STATE_LAND : STATE_AIR;
}


static int printLandingMessage(float vy, float g)
{
	ASSERT(NULL != landing_rwy);

	int w_width = STD_WINDOW_WIDTH;
		
	/* rating terminates with FLT_MAX */
	int i = 0;
	while (vy > rating[i].limit) i++;
	
    strcpy(landMsg[0], rating[i].txt);
	w_width = MAX(w_width, (int)(10 + ceil(XPLMMeasureString(xplmFont_Basic, landMsg[0], strlen(landMsg[0])))));

    sprintf(landMsg[1], "Vy: %.0f fpm / %.2f m/s", vy * MS_2_FPM, vy);
    sprintf(landMsg[2], "G:  %.3f m/s²", g);
	if (NULL != landing_rwy) {
		sprintf(landMsg[3], "Threshold %s/%s", landing_rwy->arpt->icao, landing_rwy->ends[landing_rwy_end].id);
		sprintf(landMsg[4], "Above:    %.f ft / %.f m", landing_cross_height * M_2_FT, landing_cross_height);
		sprintf(landMsg[5], "Distance: %.f ft / %.f m", landing_dist * M_2_FT, landing_dist);
		sprintf(landMsg[6], "from CL:  %.f ft / %.f m / %.1f°",
							landing_cl_delta * M_2_FT, landing_cl_delta, landing_cl_angle);
		w_width = MAX(w_width, (int)(10 + ceil(XPLMMeasureString(xplmFont_Basic, landMsg[6], strlen(landMsg[6])))));

	} else {
		strcpy(landMsg[3], "Not on a runway!");
		landMsg[4][0] = '\0';
	}
	
	return w_width;
}


static void prepareToLog()
{
    int num;
    float lat = dr_getf(&lat_dr);
    float lon = dr_getf(&lon_dr);
    XPLMNavRef ref = XPLMFindNavAid(NULL, NULL, &lat, &lon, NULL, xplm_Nav_Airport);
    
    if (XPLM_NAV_NOT_FOUND != ref)
        XPLMGetNavAidInfo(ref, NULL, &lat, &lon, NULL, NULL, NULL, logAirportId, 
                logAirportName, NULL);
    else {
        logAirportId[0] = 0;
        logAirportName[0] = 0;
    }
    landingTime = time(NULL);
    num = XPLMGetDatab(craftNumRef, logAircraftNum, 0, 49);
    logAircraftNum[num] = '\0';
	
	num = XPLMGetDatab(icaoRef, logAircraftIcao, 0, 39);
    logAircraftIcao[num] = '\0';
    logThisLanding = 1;
}


static void updateLandingResult()
{
    int changed = 0;

    if (landingSpeed < lastVSpeed) {
        landingSpeed = lastVSpeed;
        changed = 1;
    }

    if (landingG < lastG) {
        landingG = lastG;
        changed = 1;
    }

    if (changed || landMsg[0][0]) {
        int w = printLandingMessage(landingSpeed, landingG);
		if (w > window_width) {
			window_width = w;
			XPLMSetWindowGeometry(gWindow, winPosX, winPosY, 
                    winPosX + window_width, winPosY - WINDOW_HEIGHT);
		}	
	}
}


static void createEventWindow()
{
    updateLandingResult();
    remainingShowTime = 60.0f;
    remainingUpdateTime = 1.0f;
    if (! gWindow)
        gWindow = XPLMCreateWindow(winPosX, winPosY, 
                    winPosX + window_width, winPosY - WINDOW_HEIGHT, 
                    1, drawWindowCallback, keyboardCallback, 
                    mouseCallback, NULL);
    if (logEnabled && (! logThisLanding))
        prepareToLog();
}

static void get_near_airports()
{
	ASSERT(NULL == landing_rwy);
	
	if (near_airports)
		free_nearest_airport_list(near_airports);
	
	geo_pos2_t my_pos;
	my_pos = GEO_POS2(dr_getf(&lat_dr), dr_getf(&lon_dr));
	load_nearest_airport_tiles(&airportdb, my_pos);
	unload_distant_airport_tiles(&airportdb, my_pos);

	near_airports = find_nearest_airports(&airportdb, my_pos);
}

/*
 * Catch the transition into the rwy_bbox of the nearest threshold.
 *
 */
static void fix_landing_rwy()
{
	/* have it already */
	if (landing_rwy)
		return;
	
	double thresh_dist_min = 1.0E12;

	float lat = dr_getf(&lat_dr);
	float lon = dr_getf(&lon_dr);
	float hdg = dr_getf(&hdg_dr);

	int in_rwy_bb = 0;
	const airport_t *min_arpt;
	const runway_t *min_rwy;
	int min_end;
	
	ASSERT(NULL != near_airports);
	
	/* loop over all runway ends */
	for (const airport_t *arpt = list_head(near_airports);
		arpt != NULL; arpt = list_next(near_airports, arpt)) {
		ASSERT(arpt->load_complete);
		
		vect2_t pos_v = geo2fpp(GEO_POS2(lat, lon), &arpt->fpp);
		
		for (const runway_t *rwy = avl_first(&arpt->rwys); rwy != NULL;
			rwy = AVL_NEXT(&arpt->rwys, rwy)) {
			
			if (point_in_poly(pos_v, rwy->rwy_bbox)) {
				for (int e = 0; e <=1; e++) {
					const runway_end_t *rwy_end = &rwy->ends[e];
					double rhdg = fabs(rel_hdg(hdg, rwy_end->hdg));
					if (rhdg > 20)
						continue;
					
					vect2_t thr_v = rwy_end->dthr_v;
					double dist = vect2_abs(vect2_sub(thr_v, pos_v));
					if (dist < thresh_dist_min) {
						thresh_dist_min = dist;
						in_rwy_bb = 1;
						min_arpt = arpt;
						min_rwy = rwy;
						min_end = e;
					}
				}
			}
		}
	}
		
	if (in_rwy_bb) {
		landing_rwy = min_rwy;
		landing_rwy_end = min_end;
		landing_cross_height = dr_getf(&y_agl_dr);
		logMsg("fix runway airport: %s, runway: %s, distance: %0.0f\n",
			   min_arpt->icao, landing_rwy->ends[landing_rwy_end].id, thresh_dist_min);
	}
}


static float gameLoopCallback(float inElapsedSinceLastCall,
                float inElapsedTimeSinceLastFlightLoop, int inCounter,    
                void *inRefcon)
{
    int state = getCurrentState();
    float timeFromStart = XPLMGetDataf(flightTimeRef);
	float loop_delay = 0.05f;
	
	float height = dr_getf(&y_agl_dr);

	if (STATE_AIR == state) {
		
		/* low, alert mode */
		if (height < 150) {
			if (NULL == landing_rwy) {
				if (arpt_last_reload + 10.0 < timeFromStart) {
					arpt_last_reload = timeFromStart;
					get_near_airports();
				}
				
				if (NULL != near_airports) {
					loop_delay = 0.01f;		/* increase resolution */
					fix_landing_rwy();
				}
			}
		} else if (height > 200) {
			landing_rwy = NULL;		/* may a go around */
		}
		
		if (height > 500)
			loop_delay = 1.0f;		/* we can be lazy */
	}
	
    if (3.0 < timeFromStart) {
        if (windowCloseRequest) {
            windowCloseRequest = 0;
            closeEventWindow();
        } else if (0.0 < remainingUpdateTime) {
			loop_delay = -1.0; /* get high resolution in touch down phase*/
            updateLandingResult();
            remainingUpdateTime -= inElapsedSinceLastCall;
            if (0.0 > remainingUpdateTime)
                remainingUpdateTime = 0.0;
        }
        if (0.0f < remainingShowTime) {
            remainingShowTime -= inElapsedSinceLastCall;
            if (0.0f >= remainingShowTime)
                closeEventWindow();
        }

        if (STATE_AIR == lastState) {
			float yAgl = dr_getf(&y_agl_dr);
			if (yAgl < 1.0)
				loop_delay = -1.0;
			
			if (STATE_LAND == state) {
				/* catch only first TD, i.e. no bouncing,
				   landing_rwy can be NULL here after a teleportation or when not landing on a rwy */
				if (landing_dist <= 0 && NULL != landing_rwy) {
					float lat = dr_getf(&lat_dr);
					float lon = dr_getf(&lon_dr);
					
					vect2_t pos_v = geo2fpp(GEO_POS2(lat, lon), &landing_rwy->arpt->fpp);
					const runway_end_t *near_end = &landing_rwy->ends[landing_rwy_end];
					const runway_end_t *far_end = &landing_rwy->ends[(0 == landing_rwy_end ? 1 : 0)];
				
					vect2_t center_line_v = vect2_sub(far_end->dthr_v, near_end->dthr_v);
					vect2_t my_v = vect2_sub(pos_v, near_end->dthr_v);
					landing_dist = vect2_abs(my_v);
					double cl_len = vect2_abs(center_line_v);
					if (cl_len > 0) {
						vect2_t cl_unit_v = vect2_scmul(center_line_v, 1/cl_len);
					
						double dprod = vect2_dotprod(cl_unit_v, my_v);
						vect2_t p_v = vect2_scmul(cl_unit_v, dprod);			
						vect2_t dev_v = vect2_sub(my_v, p_v);
						
						/* get signed deviation, + -> right, - -> left */
						landing_cl_delta = vect2_abs(dev_v);
						double xprod_z = my_v.x * cl_unit_v.y - my_v.y * cl_unit_v.x;
						/* by sign of cross product */
						landing_cl_delta = xprod_z > 0 ? landing_cl_delta : -landing_cl_delta;
						
						/* angle between cl and my heading */
						landing_cl_angle = rel_hdg(dr_getf(&hdg_dr), near_end->hdg);
					}
				}
				
				createEventWindow();
			}
        }
    }

    lastVSpeed = fabs(XPLMGetDataf(vertSpeedRef));
    lastG = fabs(XPLMGetDataf(gForceRef));
    lastState = state;
    return loop_delay;
}

