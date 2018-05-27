#include <float.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <time.h>
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

#define VERSION "3.0b1"

static float gameLoopCallback(float inElapsedSinceLastCall,
                float inElapsedTimeSinceLastFlightLoop, int inCounter,    
                void *inRefcon);
                
#define MS_2_FPM 196.850

#define STATE_LAND 1
#define STATE_AIR 2

#define WINDOW_HEIGHT 80

static XPLMWindowID gWindow = NULL;
static XPLMDataRef yAglRef, vertSpeedRef, gearKoofRef, flightTimeRef, descrRef;
static XPLMDataRef longRef, latRef, craftNumRef, icaoRef;
static XPLMDataRef gForceRef;
static dr_t lat_dr, lon_dr, elevation_dr, hdg_dr;

static char landMsg[4][100];
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

#define STD_WINDOW_WIDTH 130
typedef struct rating_ { float limit; int w_width; char txt[100]; } rating_t;
static rating_t std_rating[] = {
	{0.5, STD_WINDOW_WIDTH, "excellent landing"},
	{1.0, STD_WINDOW_WIDTH, "good landing"},
	{1.5, STD_WINDOW_WIDTH, "acceptable landing"},
	{2.0, STD_WINDOW_WIDTH, "hard landing"},
	{2.5, STD_WINDOW_WIDTH, "you are fired!!!"},
	{3.0, STD_WINDOW_WIDTH, "anybody survived?"},
	{FLT_MAX, STD_WINDOW_WIDTH, "R.I.P."},
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

#ifdef __APPLE__
static int MacToUnixPath(const char * inPath, char * outPath, int outPathMaxLen)
{
    CFStringRef inStr = CFStringCreateWithCString(kCFAllocatorDefault, inPath, kCFStringEncodingMacRoman);
    if (inStr == NULL) return -1;
    CFURLRef url = CFURLCreateWithFileSystemPath(kCFAllocatorDefault, inStr, kCFURLHFSPathStyle,0);
    CFStringRef outStr = CFURLCopyFileSystemPath(url, kCFURLPOSIXPathStyle);
    if (!CFStringGetCString(outStr, outPath, outPathMaxLen, kCFURLPOSIXPathStyle)) return -1;
    CFRelease(outStr);
    CFRelease(url);
    CFRelease(inStr);
    return 0;
}
#endif


static FILE* getConfigFile(char *mode)
{
    char path[512];
    
    XPLMGetPrefsPath(path);
    XPLMExtractFileAndPath(path);
    strcat(path, psep);
    strcat(path, "xgs.prf");

#ifdef __APPLE__
    char unixPath[512];
    MacToUnixPath(path, unixPath, 512);
    
    return fopen(unixPath, mode);
#else
    return fopen(path, mode);
#endif
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
    strcpy(outSig, "babichev.landspeed");
    strcpy(outDesc, "A plugin that shows vertical landing speed.");

    vertSpeedRef = XPLMFindDataRef("sim/flightmodel/position/vh_ind");
	yAglRef = XPLMFindDataRef("sim/flightmodel/position/y_agl");
    gearKoofRef = XPLMFindDataRef("sim/flightmodel/forces/faxil_gear");
    flightTimeRef = XPLMFindDataRef("sim/time/total_flight_time_sec");
    descrRef = XPLMFindDataRef("sim/aircraft/view/acf_tailnum");
	icaoRef = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    craftNumRef = XPLMFindDataRef("sim/aircraft/view/acf_tailnum");
    gForceRef = XPLMFindDataRef("sim/flightmodel2/misc/gforce_normal");

	dr_find(&lat_dr, "sim/flightmodel/position/latitude");
    dr_find(&lon_dr, "sim/flightmodel/position/longitude");
    dr_find(&elevation_dr, "sim/flightmodel/position/elevation");
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
	sprintf(cache_path, "%s%s%s%s%s%s%s", xpdir, psep, "Output", psep, "caches", psep, "XGS.cache");
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

#ifdef __APPLE__
    char unixPath[512];
    MacToUnixPath(buf, unixPath, 512);

    f = fopen(unixPath, "a");
#else
    f = fopen(buf, "a");
#endif
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
				char log_line[1024];
				
				rating = std_rating;
				window_width = STD_WINDOW_WIDTH;
				
				XPLMGetNthAircraftModel(XPLM_USER_AIRCRAFT, acf_file, acf_path);
				
				char *s = strrchr(acf_path, psep[0]);
				if (NULL != s) {
					strcpy(s+1, "xgs_rating.cfg");
					sprintf(log_line, "xgs: trying config file %s\n", acf_path);
					XPLMDebugString(log_line);
					
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
									XPLMDebugString("xgs: Config file does not start with version number\n");
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
								sprintf(log_line, "xgs: ill formed line -> %s\n", line);
								XPLMDebugString(log_line);
								break;
							}
							
							s2++;
							
							float v_ms = atof(line);
							float v_fpm = atof(s1);
							sprintf(log_line, "xgs: %f, %f, <%s>\n", v_ms, v_fpm, s2);
							XPLMDebugString(log_line);
							
							s2 = strncpy(acf_rating[i].txt, s2, sizeof(acf_rating[i].txt));
							acf_rating[i].txt[ sizeof(acf_rating[i].txt) -1 ] = '\0';
							acf_rating[i].w_width = 10 + ceil(XPLMMeasureString(xplmFont_Basic, s2, strlen(s2)));
												
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
							XPLMDebugString("xgs: Invalid config file\n");
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
        for (i = 0; i < 4; i++)
            XPLMDrawString(color, left + 5, top - 20 - i*15, 
                    landMsg[i], NULL, xplmFont_Basic);
        
        glDisable(GL_TEXTURE_2D);
        glColor3f(0.7, 0.7, 0.7);
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
	int i = 0;
	
	/* rating terminates with FLT_MAX */
	while (vy > rating[i].limit) i++;
	
    strcpy(landMsg[0], rating[i].txt);

    sprintf(landMsg[1], "Vy: %.0f fpm", vy * MS_2_FPM);
    sprintf(landMsg[2], "Vy: %.3f m/s", vy);
    sprintf(landMsg[3], "G:  %.3f", g);
	
	return rating[i].w_width;
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
	if (near_airports)
		free_nearest_airport_list(near_airports);
	
	geo_pos2_t my_pos;
	my_pos = GEO_POS2(dr_getf(&lat_dr), dr_getf(&lon_dr));
	load_nearest_airport_tiles(&airportdb, my_pos);
	unload_distant_airport_tiles(&airportdb, my_pos);

	near_airports = find_nearest_airports(&airportdb, my_pos);
	if (near_airports) {
		const airport_t *arpt = list_head(near_airports);

		if (arpt) {
			logMsg("nearest airport: %s\n", arpt->icao);
		}
	}
}

static void nearest_threshold()
{
	float lat = dr_getf(&lat_dr);
	float lon = dr_getf(&lon_dr);
	float hdg = dr_getf(&hdg_dr);
	
	double thresh_dist_min = 1.0E12;
	int in_rwy_bb = 0;
	const airport_t *min_arpt;
	const runway_t *min_rwy;
	int min_end;
	
	if (NULL == near_airports)
		return;
	
	for (const airport_t *arpt = list_head(near_airports);
		arpt != NULL; arpt = list_next(near_airports, arpt)) {
		ASSERT(arpt->load_complete);
		
		vect2_t pos_v = geo2fpp(GEO_POS2(lat, lon), &arpt->fpp);
		
		for (const runway_t *rwy = avl_first(&arpt->rwys); rwy != NULL;
			rwy = AVL_NEXT(&arpt->rwys, rwy)) {
			
			if (point_in_poly(pos_v, rwy->asda_bbox)) {
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
		logMsg("Airport: %s, runway: %s, distance: %0.0f\n", min_arpt->icao, min_rwy->ends[min_end].id, thresh_dist_min);
	}
}

static float last_dist;

static float gameLoopCallback(float inElapsedSinceLastCall,
                float inElapsedTimeSinceLastFlightLoop, int inCounter,    
                void *inRefcon)
{
    int state = getCurrentState();
    float timeFromStart = XPLMGetDataf(flightTimeRef);
	
	if (arpt_last_reload + 10.0 < timeFromStart) {
		arpt_last_reload = timeFromStart;
		get_near_airports();
	}
	
	if (last_dist + 2.0 < timeFromStart) {
		last_dist = timeFromStart;
		nearest_threshold();
	}
	
	float loopDelay = 0.05;
	
    if (3.0 < timeFromStart) {
        if (windowCloseRequest) {
            windowCloseRequest = 0;
            closeEventWindow();
        } else if (0.0 < remainingUpdateTime) {
			loopDelay = -1.0; /* get high resolution in touch down phase*/
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
			float yAgl = XPLMGetDataf(yAglRef);
			if (yAgl < 1.0)
				loopDelay = -1.0;
			
			if (STATE_LAND == state)
				createEventWindow();
			
        }
    }

    lastVSpeed = fabs(XPLMGetDataf(vertSpeedRef));
    lastG = fabs(XPLMGetDataf(gForceRef));
    lastState = state;
    return loopDelay;
}

