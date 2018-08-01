/*
Copyright <2018> <Elliott Wen>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/




#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <xcb/xcb.h>
#include <xcb/present.h>
#include <xcb/dri3.h>
#include <xcb/xcb_icccm.h>
#include <unistd.h>
#include <errno.h>

 #include <pulse/simple.h>
#include "main.h"

static int running = 1;
static int graphic_server_fd = -1;
static int mouse_server_fd = -1;
static int mouse_client_fd = -1;
static int keyboard_server_fd = -1;
static int keyboard_client_fd = -1;
static int audio_play_server_fd = -1;
static int gps_server_fd = -1;
static pthread_t fetch_image_thread;
static pthread_t mouse_thread;
static pthread_t keyboard_thread;
static pthread_t audio_play_thread;
static pthread_t gps_thread;
static xcb_connection_t *connection;
static xcb_window_t window;
#define TITLE "ANDROID"

static int prepare_pos_str(char *buf)
{
	#define LATI 24.024457
	#define LONGI 115.984640
	#define ELEVATION 170
	#define NSAT 6
	int strc = 0;
	int inrc = 0;
	int      deg, min;
    char     hemi;
    int      hh = 0, mm = 0, ss = 0;
    double latitude = LATI + (rand() % 10) * 0.0001;
    double longitude = LONGI + (rand() % 10) * 0.0001;
    double metersElevation = ELEVATION;
    int nSatellites = nSatellites;
    struct timeval time;
	gettimeofday(&time, NULL);
    hh = (int) (time.tv_sec / (60 * 60)) % 24;
    mm = (int) (time.tv_sec /  60      ) % 60;
    ss = (int) (time.tv_sec            ) % 60;

    inrc = snprintf(buf + strc, 1023 - strc, "$GPGGA,%02d%02d%02d", hh, mm, ss);
    strc += inrc;

    hemi = 'N';
    if (latitude < 0) {
        hemi = 'S';
        latitude = -latitude;
    }
    deg = (int) latitude;
    latitude = 60*(latitude - deg);
    min = (int) latitude;
    latitude = 10000*(latitude - min);

    inrc = snprintf(buf + strc, 1023 - strc, ",%02d%02d.%04d,%c", deg, min, (int)latitude, hemi);
    strc += inrc;

    hemi = 'E';
    if (longitude < 0) {
        hemi = 'W';
        longitude  = -longitude;
    }
    deg = (int) longitude;
    longitude = 60*(longitude - deg);
    min = (int) longitude;
    longitude = 10000*(longitude - min);

    inrc = snprintf(buf + strc, 1023 - strc, ",%02d%02d.%04d,%c", deg, min, (int)longitude, hemi);
    strc += inrc;


    inrc = snprintf(buf + strc, 1023 - strc, ",1,6,");
    strc += inrc;

    inrc = snprintf(buf + strc, 1023 - strc, ",%.1g,M,0.,M", metersElevation);
    strc += inrc;

    inrc = snprintf(buf + strc, 1023 - strc, ",,,*47\n" );
    strc += inrc;

    return strc;

}

static void handle_gps_client_request(int clientfd)
{
	
 

	while(running) //#wait until connection is down
  	{
  		
  		char buf[1024];
  		memset(buf, 0, 1024);
  		int count = prepare_pos_str(buf);
  		//printf("%s", buf);
  		int byteR = send(clientfd, buf, count, MSG_NOSIGNAL);
  		if(byteR != count)
  		{
  			close(clientfd);
  			break;
  		}
  		usleep(10000000);
  	}
  	printf("GPS Connection Lost\n");
}

static void *gps_thread_fn(void *ptr)
{
	while(running)
	{
		int client_addr_len = 0;
	    struct sockaddr_un client_addr;
	    memset(&client_addr, 0 ,sizeof(struct sockaddr_un));
	    int clientfd = accept(gps_server_fd,  (struct sockaddr*) (&client_addr), (socklen_t*)&client_addr_len);
	    if(clientfd <= 0)
	    {
	        fprintf(stderr, "socket listen error %d\n", errno); 
	        running = 0;
	    }
	    else
	    {
	    	printf("GPS Connection Got\n");
	        handle_gps_client_request(clientfd);
	    }
	}
	return NULL;
}




static void handle_client_fb_request(int clientfd)
{
    int prime_fd = -1;
    char reply = 0;
    prime_fd = _recvfd(clientfd);
    if (prime_fd <= 0)
    {
        printf("Failed to receive\n");
        close(clientfd);
        return;
    }
   
    send(clientfd,  &reply, sizeof(reply), 0);
    close(clientfd);

    xcb_pixmap_t pixmap = xcb_generate_id(connection);
                        
                  

                        

                        //printf("prime_fd %d %d %d\n", prime_fd, dupFd, pixmap);

                        xcb_dri3_pixmap_from_buffer(connection, pixmap, window, XHEIGHT * XSTRIDE, XWIDTH, XHEIGHT, XSTRIDE, 32, 32, prime_fd);

                        //printf("ready to present");
                        xcb_present_pixmap(connection,
                           window, pixmap,
                           0, /* sbc */
                           0, /* valid */
                           0, /* update */
                           0, /* x_off */
                           0, /* y_off */
                           0,
                           0, /* wait fence */
                           0,
                           XCB_PRESENT_OPTION_NONE,
                           0, /* target msc */
                           0, /* divisor */
                           0, /* remainder */
                           0, NULL);
                        
                        xcb_free_pixmap(connection, pixmap);

                   		xcb_flush(connection);
                        close(prime_fd);
    
}

static void *fetch_image_thread_fn(void *x_void_ptr)
{

	while(running)
	{
		int client_addr_len = 0;
	    struct sockaddr_un client_addr;
	    memset(&client_addr, 0 ,sizeof(struct sockaddr_un));
	    int clientfd = accept(graphic_server_fd,  (struct sockaddr*) (&client_addr), (socklen_t*)&client_addr_len);
	    if(clientfd <= 0)
	    {
	        fprintf(stderr, "socket listen error %d\n", errno); 
	        running = 0;
	    }
	    else
	    {
	        handle_client_fb_request(clientfd);
	    }
	}
	
	return NULL;
}


static void handle_client_audio_play_request(int clientfd)
{

	pa_simple *s = NULL;;
	pa_sample_spec ss;
	ss.format = PA_SAMPLE_S16NE;
	ss.channels = 2;
	ss.rate = 44100;
	s = pa_simple_new(NULL,               // Use the default server.
	                  "Fooapp",           // Our application's name.
	                  PA_STREAM_PLAYBACK,
	                  NULL,               // Use the default device.
	                  "Music",            // Description of our stream.
	                  &ss,                // Our sample format.
	                  NULL,               // Use default channel map
	                  NULL,               // Use default buffering attributes.
	                  NULL               // Ignore error code.
	                  );

	if(s == NULL)
	{
		printf("Failed to create audio system\n");
		close(clientfd);
		return;
	}
	
	#define AUDIO_PLAY_BUFFER 4800
	char buf[AUDIO_PLAY_BUFFER];
	while(running) //#wait until connection is down
  	{
  		
  		memset(buf, 0, AUDIO_PLAY_BUFFER);
  		int byteR = recv(clientfd, buf, AUDIO_PLAY_BUFFER, 0);
  		//printf("Playing\n");
  		if(byteR <= 0)
  		{
  			printf("Audio play close\n");
  			break;
  		}
  		else
  		{
  	// 		float latency = pa_simple_get_latency(s, NULL);
			
			// if(latency <= 100000)
			// {
  			pa_simple_write(s, buf, byteR, NULL);
  				
			// }
			// else
			// {
			// 	printf("We drop it latency %f\n", latency);
				
			// }
  			//pa_simple_drain(s, NULL);
  			send(clientfd, buf, 1, 0);
  			//pa_simple_drain(s, NULL);
  			
  			//Play it
  		}
  	}
  	close(clientfd);
  	if(s != NULL)
  	pa_simple_free(s);
}

static void *audio_play_thread_fn(void *ptr)
{
	while(running)
	{
		int client_addr_len = 0;
	    struct sockaddr_un client_addr;
	    memset(&client_addr, 0 ,sizeof(struct sockaddr_un));
	    int clientfd = accept(audio_play_server_fd,  (struct sockaddr*) (&client_addr), (socklen_t*)&client_addr_len);
	    if(clientfd <= 0)
	    {
	        fprintf(stderr, "socket listen error %d\n", errno); 
	        running = 0;
	    }
	    else
	    {
	        handle_client_audio_play_request(clientfd);
	    }
	}
	return NULL;
}

static void handle_mouse_request(int clientfd)
{
	struct DeviceInfo mouseinfo;
	memset(&mouseinfo, 0, sizeof(struct DeviceInfo)); 
	strncpy(mouseinfo.name, "raccoon-pointer", 32);
	strncpy(mouseinfo.physical_location, "none", 32);
	mouseinfo.driver_version = 1;
	mouseinfo.id.bustype = BUS_VIRTUAL;
  	mouseinfo.id.product = 2;
  	mouseinfo.id.vendor = 2;
  	mouseinfo.id.version = 2;
  	set_bit(mouseinfo.key_bitmask, BTN_MOUSE);
  	set_bit(mouseinfo.rel_bitmask, REL_X);
  	set_bit(mouseinfo.rel_bitmask, REL_Y);
  	set_bit(mouseinfo.rel_bitmask, REL_HWHEEL);
  	set_bit(mouseinfo.rel_bitmask, REL_WHEEL);
  	send(clientfd, &mouseinfo, sizeof(mouseinfo), 0);
  	mouse_client_fd = clientfd;
  	printf("Mouse Plug\n");
  	while(running) //#wait until connection is down
  	{
  		char buf[1];
  		int byteR = recv(mouse_client_fd, buf, 1, 0);
  		if(byteR <= 0)
  		{
  			close(mouse_client_fd);
  			mouse_client_fd = -1;
  			break;
  		}
  	}
  	printf("Mouse Connection Lost\n");
}

static void *mouse_handle_thread_fn(void *ptr)
{
	while(running)
	{
		int client_addr_len = 0;
	    struct sockaddr_un client_addr;
	    memset(&client_addr, 0 ,sizeof(struct sockaddr_un));
	    int clientfd = accept(mouse_server_fd,  (struct sockaddr*) (&client_addr), (socklen_t*)&client_addr_len);
	    if(clientfd <= 0)
	    {
	        fprintf(stderr, "socket listen error %d\n", errno); 
	        running = 0;
	    }
	    else
	    {
	        handle_mouse_request(clientfd);
	    }
	}
	
	return NULL;
}

static void handle_keyboard_request(int clientfd)
{
	struct DeviceInfo info;
	memset(&info, 0, sizeof(struct DeviceInfo)); 
	strncpy(info.name, "raccoon-keyboard", 32);
	strncpy(info.physical_location, "none", 32);
	info.driver_version = 1;
	info.id.bustype = BUS_VIRTUAL;
  	info.id.product = 3;
  	info.id.vendor = 3;
  	info.id.version = 3;
  	set_bit(info.key_bitmask, BTN_MISC);
  	set_bit(info.rel_bitmask, KEY_OK);
  	
  	send(clientfd, &info, sizeof(info), 0);
  	keyboard_client_fd = clientfd;
  	printf("Keyboard Plug\n");
  	//#Indefinite Wait
  	while(running)
  	{
  		char buf[1];
  		int byteR = recv(keyboard_client_fd, buf, 1, 0);
  		if(byteR <= 0)
  		{
  			close(keyboard_client_fd);
  			keyboard_client_fd = -1;
  			break;
  		}
  	}
  	printf("Keyboard Connection Lost\n");
}



static void *keyboard_handle_thread_fn(void *ptr)
{
	while(running)
	{
		int client_addr_len = 0;
	    struct sockaddr_un client_addr;
	    memset(&client_addr, 0 ,sizeof(struct sockaddr_un));
	    int clientfd = accept(keyboard_server_fd,  (struct sockaddr*) (&client_addr), (socklen_t*)&client_addr_len);
	    if(clientfd <= 0)
	    {
	        fprintf(stderr, "socket listen error %d\n", errno); 
	        running = 0;
	    }
	    else
	    {
	        handle_keyboard_request(clientfd);
	    }

	}
	
	return NULL;
}
	





static int init_workspace_dir()
{
	if(_mkdir_dir(MAINWORKDIR) != 0)
	{
		return -1;
	}

	if(_mkdir_dir(INPUTWORKDIR) != 0)
	{
		return -1;
	}

	return 0;
}

static int init_network_connections()
{
	graphic_server_fd = _create_unix_server(OPENGL_SOCKET);
	if(graphic_server_fd <= 0)
		return -1;

	mouse_server_fd = _create_unix_server(MOUSE_SOCKET);
	if(mouse_server_fd <= 0)
		return -1;

	keyboard_server_fd = _create_unix_server(KEYBOARD_SOCKET);
	if(keyboard_server_fd <= 0)
		return -1;


	audio_play_server_fd = _create_unix_server(AUDIO_PLAY_SOCKET);
	if(audio_play_server_fd <= 0)
		return -1;

	gps_server_fd = _create_unix_server(GPS_SOCKET);
	if(gps_server_fd <= 0)
		return -1;

	return 0;

}

static void sendEvents(int fd, std::vector<CompatEvent> &eventCache)
{
    char *buf = (char*)malloc(sizeof(CompatEvent) * eventCache.size());

    for(int i = 0; i < eventCache.size(); i ++)
    {
        memcpy(buf + i * sizeof(CompatEvent), &eventCache[i], sizeof(CompatEvent));
    }
    send(fd, buf, sizeof(CompatEvent) * eventCache.size(), 0);
    free(buf);
}

static int init_x11()
{
	connection = xcb_connect(NULL, NULL);

    if (xcb_connection_has_error(connection)) {
        fprintf(stderr, "ERROR: failed to connection to X server\n");
        return -1;
    }

    const xcb_setup_t *setup = xcb_get_setup(connection);
    xcb_screen_t *screen = xcb_setup_roots_iterator(setup).data;

    xcb_depth_iterator_t depth_iter = xcb_screen_allowed_depths_iterator(screen);
    xcb_depth_t *depth = NULL;

    while (depth_iter.rem) {
        if (depth_iter.data->depth == 32 && depth_iter.data->visuals_len) {
            depth = depth_iter.data;
            break;
        }
        xcb_depth_next(&depth_iter);
    }

    if (!depth) {
        fprintf(stderr, "ERROR: screen does not support 32 bit color depth\n");
        xcb_disconnect(connection);
        return -1;
    }

    xcb_visualtype_iterator_t visual_iter = xcb_depth_visuals_iterator(depth);
    xcb_visualtype_t *visual = NULL;


    while (visual_iter.rem) {
        if (visual_iter.data->_class == XCB_VISUAL_CLASS_TRUE_COLOR) {
        	//printf("%d %d %d\n", visual_iter.data->red_mask, visual_iter.data->green_mask, visual_iter.data->blue_mask);
            visual = visual_iter.data;
            break;
        }
        xcb_visualtype_next(&visual_iter);
    }




    if (!visual) {
        fprintf(stderr, "ERROR: screen does not support True Color\n");
        xcb_disconnect(connection);
        return -1;
    }

    xcb_colormap_t colormap = xcb_generate_id(connection);
    xcb_void_cookie_t cookie = xcb_create_colormap(
            connection,
            XCB_COLORMAP_ALLOC_NONE,
            colormap,
            screen->root,
            visual->visual_id);

    xcb_generic_error_t *error = xcb_request_check(connection, cookie);
    if (error) {
        fprintf(stderr, "ERROR: failed to create colormap");
        xcb_disconnect(connection);
        return -1;
    }

    unsigned int cw_mask = XCB_CW_EVENT_MASK | XCB_CW_COLORMAP | XCB_CW_BORDER_PIXEL ;
    unsigned int cw_values[] = { screen->white_pixel, XCB_EVENT_MASK_EXPOSURE       | XCB_EVENT_MASK_BUTTON_PRESS   |
                                    XCB_EVENT_MASK_BUTTON_RELEASE | XCB_EVENT_MASK_POINTER_MOTION |
                                    XCB_EVENT_MASK_ENTER_WINDOW   | XCB_EVENT_MASK_LEAVE_WINDOW   |
                                    XCB_EVENT_MASK_KEY_PRESS      | XCB_EVENT_MASK_KEY_RELEASE ,  colormap };

    window = xcb_generate_id(connection);
    cookie = xcb_create_window_checked(
            connection,
            depth->depth,
            window,
            screen->root,
            0, 0,
            XWIDTH, XHEIGHT,
            0,
            XCB_WINDOW_CLASS_INPUT_OUTPUT,
            visual->visual_id,
            cw_mask,
            cw_values);

    error = xcb_request_check(connection, cookie);
    if (error) {
        fprintf(stderr, "ERROR: failed to create window");
        xcb_disconnect(connection);
        return -1;
    }

    xcb_size_hints_t hints;

    xcb_icccm_size_hints_set_min_size(&hints, XWIDTH, XHEIGHT);
    
    xcb_icccm_size_hints_set_max_size(&hints, XWIDTH, XHEIGHT);

    xcb_icccm_set_wm_size_hints(connection, window, XCB_ATOM_WM_NORMAL_HINTS, &hints);


	 cookie = xcb_change_property(connection, XCB_PROP_MODE_REPLACE, window,
			XCB_ATOM_WM_NAME, XCB_ATOM_STRING, 8, strlen(TITLE),
			TITLE);

    xcb_cursor_t cur = xcb_generate_id (connection);
    xcb_pixmap_t pix = xcb_generate_id (connection);

    xcb_create_pixmap (connection, 1, pix, screen->root, 1, 1);
    xcb_create_cursor (connection, cur, pix, pix, 0, 0, 0, 0, 0, 0, 1, 1);

    xcb_change_window_attributes (connection, window,
                                  XCB_CW_CURSOR, &cur);

    xcb_map_window(connection, window);

    xcb_flush(connection);

    return 0;
}




int main()
{

	if(init_workspace_dir() != 0)
	{
		return -1;
	}
	
	
	if(init_network_connections() != 0)
	{
		return -1;
	}

	if(init_x11() != 0)
	{
		return -1;
	}
    

    

	pthread_create(&mouse_thread, NULL, mouse_handle_thread_fn, 0);

	pthread_create(&keyboard_thread, NULL, keyboard_handle_thread_fn, 0);

	pthread_create(&fetch_image_thread, NULL, fetch_image_thread_fn, 0);

	pthread_create(&audio_play_thread, NULL, audio_play_thread_fn, 0);

	pthread_create(&gps_thread, NULL, gps_thread_fn, 0);

	xcb_generic_event_t *e;
	while(running)
	{
		struct timespec spec;
  		clock_gettime(CLOCK_MONOTONIC, &spec);
  		#define TIMESPEC (unsigned long)(spec.tv_sec), \
 			(unsigned long)(spec.tv_nsec/1000)
        std::vector<CompatEvent> keyEventCache;
        std::vector<CompatEvent> mouseEventCache;
		while ((e = xcb_poll_for_event (connection)) != NULL) {
			    switch (e->response_type & ~0x80) {
			    case XCB_EXPOSE: {
			      xcb_expose_event_t *ev = (xcb_expose_event_t *)e;

			      // printf ("Window %u exposed. Region to be redrawn at location (%d,%d), with dimension (%d,%d)\n",
			      //         ev->window, ev->x, ev->y, ev->width, ev->height);
			      break;
			    }
			    case XCB_BUTTON_PRESS: {
			      xcb_button_press_event_t *ev = (xcb_button_press_event_t *)e;
			      
			      switch (ev->detail) {
			      case 4:
			      	//printf("%d\n", ev->event_x);
			        mouseEventCache.push_back({TIMESPEC, EV_REL, REL_WHEEL, (unsigned)(-3)});
					mouseEventCache.push_back({TIMESPEC, EV_SYN, SYN_REPORT, 0});
			        break;
			      case 5:
			        //printf("%d\n", ev->event_x);
			        mouseEventCache.push_back({TIMESPEC, EV_REL, REL_WHEEL, (unsigned)(3)});
					mouseEventCache.push_back({TIMESPEC, EV_SYN, SYN_REPORT, 0});
			        break;
			      default:
			        mouseEventCache.push_back({TIMESPEC, EV_KEY, BTN_LEFT, 1});
				  	mouseEventCache.push_back({TIMESPEC, EV_SYN, SYN_REPORT, 0});
				  	break;
			      }
			      break;
			    }
			    case XCB_BUTTON_RELEASE: {
			      xcb_button_release_event_t *ev = (xcb_button_release_event_t *)e;
			      
				  mouseEventCache.push_back({TIMESPEC, EV_KEY, BTN_LEFT, 0});
                  mouseEventCache.push_back({TIMESPEC, EV_SYN, SYN_REPORT, 0});
			      // printf ("Button %d released in window %u, at coordinates (%d,%d)\n",
			      //         ev->detail, ev->event, ev->event_x, ev->event_y);
			      break;
			    }
			    case XCB_MOTION_NOTIFY: {
			      xcb_motion_notify_event_t *ev = (xcb_motion_notify_event_t *)e;

			       // printf ("Mouse moved in window %u, at coordinates (%d,%d)\n",
			       //         ev->event, ev->event_x, ev->event_y);
			      			mouseEventCache.push_back({TIMESPEC, EV_ABS, ABS_X, static_cast<unsigned int>(ev->event_x)});
                            mouseEventCache.push_back({TIMESPEC, EV_ABS, ABS_Y, static_cast<unsigned int>(ev->event_y)});
                            mouseEventCache.push_back({TIMESPEC, EV_REL, REL_X, 1});
                            mouseEventCache.push_back({TIMESPEC, EV_REL, REL_Y, 1});
                            mouseEventCache.push_back({TIMESPEC, EV_SYN, SYN_REPORT, 0});
			      break;
			    }
			    case XCB_ENTER_NOTIFY: {
			      xcb_enter_notify_event_t *ev = (xcb_enter_notify_event_t *)e;

			      // printf ("Mouse entered window %u, at coordinates (%d,%d)\n",
			      //         ev->event, ev->event_x, ev->event_y);
			      break;
			    }
			    case XCB_LEAVE_NOTIFY: {
			      xcb_leave_notify_event_t *ev = (xcb_leave_notify_event_t *)e;

			      // printf ("Mouse left window %u, at coordinates (%d,%d)\n",
			      //         ev->event, ev->event_x, ev->event_y);
			      break;
			    }
			    case XCB_KEY_PRESS: {
			      xcb_key_press_event_t *ev = (xcb_key_press_event_t *)e;
			      keyEventCache.push_back({TIMESPEC, EV_KEY, static_cast<unsigned short>(ev->detail - 8), 1});

			      // printf ("Key pressed in window %u %u\n",
			      //         ev->event, ev->detail);
			      break;
			    }
			    case XCB_KEY_RELEASE: {
			      xcb_key_release_event_t *ev = (xcb_key_release_event_t *)e;
			      keyEventCache.push_back({TIMESPEC, EV_KEY, static_cast<unsigned short>(ev->detail - 8), 0});
			      // printf ("Key released in window %u %u\n",
			      //         ev->event, ev->detail);
			      break;
			    }

			    
			    default:
			      /* Unknown event type, ignore it */
			      //printf("Unknown event: %d\n", e->response_type);
			      break;
			    }
			    /* Free the Generic Event */
			    free (e);
	}
			   if(mouseEventCache.size() > 0 && mouse_client_fd> 0)
                {
                    sendEvents(mouse_client_fd, mouseEventCache);
                }

                if(keyEventCache.size() > 0 && keyboard_client_fd > 0)
                {
                    sendEvents(keyboard_client_fd, keyEventCache);
                }
			  usleep(1000);
	}
    


    xcb_disconnect(connection);
    return 0;
}