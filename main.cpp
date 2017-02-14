#include <iostream>
#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "RaspiPreview.h"
#include "RaspiCamControl.h"

using namespace std;

#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

#define SPLITTER_OUTPUT_PORT 0
#define SPLITTER_PREVIEW_PORT 1

// Video format information
// 0 implies variable
#define VIDEO_FRAME_RATE_NUM 30
#define VIDEO_FRAME_RATE_DEN 1

#define WAIT_METHOD_NONE           0

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

typedef struct RASPIVID_STATE_S RASPIVID_STATE;

typedef enum {
	RAW_OUTPUT_FMT_YUV = 1,
	RAW_OUTPUT_FMT_RGB,
	RAW_OUTPUT_FMT_GRAY,
} RAW_OUTPUT_FMT;

typedef struct{
	FILE *file_handle;                   /// File handle to write buffer data to.
	RASPIVID_STATE *pstate;              /// pointer to our state in case required in callback
	int abort;                           /// Set to 1 in callback if an error occurs to attempt to abort the capture
	char *cb_buff;                       /// Circular buffer
	int   cb_len;                        /// Length of buffer
	int   cb_wptr;                       /// Current write pointer
	int   cb_wrap;                       /// Has buffer wrapped at least once?
	int   cb_data;                       /// Valid bytes in buffer
#define IFRAME_BUFSIZE (60*1000)
	int   iframe_buff[IFRAME_BUFSIZE];          /// buffer of iframe pointers
	int   iframe_buff_wpos;
	int   iframe_buff_rpos;
	char  header_bytes[29];
	int  header_wptr;
	FILE *imv_file_handle;               /// File handle to write inline motion vectors to.
	FILE *raw_file_handle;               /// File handle to write raw data to.
	int  flush_buffers;
	FILE *pts_file_handle;               /// File timestamps
} PORT_USERDATA;

struct RASPIVID_STATE_S{
	int timeout;                        /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
	int width;                          /// Requested width of image
	int height;                         /// requested height of image
	MMAL_FOURCC_T encoding;             /// Requested codec video encoding (MJPEG or H264)
	int bitrate;                        /// Requested bitrate
	int framerate;                      /// Requested frame rate (fps)
	int intraperiod;                    /// Intra-refresh period (key frame rate)
	int quantisationParameter;          /// Quantisation parameter - quality. Set bitrate 0 and set this for variable bitrate
	int bInlineHeaders;                  /// Insert inline headers to stream (SPS, PPS)
	char *filename;                     /// filename of output file
	int verbose;                        /// !0 if want detailed run information
	int demoMode;                       /// Run app in demo mode
	int demoInterval;                   /// Interval between camera settings changes
	int immutableInput;                 /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
	/// the camera output or the encoder output (with compression artifacts)
	int profile;                        /// H264 profile to use for encoding
	int level;                          /// H264 level to use for encoding
	int waitMethod;                     /// Method for switching between pause and capture

	int onTime;                         /// In timed cycle mode, the amount of time the capture is on per cycle
	int offTime;                        /// In timed cycle mode, the amount of time the capture is off per cycle

	int segmentSize;                    /// Segment mode In timed cycle mode, the amount of time the capture is off per cycle
	int segmentWrap;                    /// Point at which to wrap segment counter
	int segmentNumber;                  /// Current segment counter
	int splitNow;                       /// Split at next possible i-frame if set to 1.
	int splitWait;                      /// Switch if user wants splited files

	RASPIPREVIEW_PARAMETERS preview_parameters;   /// Preview setup parameters
	RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

	MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
	MMAL_COMPONENT_T *splitter_component;  /// Pointer to the splitter component
	MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
	MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera or splitter to preview
	MMAL_CONNECTION_T *splitter_connection;/// Pointer to the connection from camera to splitter
	MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder

	MMAL_POOL_T *splitter_pool; /// Pointer to the pool of buffers used by splitter output port 0
	MMAL_POOL_T *encoder_pool; /// Pointer to the pool of buffers used by encoder output port

	PORT_USERDATA callback_data;        /// Used to move data to the encoder callback

	int bCapturing;                     /// State of capture/pause
	int bCircularBuffer;                /// Whether we are writing to a circular buffer

	int inlineMotionVectors;             /// Encoder outputs inline Motion Vectors
	char *imv_filename;                  /// filename of inline Motion Vectors output
	int raw_output;                      /// Output raw video from camera as well
	RAW_OUTPUT_FMT raw_output_fmt;       /// The raw video format
	char *raw_filename;                  /// Filename for raw video output
	int cameraNum;                       /// Camera number
	int settings;                        /// Request settings from the camera
	int sensor_mode;			            /// Sensor mode. 0=auto. Check docs/forum for modes selected by other values.
	int intra_refresh_type;              /// What intra refresh type to use. -1 to not set.
	int frame;
	char *pts_filename;
	int save_pts;
	int64_t starttime;
	int64_t lasttime;

	bool netListen;
};

static void camera_control_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	if (buffer->cmd == MMAL_EVENT_PARAMETER_CHANGED)
	{
		MMAL_EVENT_PARAMETER_CHANGED_T *param = (MMAL_EVENT_PARAMETER_CHANGED_T *)buffer->data;
		switch (param->hdr.id) {
		case MMAL_PARAMETER_CAMERA_SETTINGS:
		{
			MMAL_PARAMETER_CAMERA_SETTINGS_T *settings = (MMAL_PARAMETER_CAMERA_SETTINGS_T*)param;
			vcos_log_error("Exposure now %u, analog gain %u/%u, digital gain %u/%u",
				       settings->exposure,
				       settings->analog_gain.num, settings->analog_gain.den,
				       settings->digital_gain.num, settings->digital_gain.den);
			vcos_log_error("AWB R=%u/%u, B=%u/%u",
				       settings->awb_red_gain.num, settings->awb_red_gain.den,
				       settings->awb_blue_gain.num, settings->awb_blue_gain.den
				);
		}
		break;
		}
	}
	else if (buffer->cmd == MMAL_EVENT_ERROR)
	{
		vcos_log_error("No data received from sensor. Check all connections, including the Sunny one on the camera board");
	}
	else
	{
		vcos_log_error("Received unexpected camera control callback event, 0x%08x", buffer->cmd);
	}

	mmal_buffer_header_release(buffer);
}

static void default_status(RASPIVID_STATE *state)
{
	if (!state)
	{
		vcos_assert(0);
		return;
	}

	// Default everything to zero
	memset(state, 0, sizeof(RASPIVID_STATE));

	// Now set anything non-zero
	state->timeout = 5000;     // 5s delay before take image
	state->width = 1920;       // Default to 1080p
	state->height = 1080;
	state->encoding = MMAL_ENCODING_H264;
	state->bitrate = 17000000; // This is a decent default bitrate for 1080p
	state->framerate = VIDEO_FRAME_RATE_NUM;
	state->intraperiod = -1;    // Not set
	state->quantisationParameter = 0;
	state->demoMode = 0;
	state->demoInterval = 250; // ms
	state->immutableInput = 1;
	state->profile = MMAL_VIDEO_PROFILE_H264_HIGH;
	state->level = MMAL_VIDEO_LEVEL_H264_4;
	state->waitMethod = WAIT_METHOD_NONE;
	state->onTime = 5000;
	state->offTime = 5000;

	state->bCapturing = 0;
	state->bInlineHeaders = 0;

	state->segmentSize = 0;  // 0 = not segmenting the file.
	state->segmentNumber = 1;
	state->segmentWrap = 0; // Point at which to wrap segment number back to 1. 0 = no wrap
	state->splitNow = 0;
	state->splitWait = 0;

	state->inlineMotionVectors = 0;
	state->cameraNum = 0;
	state->settings = 0;
	state->sensor_mode = 0;

	state->intra_refresh_type = -1;

	state->frame = 0;
	state->save_pts = 0;

	state->netListen = false;


	// Setup preview window defaults
	raspipreview_set_defaults(&state->preview_parameters);

	// Set up the camera_parameters to default
	raspicamcontrol_set_defaults(&state->camera_parameters);
}

static MMAL_STATUS_T create_camera_component(RASPIVID_STATE *state)
{
	MMAL_COMPONENT_T *camera = 0;
	MMAL_ES_FORMAT_T *format;
	MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
	MMAL_STATUS_T status;
	MMAL_PARAMETER_INT32_T camera_num;

	/* Create the component */
	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create camera component");
		goto error;
	}

	status = raspicamcontrol_set_stereo_mode(camera->output[0], &state->camera_parameters.stereo_mode);
	status = raspicamcontrol_set_stereo_mode(camera->output[1], &state->camera_parameters.stereo_mode);
	status = raspicamcontrol_set_stereo_mode(camera->output[2], &state->camera_parameters.stereo_mode);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Could not set stereo mode : error %d", status);
		goto error;
	}

	camera_num = {{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, state->cameraNum};

	status = mmal_port_parameter_set(camera->control, &camera_num.hdr);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Could not select camera : error %d", status);
		goto error;
	}

	if (!camera->output_num)
	{
		status = MMAL_ENOSYS;
		vcos_log_error("Camera doesn't have output ports");
		goto error;
	}

	status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, state->sensor_mode);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Could not set sensor mode : error %d", status);
		goto error;
	}

	preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
	video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
	still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

	if (state->settings)
	{
		MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T change_event_request =
			{{MMAL_PARAMETER_CHANGE_EVENT_REQUEST, sizeof(MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T)},
			 MMAL_PARAMETER_CAMERA_SETTINGS, 1};

		status = mmal_port_parameter_set(camera->control, &change_event_request.hdr);
		if ( status != MMAL_SUCCESS )
		{
			vcos_log_error("No camera settings events");
		}
	}

	// Enable the camera, and tell it its control callback function
	status = mmal_port_enable(camera->control, camera_control_callback);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Unable to enable control port : error %d", status);
		goto error;
	}

	//  set up the camera configuration
	{
		MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
			{
				{ MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
				.max_stills_w = state->width,
				.max_stills_h = state->height,
				.stills_yuv422 = 0,
				.one_shot_stills = 0,
				.max_preview_video_w = state->width,
				.max_preview_video_h = state->height,
				.num_preview_video_frames = 3 + vcos_max(0, (state->framerate-30)/10),
				.stills_capture_circular_buffer_height = 0,
				.fast_preview_resume = 0,
				.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RAW_STC
			};
		mmal_port_parameter_set(camera->control, &cam_config.hdr);
	}

	// Now set up the port formats

	// Set the encode format on the Preview port
	// HW limitations mean we need the preview to be the same size as the required recorded output

	format = preview_port->format;

	format->encoding = MMAL_ENCODING_OPAQUE;
	format->encoding_variant = MMAL_ENCODING_I420;

	if(state->camera_parameters.shutter_speed > 6000000)
	{
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
							{ 50, 1000 }, {166, 1000}};
		mmal_port_parameter_set(preview_port, &fps_range.hdr);
	}
	else if(state->camera_parameters.shutter_speed > 1000000)
	{
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
							{ 166, 1000 }, {999, 1000}};
		mmal_port_parameter_set(preview_port, &fps_range.hdr);
	}

	//enable dynamic framerate if necessary
	if (state->camera_parameters.shutter_speed)
	{
		if (state->framerate > 1000000./state->camera_parameters.shutter_speed)
		{
			state->framerate=0;
			if (state->verbose)
				cerr << "Enable dynamic frame rate to fulfil shutter speed requirement\n";
		}
	}

	format->encoding = MMAL_ENCODING_OPAQUE;
	format->es->video.width = VCOS_ALIGN_UP(state->width, 32);
	format->es->video.height = VCOS_ALIGN_UP(state->height, 16);
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = state->width;
	format->es->video.crop.height = state->height;
	format->es->video.frame_rate.num = PREVIEW_FRAME_RATE_NUM;
	format->es->video.frame_rate.den = PREVIEW_FRAME_RATE_DEN;

	status = mmal_port_format_commit(preview_port);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("camera viewfinder format couldn't be set");
		goto error;
	}

	// Set the encode format on the video  port

	format = video_port->format;
	format->encoding_variant = MMAL_ENCODING_I420;

	if(state->camera_parameters.shutter_speed > 6000000)
	{
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
							{ 50, 1000 }, {166, 1000}};
		mmal_port_parameter_set(video_port, &fps_range.hdr);
	}
	else if(state->camera_parameters.shutter_speed > 1000000)
	{
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
							{ 167, 1000 }, {999, 1000}};
		mmal_port_parameter_set(video_port, &fps_range.hdr);
	}

	format->encoding = MMAL_ENCODING_OPAQUE;
	format->es->video.width = VCOS_ALIGN_UP(state->width, 32);
	format->es->video.height = VCOS_ALIGN_UP(state->height, 16);
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = state->width;
	format->es->video.crop.height = state->height;
	format->es->video.frame_rate.num = state->framerate;
	format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

	status = mmal_port_format_commit(video_port);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("camera video format couldn't be set");
		goto error;
	}

	// Ensure there are enough buffers to avoid dropping frames
	if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
		video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;


	// Set the encode format on the still  port

	format = still_port->format;

	format->encoding = MMAL_ENCODING_OPAQUE;
	format->encoding_variant = MMAL_ENCODING_I420;

	format->es->video.width = VCOS_ALIGN_UP(state->width, 32);
	format->es->video.height = VCOS_ALIGN_UP(state->height, 16);
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = state->width;
	format->es->video.crop.height = state->height;
	format->es->video.frame_rate.num = 0;
	format->es->video.frame_rate.den = 1;

	status = mmal_port_format_commit(still_port);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("camera still format couldn't be set");
		goto error;
	}

	/* Ensure there are enough buffers to avoid dropping frames */
	if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
		still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

	/* Enable component */
	status = mmal_component_enable(camera);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("camera component couldn't be enabled");
		goto error;
	}

	raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

	state->camera_component = camera;

	if (state->verbose)
		cerr << "Camera component done\n";

	return status;

error:

	if (camera)
		mmal_component_destroy(camera);

	return status;
}

int main(int argc, char * argv[]){

	RASPIVID_STATE state;

	MMAL_STATUS_T status = MMAL_SUCCESS;
	MMAL_PORT_T *camera_preview_port = NULL;
	MMAL_PORT_T *camera_video_port = NULL;
	MMAL_PORT_T *camera_still_port = NULL;
	MMAL_PORT_T *preview_input_port = NULL;
	MMAL_PORT_T *encoder_input_port = NULL;
	MMAL_PORT_T *encoder_output_port = NULL;
	MMAL_PORT_T *splitter_input_port = NULL;
	MMAL_PORT_T *splitter_output_port = NULL;
	MMAL_PORT_T *splitter_preview_port = NULL;

	bcm_host_init();

	default_status(&state);

	if((status = create_camera_component(&state)) != MMAL_SUCCESS){
		cout << "Error 1\n";
		return -1;
	}

	return 0;
}
