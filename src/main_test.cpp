/* Copyright (C) 2012-2017 Ultraleap Limited. All rights reserved.
 *
 * Use of this code is subject to the terms of the Ultraleap SDK agreement
 * available at https://central.leapmotion.com/agreements/SdkAgreement unless
 * Ultraleap has signed a separate license agreement with you or your
 * organisation.
 *
 */

#include "LeapC.h"
#include "leapmotion.hpp"
#include <stdio.h>
#include <stdlib.h>

// Assuming definitions as given:
#define TEX_WIDTH 384
#define TEX_HEIGHT 384
#define MAX_FOV 8

//opencv
#include <opencv2/opencv.hpp>

static LEAP_CONNECTION *connectionHandle;

/** Callback for when the connection opens. */
static void
OnConnect(void)
{
    printf("Connected.\n");
}

/** Callback for when a device is found. */
static void
OnDevice(const LEAP_DEVICE_INFO *props)
{
    printf("Found device %s.\n", props->serial);
}

/** Callback for when a frame of tracking data is available. */
static void
OnFrame(const LEAP_TRACKING_EVENT *frame)
{
    if(frame->info.frame_id % 60 == 0)
        printf("Frame %lli with %i hands.\n",
               (long long int)frame->info.frame_id, frame->nHands);

    for(uint32_t h = 0; h < frame->nHands; h++)
    {
        LEAP_HAND *hand = &frame->pHands[h];
        printf("    Hand id %i is a %s hand with position (%f, %f, %f).\n",
               hand->id, (hand->type == eLeapHandType_Left ? "left" : "right"),
               hand->palm.position.x, hand->palm.position.y,
               hand->palm.position.z);
    }
}

static void
OnImage(const LEAP_IMAGE_EVENT *image)
{
    printf("Image %lli => Left: %d x %d (bpp=%d), Right: %d x %d (bpp=%d)\n",
           (long long int)image->info.frame_id,
           image->image[0].properties.width, image->image[0].properties.height,
           image->image[0].properties.bpp * 8, image->image[1].properties.width,
           image->image[1].properties.height,
           image->image[1].properties.bpp * 8);

    // Get the image data
    cv::Mat img(image->image[0].properties.height, image->image[0].properties.width, CV_8UC1, (void*)image->image[0].data);
    const uint8_t *image_buffer = (const uint8_t *)image->image[0].data;

    //create undistorted_image
    cv::Mat undistorted(TEX_HEIGHT, TEX_WIDTH, CV_8UC1);
    uint8_t *undistorted_image = undistorted.data;

    for(float row = 0; row < TEX_HEIGHT; row++)
    {
        for(float col = 0; col < TEX_WIDTH; col++)
        {
            // Normalize from pixel xy to range [0..1]
            LEAP_VECTOR input;
            input.x = col / TEX_WIDTH;
            input.y = row / TEX_HEIGHT;

            // Convert from normalized [0..1] to ray slopes
            input.x = (input.x - 0.5) * MAX_FOV;
            input.y = (input.y - 0.5) * MAX_FOV;

            // Convert rectilinear coordinates to pixel coordinates
            LEAP_VECTOR pixel = LeapRectilinearToPixel(
                *connectionHandle, eLeapPerspectiveType_stereo_left, input);
            int dindex = (int)floor(row * TEX_WIDTH +
                                    col); // Index in the undistorted image
            int pindex = (int)roundf(pixel.y) * 384 +
                         (int)roundf(pixel.x); // Index in the original image

            // Check if the pixel coordinates are within the image boundaries
            if(pixel.x >= 0 && pixel.x < 384 && pixel.y >= 0 &&
               pixel.y < 384)
            {
                undistorted_image[dindex] = image_buffer[pindex];
            }
            else
            {
                undistorted_image[dindex] =
                    128; // Use a default value (e.g., gray) for out-of-bounds
            }
        }
    }
    // Display or further process undistorted images
    cv::imshow("Left Undistorted", undistorted);
    cv::waitKey(1); // Refresh the display window
}

static void
OnLogMessage(const eLeapLogSeverity severity,
             const int64_t timestamp,
             const char *message)
{
    const char *severity_str;
    switch(severity)
    {
    case eLeapLogSeverity_Critical:
        severity_str = "Critical";
        break;
    case eLeapLogSeverity_Warning:
        severity_str = "Warning";
        break;
    case eLeapLogSeverity_Information:
        severity_str = "Info";
        break;
    default:
        severity_str = "";
        break;
    }
    printf("[%s][%lli] %s\n", severity_str, (long long int)timestamp, message);
}

static void *
allocate(uint32_t size, eLeapAllocatorType typeHint, void *state)
{
    void *ptr = malloc(size);
    return ptr;
}

static void
deallocate(void *ptr, void *state)
{
    if(!ptr)
        return;
    free(ptr);
}

void
OnPointMappingChange(const LEAP_POINT_MAPPING_CHANGE_EVENT *change)
{
    if(!connectionHandle)
        return;

    uint64_t size = 0;
    if(LeapGetPointMappingSize(*connectionHandle, &size) != eLeapRS_Success ||
       !size)
        return;

    LEAP_POINT_MAPPING *pointMapping =
        (LEAP_POINT_MAPPING *)malloc((size_t)size);
    if(!pointMapping)
        return;

    if(LeapGetPointMapping(*connectionHandle, pointMapping, &size) ==
           eLeapRS_Success &&
       pointMapping->nPoints > 0)
    {
        printf("Managing %u points as of frame %lld at %lld\n",
               pointMapping->nPoints, (long long int)pointMapping->frame_id,
               (long long int)pointMapping->timestamp);
    }
    free(pointMapping);
}

void
OnHeadPose(const LEAP_HEAD_POSE_EVENT *event)
{
    printf("Head pose:\n");
    printf("    Head position (%f, %f, %f).\n", event->head_position.x,
           event->head_position.y, event->head_position.z);
    printf("    Head orientation (%f, %f, %f, %f).\n",
           event->head_orientation.w, event->head_orientation.x,
           event->head_orientation.y, event->head_orientation.z);
    printf("    Head linear velocity (%f, %f, %f).\n",
           event->head_linear_velocity.x, event->head_linear_velocity.y,
           event->head_linear_velocity.z);
    printf("    Head angular velocity (%f, %f, %f).\n",
           event->head_angular_velocity.x, event->head_angular_velocity.y,
           event->head_angular_velocity.z);
}

int
main(int argc, char **argv)
{

    // Create a window
    cv::namedWindow("Left", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Right", cv::WINDOW_AUTOSIZE);

    cv::Mat left, right;
    //red image
    left = cv::Mat(480, 640, CV_8UC1, cv::Scalar(0));
    //green image
    right = cv::Mat(480, 640, CV_8UC1, cv::Scalar(0));

    cv::imshow("Left", left);
    cv::imshow("Right", right);
    cv::waitKey(1);

    //Set callback function pointers
    ConnectionCallbacks.on_connection = &OnConnect;
    ConnectionCallbacks.on_device_found = &OnDevice;
    ConnectionCallbacks.on_frame = &OnFrame;
    ConnectionCallbacks.on_image = &OnImage;
    ConnectionCallbacks.on_point_mapping_change = &OnPointMappingChange;
    ConnectionCallbacks.on_log_message = &OnLogMessage;
    ConnectionCallbacks.on_head_pose = &OnHeadPose;

    connectionHandle = OpenConnection();
    {
        LEAP_ALLOCATOR allocator = {allocate, deallocate, NULL};
        LeapSetAllocator(*connectionHandle, &allocator);
    }
    LeapSetPolicyFlags(*connectionHandle,
                       eLeapPolicyFlag_Images | eLeapPolicyFlag_MapPoints, 0);

    printf("Press Enter to exit program.\n");
    getchar();

    CloseConnection();
    DestroyConnection();

    return 0;
}
//End-of-Sample
