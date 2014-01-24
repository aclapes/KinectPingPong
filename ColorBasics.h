//------------------------------------------------------------------------------
// <copyright file="ColorBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include <Windows.h>
#include <ShellAPI.h>

#include "resource.h"
#include "NuiApi.h"
#include "ImageRenderer.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/system/error_code.hpp>
#include <boost/timer.hpp>

#include <opencv2/opencv.hpp>

//#define DEBUG_INFRARED
//#define DEBUG_TIMES
//#define DEBUG_USERDETECTION

//#define USE_SKELETON

//#define SYNC_WEARABLE

class CColorBasics
{
	static const int					cBytesPerPixel   = 4;

	static const NUI_IMAGE_RESOLUTION	cDepthResolution = NUI_IMAGE_RESOLUTION_640x480;
	static const NUI_IMAGE_RESOLUTION	cColorResolution = NUI_IMAGE_RESOLUTION_640x480;

    static const int        cStatusMessageMaxLen = MAX_PATH*2;

	static const int					cInfraredDelayTime = 250; // ms

public:
    /// <summary>
    /// Constructor
    /// </summary>
    CColorBasics();

    /// <summary>
    /// Destructor
    /// </summary>
    ~CColorBasics();

    /// <summary>
    /// Handles window messages, passes most to the class instance to handle
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Handle windows messages for a class instance
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    LRESULT CALLBACK        DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Creates the main window and begins processing
    /// </summary>
    /// <param name="hInstance"></param>
    /// <param name="nCmdShow"></param>
    int                     Run(HINSTANCE hInstance, int nCmdShow);

	/// <summary>
    /// Creates the main window and begins processing
    /// </summary>
    /// <param name="hInstance"></param>
    /// <param name="nCmdShow"></param>
	/// <param name="host"></param>
    /// <param name="port"></param>
    int                     RunClient(HINSTANCE hInstance, int nCmdShow, const char* host, const char* port);

    /// <summary>
    /// Creates the main window and begins processing
    /// </summary>
    /// <param name="hInstance"></param>
    /// <param name="nCmdShow"></param>
	/// <param name="port"></param>
    int                     RunServer(HINSTANCE hInstance, int nCmdShow, const char* port, const char* wearhost, const char* wearport);

private:
    HWND                    m_hWnd;

    bool                    m_bSaveScreenshot;
	bool                    m_bSeatedMode;

    // Current Kinect
    INuiSensor*             m_pNuiSensor;

	ID2D1HwndRenderTarget*   m_pRenderTarget;
#ifdef USE_SKELETON
    // Skeletal and skeletal drawing
	float					 m_SkelsData[NUI_SKELETON_COUNT][NUI_SKELETON_POSITION_COUNT+1];
    ID2D1SolidColorBrush*    m_pBrushJointTracked;
    ID2D1SolidColorBrush*    m_pBrushJointInferred;
    ID2D1SolidColorBrush*    m_pBrushBoneTracked;
    ID2D1SolidColorBrush*    m_pBrushBoneInferred;
    D2D1_POINT_2F            m_Points[NUI_SKELETON_POSITION_COUNT];
#endif

    // Direct2D
    ImageRenderer*          m_pDrawColor;
    ID2D1Factory*           m_pD2DFactory;
    
    HANDLE                  m_pColorStreamHandle;
	HANDLE                  m_pDepthStreamHandle;

    HANDLE                  m_hNextDepthFrameEvent;
    HANDLE                  m_hNextColorFrameEvent;

#ifdef USE_SKELETON
	HANDLE                  m_pSkeletonStreamHandle;
	HANDLE                  m_hNextSkeletonEvent;
#endif

	USHORT*                 m_depthD16;
	BYTE*					m_depthRGBX;
	BYTE*					m_colorRGB;
	BYTE*					m_colorRGBX;
	USHORT*					m_alignedDepth;
	BYTE*					m_alignedPlayerIdx;

    LONG                    m_depthWidth;
    LONG                    m_depthHeight;

    LONG                    m_colorWidth;
    LONG                    m_colorHeight;

	LONG					m_colorToDepthDivisor;
	
	LONG					m_numRecordedFrames;
	std::string				m_colorPath;
	std::string				m_depthPath;
	std::string				m_extensionType;

#ifdef USE_SKELETON
	NUI_SKELETON_FRAME		m_skeletonFrame;
	std::string				m_skelsFileName;
	cv::FileStorage			m_SkelsDataFS;
#endif

	bool					m_clientRole;
	bool					m_bPingPong;
	boost::asio::ip::tcp::socket* m_pSocket;
	boost::mutex			m_mtx;

	std::vector<double> m_times;
	boost::timer m_general;

#ifdef DEBUG_TIMES
	std::vector<double>		m_FrameTimes;
	cv::FileStorage			m_TimesFS;
#endif

	
	void peredelafavera(void);

    /// <summary>
    /// Update depth frame
    /// </summary>
    int                    UpdateDepth();

    /// <summary>
    /// Update color frame, skeleton, perfom color-depth alignment, and frame storing on disk
    /// </summary>
    int                    UpdateAllExceptDepth();

    /// <summary>
    /// Update all frames and variables without exception
    /// </summary>
    int                    UpdateAll();

    /// <summary>
    /// Create the first connected Kinect found 
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 CreateFirstConnected();

    /// <summary>
    /// Handle new depth data
    /// </summary>
    void                    ProcessDepth();

    /// <summary>
    /// Handle new color data
    /// </summary>
    void                    ProcessColor();

#ifdef USE_SKELETON
    /// <summary>
    /// Handle new skeleton data
    /// </summary>
    void                    ProcessSkeleton();
#endif

    /// <summary>
    /// Ensure necessary Direct2d resources are created
    /// </summary>
    /// <returns>S_OK if successful, otherwise an error code</returns>
    HRESULT                 EnsureDirect2DResources( );
	
    /// <summary>
    /// Dispose Direct2d resources 
    /// </summary>
    void                    DiscardDirect2DResources( );

#ifdef USE_SKELETON
    /// <summary>
    /// Draws a bone line between two joints
    /// </summary>
    /// <param name="skel">skeleton to draw bones from</param>
    /// <param name="joint0">joint to start drawing from</param>
    /// <param name="joint1">joint to end drawing at</param>
    void                    DrawBone(const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX bone0, NUI_SKELETON_POSITION_INDEX bone1);

	/// <summary>
    /// Draws a skeleton
    /// </summary>
    /// <param name="skel">skeleton to draw</param>
    /// <param name="windowWidth">width (in pixels) of output buffer</param>
    /// <param name="windowHeight">height (in pixels) of output buffer</param>
	void					DrawSkeletonFrame(const NUI_SKELETON_FRAME & skeletonFrame);

    /// <summary>
    /// Draws a skeleton
    /// </summary>
    /// <param name="skel">skeleton to draw</param>
    /// <param name="windowWidth">width (in pixels) of output buffer</param>
    /// <param name="windowHeight">height (in pixels) of output buffer</param>
    void                    DrawSkeleton(const NUI_SKELETON_DATA & skel, int windowWidth, int windowHeight);

    /// <summary>
    /// Converts a skeleton point to screen space
    /// </summary>
    /// <param name="skeletonPoint">skeleton point to tranform</param>
    /// <param name="width">width (in pixels) of output buffer</param>
    /// <param name="height">height (in pixels) of output buffer</param>
    /// <returns>point in screen-space</returns>
    D2D1_POINT_2F           SkeletonToScreen(Vector4 skeletonPoint, int width, int height);
#endif

	/// <summary>
    /// Register color to depth (malfunctioning?). Better use MapDepthToColor instead.
    /// </summary>
	void					MapColorToDepth();

	/// <summary>
    /// Register depth to color
    /// </summary>
	void					MapDepthToColor(const USHORT* depth, USHORT* alignedDepth);

	/// <summary>
    /// Register depth to color, and returns separately the depth and the player idx information
    /// </summary>
	void					MapDepthToColor(const USHORT* depth, USHORT* alignedDepth, BYTE* alignedPlayerIdx, bool rmPlayerBits = true);

	/// <summary>
	/// Store the generated depth and color data to disk in images
	/// </summary>
	void					saveDataToDisk();

	/// <summary>
	/// Create a client
	/// </summary>
	void					CreateClient(const char* host, const char* port);

	/// <summary>
	/// Create a server
	/// </summary>
	void					CreateServer(const char* port);

	/// <summary>
	/// Synchronize with wearable sensors via other computer
	/// </summary>
	int					SyncWearable(const char* host, const char* port);

	/// <summary>
	/// Put the created server to listen
	/// </summary>
	void					Server(boost::asio::io_service& io_service, unsigned short port);

	/// <summary>
	/// Listen a socket in a thread
	/// </summary>
	void					Session(boost::shared_ptr<boost::asio::ip::tcp::socket> sock);

    /// <summary>
    /// Set the status bar message
    /// </summary>
    /// <param name="szMessage">message to display</param>
    void                    SetStatusMessage(WCHAR* szMessage);

    /// <summary>
    /// Save passed in image data to disk as a bitmap
    /// </summary>
    /// <param name="pBitmapBits">image data to save</param>
    /// <param name="lWidth">width (in pixels) of input image data</param>
    /// <param name="lHeight">height (in pixels) of input image data</param>
    /// <param name="wBitsPerPixel">bits per pixel of image data</param>
    /// <param name="lpszFilePath">full file path to output bitmap to</param>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCTSTR lpszFilePath);

};
