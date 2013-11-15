//------------------------------------------------------------------------------
// <copyright file="ColorBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <strsafe.h>
#include "ColorBasics.h"
#include "resource.h"

#include "boost/date_time/local_time/local_time.hpp"

static const float g_JointThickness = 3.0f;
static const float g_TrackedBoneThickness = 6.0f;
static const float g_InferredBoneThickness = 1.0f;

using boost::asio::ip::tcp;

const int max_length = 64;

typedef boost::shared_ptr<tcp::socket> socket_ptr;

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR lpCmdLine, int nCmdShow)
{
	// Get command line arguments
	std::vector< std::string > args;

	LPWSTR *szArglist;
    int nArgs;

    szArglist = CommandLineToArgvW(GetCommandLineW(), &nArgs);
    if( NULL == szArglist )
    {
       wprintf(L"CommandLineToArgvW failed\n");
       return 0;
    }
    else 
	{
		for (int i = 0; i < nArgs; i++)
		{
			std::wstring wstr (szArglist[i]);
			std::string str (wstr.begin(), wstr.end()); // insane fix
			args.push_back(str);
		}
	}

    LocalFree(szArglist); // Free memory allocated for CommandLineToArgvW arguments.

	// Create the application
	CColorBasics application;

	if (args.size() < 2) // standalone version
	{
		application.Run(hInstance, nCmdShow);
	}
	else // client-server pingpong
	{
		if (args.size() > 2) // client
		{
			application.RunClient(hInstance, nCmdShow, args[1].c_str(), args[2].c_str());

		}
		else // server
		{
			application.RunServer(hInstance, nCmdShow, args[1].c_str());
		}
	}
}

/// <summary>
/// Constructor
/// </summary>
CColorBasics::CColorBasics() :
    m_pD2DFactory(NULL),
    m_pDrawColor(NULL),
	m_pRenderTarget(NULL),
    m_hNextColorFrameEvent(INVALID_HANDLE_VALUE),
	m_hNextDepthFrameEvent(INVALID_HANDLE_VALUE),
#ifdef USE_SKELETON
	m_hNextSkeletonEvent(INVALID_HANDLE_VALUE),
#endif
    m_pColorStreamHandle(INVALID_HANDLE_VALUE),
    m_pDepthStreamHandle(INVALID_HANDLE_VALUE),
#ifdef USE_SKELETON
    m_pSkeletonStreamHandle(INVALID_HANDLE_VALUE),
#endif
    m_bSaveScreenshot(false),
    m_pNuiSensor(NULL)
{
	// get resolution as DWORDS, but store as LONGs to avoid casts later
    DWORD width = 0;
    DWORD height = 0;

    NuiImageResolutionToSize(cDepthResolution, width, height);
    m_depthWidth  = static_cast<LONG>(width);
    m_depthHeight = static_cast<LONG>(height);

    NuiImageResolutionToSize(cColorResolution, width, height);
    m_colorWidth  = static_cast<LONG>(width);
    m_colorHeight = static_cast<LONG>(height);

	m_colorToDepthDivisor = m_colorWidth/m_depthWidth;

	// create heap storage for depth pixel data in RGBX format
	m_depthD16			= new USHORT[m_depthWidth*m_depthHeight];
    m_depthRGBX			= new BYTE[m_depthWidth*m_depthHeight*cBytesPerPixel];
	m_colorRGBX			= new BYTE[m_colorWidth*m_colorHeight*cBytesPerPixel];
	m_colorRGB			= new BYTE[m_colorWidth*m_colorHeight*3];

	m_alignedDepth		= new USHORT[m_depthWidth*m_depthHeight];
	m_alignedPlayerIdx	= new BYTE[m_depthWidth*m_depthHeight];

	// Choose image saving extension type (e.g. PNG)
	m_extensionType = ".png";

	m_colorPath = "Data/Color/";
	m_depthPath = "Data/Depth/";

#ifdef USE_SKELETON
	// skeletal drawing
	ZeroMemory(m_Points,sizeof(m_Points));

	m_skelsFileName = "Data/skels.xml";
	m_SkelsDataFS.open(m_skelsFileName, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_XML );
#endif

#ifdef DEBUG_TIMES
	m_TimesFS.open("Data/times.yml", cv::FileStorage::WRITE );
#endif

	m_numRecordedFrames = 0;

	peredelafavera();
	m_general.restart(); // initialize

#ifdef DEBUG_USERDETECTION
	cv::namedWindow("debug");
#endif
}

/// <summary>
/// Destructor
/// </summary>
CColorBasics::~CColorBasics()
{
	cv::FileStorage fs ("data/frametimes.yml", cv::FileStorage::WRITE);
	fs << "timesvector" << m_times;
	fs.release();

#ifdef USE_SKELETON
	m_SkelsDataFS.release();
#endif // USE_SKELETON
#ifdef DEBUG_TIMES
	m_TimesFS.release();
#endif // DEBUG_TIMES

    if (m_pNuiSensor)
    {
        m_pNuiSensor->NuiShutdown();
    }

    if (m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_hNextDepthFrameEvent);
    }

    if (m_hNextColorFrameEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_hNextColorFrameEvent);
    }

#ifdef USE_SKELETON
	if (m_hNextSkeletonEvent && (m_hNextSkeletonEvent != INVALID_HANDLE_VALUE))
    {
        CloseHandle(m_hNextSkeletonEvent);
    }
#endif

    // clean up Direct2D renderer
    delete m_pDrawColor;
    m_pDrawColor = NULL;

	// done with depth pixel data
    delete[] m_depthD16;
	delete[] m_depthRGBX;
	delete[] m_colorRGBX;
	delete[] m_colorRGB;
	delete[] m_alignedDepth;
	delete[] m_alignedPlayerIdx;

	// clean up Direct2D objects
    DiscardDirect2DResources();

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

    SafeRelease(m_pNuiSensor);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CColorBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hInstance     = hInstance;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"ColorBasicsAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        hInstance,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CColorBasics::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);

    const int eventCount = 1;
    HANDLE hEvents[eventCount];

    // Main message loop
	int pairedState = 0;
    while (WM_QUIT != msg.message && pairedState == 0)
    {
		

#ifdef DEBUG_INFRARED
		boost::timer t;
		m_pNuiSensor->NuiSetForceInfraredEmitterOff(false);
		double emitterOffTime = t.elapsed() * 1000.0; // in millisecs
		double sleepTime = std::max<double>(0, cInfraredDelayTime - emitterOffTime);
		Sleep(sleepTime);
#ifdef DEBUG_TIMES
		m_FrameTimes.clear();
		m_FrameTimes.push_back(emitterOffTime); // 1
#endif // DEBUG_TIMES
#endif // DEBUG_INFRARED

		//hEvents[2] = m_hNextSkeletonEvent;
  //      hEvents[1] = m_hNextColorFrameEvent;
		hEvents[0] = m_hNextDepthFrameEvent;

#ifdef DEBUG_TIMES
		t.restart();
#endif
        // Check to see if we have either a message (by passing in QS_ALLINPUT)
        // Or a Kinect event (hEvents)
        // Update() will check for Kinect events individually, in case more than one are signalled
        MsgWaitForMultipleObjects(eventCount, hEvents, FALSE, INFINITE, QS_ALLINPUT);
#ifdef DEBUG_TIMES
		m_FrameTimes.push_back(t.elapsed() * 1000.0); // 2
#endif

        // Explicitly check the Kinect frame event since MsgWaitForMultipleObjects
        // can return for other reasons even though it is signaled.
		UpdateAll(); // timers: 3 - 7

#ifdef DEBUG_INFRARED
		m_pNuiSensor->NuiSetForceInfraredEmitterOff(true);
#endif

#ifdef DEBUG_TIMES
		t.restart();
#endif
        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if ((hWndApp != NULL) && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }
			
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
#ifdef DEBUG_TIMES
		m_FrameTimes.push_back(t.elapsed() * 1000.0); // 8
		m_TimesFS << "num_frames" << m_numRecordedFrames;
		std::stringstream ss;
		ss << "time_" << m_numRecordedFrames;
		m_TimesFS << ss.str() << m_FrameTimes;
#endif

		m_numRecordedFrames++;
    }

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <param name="host">host server to which establish connection</param>
/// <param name="port">port to communicate with</param>
int CColorBasics::RunClient(HINSTANCE hInstance, int nCmdShow, const char* host, const char* port)
{
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hInstance     = hInstance;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"ColorBasicsAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        hInstance,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CColorBasics::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);

    const int eventCount = 1;
    HANDLE hEvents[eventCount];

	CreateClient(host, port);

		boost::timer t;
		char request[max_length];
		request[0] = 'A';
		size_t request_length = strlen(request);
		boost::asio::write(*m_pSocket, boost::asio::buffer(request, request_length));

		char reply[max_length];
		size_t reply_length = boost::asio::read(*m_pSocket, boost::asio::buffer(reply, request_length));
		double elapsedt = t.elapsed();

    // Main message loop
	int pairedState = 0;
    while (WM_QUIT != msg.message && pairedState == 0)
    {
		m_mtx.lock();

		boost::timer timer;
		m_pNuiSensor->NuiSetForceInfraredEmitterOff(false);
		// Take profit of this time
		//--------------------------------------------------------------------
		while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if ((hWndApp != NULL) && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }
			
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
		//--------------------------------------------------------------------
		// Time to take profit is over

		double emitterOffTime = timer.elapsed() * 1000.0; // in millisecs

		double sleepTime = std::max<double>(0, cInfraredDelayTime - emitterOffTime);
		Sleep(sleepTime);

		//hEvents[2] = m_hNextSkeletonEvent;
  //      hEvents[1] = m_hNextColorFrameEvent;
		hEvents[0] = m_hNextDepthFrameEvent;

        // Check to see if we have either a message (by passing in QS_ALLINPUT)
        // Or a Kinect event (hEvents)
        // Update() will check for Kinect events individually, in case more than one are signalled
        MsgWaitForMultipleObjects(eventCount, hEvents, FALSE, INFINITE, QS_ALLINPUT);

        // Explicitly check the Kinect frame event since MsgWaitForMultipleObjects
        // can return for other reasons even though it is signaled.
		UpdateDepth();

		m_mtx.unlock();

		m_pNuiSensor->NuiSetForceInfraredEmitterOff(true);

		boost::thread t (&CColorBasics::UpdateAllExceptDepth, this);

		char request[max_length];
		request[0] = 'A';
		size_t request_length = strlen(request);
		boost::asio::write(*m_pSocket, boost::asio::buffer(request, request_length));

		char reply[max_length];
		size_t reply_length = boost::asio::read(*m_pSocket, boost::asio::buffer(reply, request_length));
    
		m_numRecordedFrames++;
	}

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <param name="port">port to communicate with</param>
int CColorBasics::RunServer(HINSTANCE hInstance, int nCmdShow, const char* port)
{
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hInstance     = hInstance;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"ColorBasicsAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        hInstance,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CColorBasics::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);

    const int eventCount = 1;
    HANDLE hEvents[eventCount];

	CreateServer(port);

    // Main message loop
	int pairedState = 0;
    while (WM_QUIT != msg.message && pairedState == 0)
    {
		char data[max_length];
		boost::system::error_code error;

		size_t length = m_pSocket->read_some(boost::asio::buffer(data), error);
		if (error == boost::asio::error::eof)
			return -1; // Connection closed cleanly by peer.
		else if (error)
			throw boost::system::system_error(error); // Some other error.

		m_mtx.lock();

		boost::timer timer;
		m_pNuiSensor->NuiSetForceInfraredEmitterOff(false);
		// Take profit of this time
		//--------------------------------------------------------------------
		while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if ((hWndApp != NULL) && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }
			
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
		//--------------------------------------------------------------------
		// Time to take profit is over
		double emitterOffTime = timer.elapsed() * 1000.0; // in millisecs

		double sleepTime = std::max<double>(0, cInfraredDelayTime - emitterOffTime);
		Sleep(sleepTime);

		//hEvents[2] = m_hNextSkeletonEvent;
  //      hEvents[1] = m_hNextColorFrameEvent;
		hEvents[0] = m_hNextDepthFrameEvent;

        // Check to see if we have either a message (by passing in QS_ALLINPUT)
        // Or a Kinect event (hEvents)
        // Update() will check for Kinect events individually, in case more than one are signalled
        MsgWaitForMultipleObjects(eventCount, hEvents, FALSE, INFINITE, QS_ALLINPUT);

        // Explicitly check the Kinect frame event since MsgWaitForMultipleObjects
        // can return for other reasons even though it is signaled.
		
		UpdateDepth();

		m_mtx.unlock();

		m_pNuiSensor->NuiSetForceInfraredEmitterOff(true);

		boost::thread t (&CColorBasics::UpdateAllExceptDepth, this);
		boost::asio::write(*m_pSocket, boost::asio::buffer(data, length));

		m_numRecordedFrames++;
    }

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Update depth frame
/// </summary>
int CColorBasics::UpdateDepth()
{
    if (NULL == m_pNuiSensor)
    {
        return -1;
    }

	if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0) )
    {
		ProcessDepth();
	}

	m_times.push_back(m_general.elapsed() * 1000);

	return 0;
}

/// <summary>
/// Update color frame, skeleton, perfom color-depth alignment, and frame storing on disk
/// </summary>
int CColorBasics::UpdateAllExceptDepth()
{
	m_mtx.lock();

    if (NULL == m_pNuiSensor)
    {
        return -1;
    }

	MapDepthToColor(m_depthD16, m_alignedDepth, m_alignedPlayerIdx, false); // putting before processColor()

    if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextColorFrameEvent, 0) )
    {
        ProcessColor(); // mapdepthtocolor before
    }

#ifdef USE_SKELETON
	// Wait for 0ms, just quickly test if it is time to process a skeleton
    if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextSkeletonEvent, 0) )
    {
        ProcessSkeleton();
    }
#endif

	saveDataToDisk();

	m_mtx.unlock();

	return 0;
}

/// <summary>
/// Update all frames and variables without exception
/// </summary>
int CColorBasics::UpdateAll()
{
	m_mtx.lock();
	boost::timer t;

    if (NULL == m_pNuiSensor)
    {
        return -1;
    }

#ifdef DEBUG_TIMES
		t.restart();
#endif
	if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0) )
    {
		ProcessDepth();
	}
	m_times.push_back(m_general.elapsed() * 1000.0);
#ifdef DEBUG_TIMES
		m_FrameTimes.push_back(t.elapsed() * 1000.0); // 3
#endif

	MapDepthToColor(m_depthD16, m_alignedDepth, m_alignedPlayerIdx, false); //MapColorToDepth(); // align spatially color and depth

#ifdef DEBUG_TIMES
		t.restart();
#endif
    if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextColorFrameEvent, 0) )
    {
        ProcessColor();
    }
#ifdef DEBUG_TIMES
		m_FrameTimes.push_back(t.elapsed() * 1000.0); // 4
#endif

#ifdef USE_SKELETON
#ifdef DEBUG_TIMES
		t.restart();
#endif

	// Wait for 0ms, just quickly test if it is time to process a skeleton
    if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextSkeletonEvent, 0) )
    {
        ProcessSkeleton();
    }
#ifdef DEBUG_TIMES
	m_FrameTimes.push_back(t.elapsed() * 1000.0); // 5
#endif
#endif // USE_SKELETON

#ifdef DEBUG_TIMES
	t.restart();
#endif

#ifdef DEBUG_TIMES
	m_FrameTimes.push_back(t.elapsed() * 1000.0); // 6
#endif
#ifdef DEBUG_USERDETECTION
	cv::Mat piMat (m_depthHeight, m_depthWidth, CV_8UC1, m_alignedPlayerIdx);
	cv::imshow("debug", piMat * 255.0);
	cv::waitKey(200);
#endif

#ifdef DEBUG_TIMES
	t.restart();
#endif
	saveDataToDisk(); // save data to disk
#ifdef DEBUG_TIMES
	m_FrameTimes.push_back(t.elapsed() * 1000.0); // 7
#endif

	m_mtx.unlock();

	return 0;
}

void CColorBasics::saveDataToDisk()
{
	// Create two cv structures pointing to already existing color data and depth data arrays
	cv::Mat colorMat (m_colorHeight, m_colorWidth, CV_8UC3, m_colorRGB);
	cv::Mat depthMat (m_depthHeight, m_depthWidth, CV_16UC1, m_alignedDepth);

	// Save it to disk
	std::stringstream ss;
	ss << m_numRecordedFrames;
	std::string colorFileName = m_colorPath + ss.str() + m_extensionType;
	std::string depthFilename = m_depthPath + ss.str() + m_extensionType;

	cv::imwrite(colorFileName.c_str(), colorMat);
	cv::imwrite(depthFilename.c_str(), depthMat);

#ifdef USE_SKELETON
	cv::Mat skelsMat (NUI_SKELETON_COUNT, NUI_SKELETON_POSITION_COUNT * 4 + 1, CV_32FC1, m_SkelsData);

	std::stringstream ss2;
	ss2 << "skel_" << m_numRecordedFrames;
	std::string skelName = ss2.str();
	m_SkelsDataFS << skelName.c_str() << skelsMat;
#endif
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CColorBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CColorBasics* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CColorBasics*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CColorBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CColorBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init Direct2D
            D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

            // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
            // We'll use this to draw the data we receive from the Kinect to the screen
            m_pDrawColor = new ImageRenderer();
            HRESULT hr = m_pDrawColor->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), m_pD2DFactory, m_colorWidth, m_colorHeight, m_colorWidth * sizeof(long));
            if (FAILED(hr))
            {
                SetStatusMessage(L"Failed to initialize the Direct2D draw device.");
            }

            // Look for a connected Kinect, and create it if found
            CreateFirstConnected();
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;

        // Handle button press
        case WM_COMMAND:
            // If it was for the screenshot control and a button clicked event, save a screenshot next frame 
            if (IDC_BUTTON_SCREENSHOT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
            {
                m_bSaveScreenshot = true;
            }
          
			// If it was for the near mode control and a clicked event, change near mode
			if (IDC_CHECK_SEATED == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
			{
				// Toggle out internal state for near mode
				m_bSeatedMode = !m_bSeatedMode;

#ifdef USE_SKELETON
				if (NULL != m_pNuiSensor)
				{
				// Set near mode for sensor based on our internal state
					m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonEvent, m_bSeatedMode ? NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT : 0);
				}
#endif
			}
			break;
    }

    return FALSE;
}

/// <summary>
/// Create the first connected Kinect found 
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CColorBasics::CreateFirstConnected()
{
    INuiSensor * pNuiSensor;
    HRESULT hr;

    int iSensorCount = 0;
    hr = NuiGetSensorCount(&iSensorCount);
    if (FAILED(hr))
    {
        return hr;
    }

    // Look at each Kinect sensor
    for (int i = 0; i < iSensorCount; ++i)
    {
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex(i, &pNuiSensor);
        if (FAILED(hr))
        {
            continue;
        }

        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pNuiSensor->NuiStatus();
        if (S_OK == hr)
        {
            m_pNuiSensor = pNuiSensor;
            break;
        }

        // This sensor wasn't OK, so release it since we're not using it
        pNuiSensor->Release();
    }

    if (NULL != m_pNuiSensor)
    {
        // Initialize the Kinect and specify that we'll be using color
        hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX);
        if (SUCCEEDED(hr))
        {
            // Create an event that will be signaled when color data is available
            m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			// Open a depth image stream to receive depth frames
            hr = m_pNuiSensor->NuiImageStreamOpen(
                NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
                NUI_IMAGE_RESOLUTION_640x480,
                0,
                2,
                m_hNextDepthFrameEvent,
                &m_pDepthStreamHandle);

			m_hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
            hr = m_pNuiSensor->NuiImageStreamOpen(
                NUI_IMAGE_TYPE_COLOR,
                NUI_IMAGE_RESOLUTION_640x480,
                0,
                2,
                m_hNextColorFrameEvent,
                &m_pColorStreamHandle);

#ifdef USE_SKELETON
			m_hNextSkeletonEvent = CreateEventW(NULL, TRUE, FALSE, NULL);
            hr = m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonEvent, 0);
#endif
        }
    }

    if (NULL == m_pNuiSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!");
        return E_FAIL;
    }

    return hr;
}

/// <summary>
/// Get the name of the file where screenshot will be stored.
/// </summary>
/// <param name="screenshotName">
/// [out] String buffer that will receive screenshot file name.
/// </param>
/// <param name="screenshotNameSize">
/// [in] Number of characters in screenshotName string buffer.
/// </param>
/// <returns>
/// S_OK on success, otherwise failure code.
/// </returns>
HRESULT GetScreenshotFileName(wchar_t *screenshotName, UINT screenshotNameSize)
{
    wchar_t *knownPath = NULL;
    HRESULT hr = SHGetKnownFolderPath(FOLDERID_Pictures, 0, NULL, &knownPath);

    if (SUCCEEDED(hr))
    {
        // Get the time
        wchar_t timeString[MAX_PATH];
        GetTimeFormatEx(NULL, 0, NULL, L"hh'-'mm'-'ss", timeString, _countof(timeString));

        // File name will be KinectSnapshot-HH-MM-SS.wav
        StringCchPrintfW(screenshotName, screenshotNameSize, L"%s\\KinectSnapshot-%s.bmp", knownPath, timeString);
    }

    CoTaskMemFree(knownPath);
    return hr;
}

/// <summary>
/// Handle new depth data
/// </summary>
void CColorBasics::ProcessDepth()
{
//    HRESULT hr;
//    NUI_IMAGE_FRAME imageFrame;
//
//    // Attempt to get the depth frame
//    hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
//    if (FAILED(hr))
//    {
//        return;
//    }
//
//    BOOL nearMode;
//    INuiFrameTexture* pTexture;
//
//    // Get the depth image pixel texture
//    hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(
//        m_pDepthStreamHandle, &imageFrame, &nearMode, &pTexture);
//    if (FAILED(hr))
//    {
//        goto ReleaseFrame;
//    }
//
//    NUI_LOCKED_RECT LockedRect;
//
//    // Lock the frame data so the Kinect knows not to modify it while we're reading it
//    pTexture->LockRect(0, &LockedRect, NULL, 0);
//
//    // Make sure we've received valid data
//    if (LockedRect.Pitch != 0)
//    {
//        // Get the min and max reliable depth for the current frame
//        int minDepth = (nearMode ? NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MINIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
//        int maxDepth = (nearMode ? NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MAXIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
//
//        BYTE * rgbrun = m_depthRGBX;
//        const NUI_DEPTH_IMAGE_PIXEL * pBufferRun = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);
//
//        // end pixel is start + width*height - 1
//        const NUI_DEPTH_IMAGE_PIXEL * pBufferEnd = pBufferRun + (m_depthWidth * m_depthHeight);
//
//        while ( pBufferRun < pBufferEnd )
//        {
//            // discard the portion of the depth that contains only the player index
//            USHORT depth = pBufferRun->depth;
//
//            // To convert to a byte, we're discarding the most-significant
//            // rather than least-significant bits.
//            // We're preserving detail, although the intensity will "wrap."
//            // Values outside the reliable depth range are mapped to 0 (black).
//
//            // Note: Using conditionals in this loop could degrade performance.
//            // Consider using a lookup table instead when writing production code.
//            BYTE intensity = static_cast<BYTE>(depth >= minDepth && depth <= maxDepth ? depth % 256 : 0);
//
//            // Write out
//            *(rgbrun++) = intensity; //b
//			*(rgbrun++) = intensity; //g
//			*(rgbrun++) = intensity; //r
//			rgbrun++;
//
//            // Increment our index into the Kinect's depth buffer
//            ++pBufferRun;
//        }
//
//    }
//
//    // We're done with the texture so unlock it
//    pTexture->UnlockRect(0);
//
//    pTexture->Release();
//
//ReleaseFrame:
//    // Release the frame
//    m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);
	NUI_IMAGE_FRAME imageFrame;

	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
    if ( FAILED(hr) ) { 
		return ; 
	}
   
    NUI_LOCKED_RECT LockedRect;
    hr = imageFrame.pFrameTexture->LockRect(0, &LockedRect, NULL, 0);
    if ( FAILED(hr) ) {
		return ; 
	}

	memcpy(m_depthD16, LockedRect.pBits, LockedRect.size);

    hr = imageFrame.pFrameTexture->UnlockRect(0);
    if ( FAILED(hr) ) {
		return ; 
	}

    hr = m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);

    return;
}

/// <summary>
/// Handle new color data
/// </summary>
/// <returns>indicates success or failure</returns>
void CColorBasics::ProcessColor()
{
    HRESULT hr;
    NUI_IMAGE_FRAME imageFrame;

    // Attempt to get the color frame
    hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pColorStreamHandle, 0, &imageFrame);
    if (FAILED(hr))
    {
        return;
    }

    INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
    NUI_LOCKED_RECT LockedRect;

    // Lock the frame data so the Kinect knows not to modify it while we're reading it
    pTexture->LockRect(0, &LockedRect, NULL, 0);



	memcpy(m_colorRGBX, LockedRect.pBits, LockedRect.size);
	//cv::Mat intermColorMat (m_colorHeight, m_colorWidth, CV_8UC4, m_colorRGBX);
	//cv::namedWindow("debug");
	//cv::imshow("debug", intermColorMat);
	//cv::waitKey();

	for (int y = 0; y < m_colorHeight; y++)
	{
		BYTE* src = m_colorRGBX + y * m_colorWidth * cBytesPerPixel;
		BYTE* dst = m_colorRGB + y * m_colorWidth * 3;

		BYTE* pi = m_alignedPlayerIdx + y * m_depthWidth;
		for (int x = 0; x < m_colorWidth; x++)
		{
			*dst = *src;
			*(dst+1) = *(src+1);
			*(dst+2) = *(src+2);

			// debug: show user detection in color image in gui
			*(src)	 = *(pi) > 0 ? 255 : *(src); // bluish
			*(src+3) = *(pi) > 0 ? 127 : *(src+3); // half transparent

			dst += 3;
			src += cBytesPerPixel;
			pi ++;
		}
	}

	//cv::Mat colorMat (m_colorHeight, m_colorWidth, CV_8UC3, m_colorRGB);
	//cv::imshow("debug", colorMat);
	//cv::waitKey();

    // Make sure we've received valid data
    if (LockedRect.Pitch != 0)
    {
        // Draw the data with Direct2D
        m_pDrawColor->Draw(static_cast<BYTE *>(m_colorRGBX), LockedRect.size);
    }


    // We're done with the texture so unlock it
    pTexture->UnlockRect(0);

    // Release the frame
    m_pNuiSensor->NuiImageStreamReleaseFrame(m_pColorStreamHandle, &imageFrame);
}


#ifdef USE_SKELETON

/// <summary>
/// Handle new skeleton data
/// </summary>
void CColorBasics::ProcessSkeleton()
{
    NUI_SKELETON_FRAME skeletonFrame = {0};

    HRESULT hr = m_pNuiSensor->NuiSkeletonGetNextFrame(0, &skeletonFrame);
    if ( FAILED(hr) )
    {
        return;
    }

    // smooth out the skeleton data
    m_pNuiSensor->NuiTransformSmooth(&skeletonFrame, NULL);
	
	// Keep a copy to save proper information later
	for (int i = 0 ; i < NUI_SKELETON_COUNT; ++i)
    {
        NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

        if (NUI_SKELETON_TRACKED == trackingState)
        {
			m_SkelsData[i][0] = NUI_SKELETON_TRACKED;
			for (int j = 1; j < NUI_SKELETON_POSITION_COUNT;)
			{
				// We're tracking the skeleton, draw it
				m_SkelsData[i][j] = skeletonFrame.SkeletonData[i].SkeletonPositions[j].w;
				m_SkelsData[i][j+1] = skeletonFrame.SkeletonData[i].SkeletonPositions[j].x;
				m_SkelsData[i][j+2] = skeletonFrame.SkeletonData[i].SkeletonPositions[j].y;
				m_SkelsData[i][j+3] = skeletonFrame.SkeletonData[i].SkeletonPositions[j].z;
				j += 4;
			}
        }
        else if (NUI_SKELETON_POSITION_ONLY == trackingState)
        {
            // we've only received the center point of the skeleton, draw that
            m_SkelsData[i][0] = NUI_SKELETON_TRACKED;
			m_SkelsData[i][1] = skeletonFrame.SkeletonData[i].Position.w;
			m_SkelsData[i][2] = skeletonFrame.SkeletonData[i].Position.x;
			m_SkelsData[i][3] = skeletonFrame.SkeletonData[i].Position.y;
			m_SkelsData[i][4] = skeletonFrame.SkeletonData[i].Position.z;
        }
    }

	// Draw purposes function
	//DrawSkeletonFrame(skeletonFrame);
}

/// <summary>
/// Draws all skeletons in a NUI_SKELETON_FRAME
/// </summary>
/// <param name="skel">skeleton frame</param>
void CColorBasics::DrawSkeletonFrame(const NUI_SKELETON_FRAME & skeletonFrame)
{
	HRESULT hr;

	// Endure Direct2D is ready to draw
    hr = EnsureDirect2DResources( );
    if ( FAILED(hr) )
    {
        return;
    }

	m_pRenderTarget->BeginDraw();
    m_pRenderTarget->Clear( );

    RECT rct;
    GetClientRect( GetDlgItem( m_hWnd, IDC_VIDEOVIEW ), &rct);
    int width = rct.right;
    int height = rct.bottom;

    for (int i = 0 ; i < NUI_SKELETON_COUNT; ++i)
    {
        NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

        if (NUI_SKELETON_TRACKED == trackingState)
        {
            // We're tracking the skeleton, draw it
            DrawSkeleton(skeletonFrame.SkeletonData[i], width, height);
        }
        else if (NUI_SKELETON_POSITION_ONLY == trackingState)
        {
            // we've only received the center point of the skeleton, draw that
            D2D1_ELLIPSE ellipse = D2D1::Ellipse(
                SkeletonToScreen(skeletonFrame.SkeletonData[i].Position, width, height),
                g_JointThickness,
                g_JointThickness
                );

            m_pRenderTarget->DrawEllipse(ellipse, m_pBrushJointTracked);
        }
    }

    hr = m_pRenderTarget->EndDraw();

    // Device lost, need to recreate the render target
    // We'll dispose it now and retry drawing
    if (D2DERR_RECREATE_TARGET == hr)
    {
        hr = S_OK;
        DiscardDirect2DResources();
    }
}

/// <summary>
/// Draws a skeleton
/// </summary>
/// <param name="skel">skeleton to draw</param>
/// <param name="windowWidth">width (in pixels) of output buffer</param>
/// <param name="windowHeight">height (in pixels) of output buffer</param>
void CColorBasics::DrawSkeleton(const NUI_SKELETON_DATA & skel, int windowWidth, int windowHeight)
{      
    int i;

    for (i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
    {
        m_Points[i] = SkeletonToScreen(skel.SkeletonPositions[i], windowWidth, windowHeight);
    }

    // Render Torso
    DrawBone(skel, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER);
    DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT);
    DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT);
    DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE);
    DrawBone(skel, NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER);
    DrawBone(skel, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT);
    DrawBone(skel, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT);

    // Left Arm
    DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT);
    DrawBone(skel, NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT);
    DrawBone(skel, NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT);

    // Right Arm
    DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT);
    DrawBone(skel, NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT);
    DrawBone(skel, NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT);

    // Left Leg
    DrawBone(skel, NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT);
    DrawBone(skel, NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT);
    DrawBone(skel, NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT);

    // Right Leg
    DrawBone(skel, NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT);
    DrawBone(skel, NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT);
    DrawBone(skel, NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT);

    // Draw the joints in a different color
    for (i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
    {
        D2D1_ELLIPSE ellipse = D2D1::Ellipse( m_Points[i], g_JointThickness, g_JointThickness );

        if ( skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_INFERRED )
        {
            m_pRenderTarget->DrawEllipse(ellipse, m_pBrushJointInferred);
        }
        else if ( skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_TRACKED )
        {
            m_pRenderTarget->DrawEllipse(ellipse, m_pBrushJointTracked);
        }
    }
}

/// <summary>
/// Draws a bone line between two joints
/// </summary>
/// <param name="skel">skeleton to draw bones from</param>
/// <param name="joint0">joint to start drawing from</param>
/// <param name="joint1">joint to end drawing at</param>
void CColorBasics::DrawBone(const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX joint0, NUI_SKELETON_POSITION_INDEX joint1)
{
    NUI_SKELETON_POSITION_TRACKING_STATE joint0State = skel.eSkeletonPositionTrackingState[joint0];
    NUI_SKELETON_POSITION_TRACKING_STATE joint1State = skel.eSkeletonPositionTrackingState[joint1];

    // If we can't find either of these joints, exit
    if (joint0State == NUI_SKELETON_POSITION_NOT_TRACKED || joint1State == NUI_SKELETON_POSITION_NOT_TRACKED)
    {
        return;
    }

    // Don't draw if both points are inferred
    if (joint0State == NUI_SKELETON_POSITION_INFERRED && joint1State == NUI_SKELETON_POSITION_INFERRED)
    {
        return;
    }

    // We assume all drawn bones are inferred unless BOTH joints are tracked
    if (joint0State == NUI_SKELETON_POSITION_TRACKED && joint1State == NUI_SKELETON_POSITION_TRACKED)
    {
        m_pRenderTarget->DrawLine(m_Points[joint0], m_Points[joint1], m_pBrushBoneTracked, g_TrackedBoneThickness);
    }
    else
    {
        m_pRenderTarget->DrawLine(m_Points[joint0], m_Points[joint1], m_pBrushBoneInferred, g_InferredBoneThickness);
    }
}

/// <summary>
/// Converts a skeleton point to screen space
/// </summary>
/// <param name="skeletonPoint">skeleton point to tranform</param>
/// <param name="width">width (in pixels) of output buffer</param>
/// <param name="height">height (in pixels) of output buffer</param>
/// <returns>point in screen-space</returns>
D2D1_POINT_2F CColorBasics::SkeletonToScreen(Vector4 skeletonPoint, int width, int height)
{
    LONG x, y;
    USHORT depth;

    // Calculate the skeleton's position on the screen
    // NuiTransformSkeletonToDepthImage returns coordinates in NUI_IMAGE_RESOLUTION_320x240 space
    NuiTransformSkeletonToDepthImage(skeletonPoint, &x, &y, &depth);

    float screenPointX = static_cast<float>(x * width) / 320;
    float screenPointY = static_cast<float>(y * height) / 240;

    return D2D1::Point2F(screenPointX, screenPointY);
}

#endif // USE_SKELETON


/// <summary>
/// Ensure necessary Direct2d resources are created
/// </summary>
/// <returns>S_OK if successful, otherwise an error code</returns>
HRESULT CColorBasics::EnsureDirect2DResources()
{
    HRESULT hr = S_OK;

    // If there isn't currently a render target, we need to create one
    if (NULL == m_pRenderTarget)
    {
        RECT rc;
        GetWindowRect( GetDlgItem( m_hWnd, IDC_VIDEOVIEW ), &rc );  

        int width = rc.right - rc.left;
        int height = rc.bottom - rc.top;
        D2D1_SIZE_U size = D2D1::SizeU( width, height );
        D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
        rtProps.pixelFormat = D2D1::PixelFormat( DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
        rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

        // Create a Hwnd render target, in order to render to the window set in initialize
        hr = m_pD2DFactory->CreateHwndRenderTarget(
            rtProps,
            D2D1::HwndRenderTargetProperties(GetDlgItem( m_hWnd, IDC_VIDEOVIEW), size),
            &m_pRenderTarget
            );
        if ( FAILED(hr) )
        {
            SetStatusMessage(L"Couldn't create Direct2D render target!");
            return hr;
        }

#ifdef USE_SKELETON
        //light green
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(0.27f, 0.75f, 0.27f), &m_pBrushJointTracked);

        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Yellow, 1.0f), &m_pBrushJointInferred);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 1.0f), &m_pBrushBoneTracked);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Gray, 1.0f), &m_pBrushBoneInferred);
#endif
    }

    return hr;
}

/// <summary>
/// Dispose Direct2d resources 
/// </summary>
void CColorBasics::DiscardDirect2DResources( )
{
    SafeRelease(m_pRenderTarget);
#ifdef USE_SKELETON
    SafeRelease(m_pBrushJointTracked);
    SafeRelease(m_pBrushJointInferred);
    SafeRelease(m_pBrushBoneTracked);
    SafeRelease(m_pBrushBoneInferred);
#endif
}


/// <summary>
/// Register depth to color, and returns separately the depth and the player idx information
/// </summary>
void CColorBasics::MapDepthToColor(const USHORT* depth, USHORT* alignedDepth, BYTE* alignedPlayerIdx, bool rmPlayerBits)
{
	HRESULT hr;
    
	//cv::Mat fakeDepthMat = cv::imread("00217.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH );

	std::fill_n(alignedDepth, m_depthHeight*m_depthWidth, 0);
	std::fill_n(alignedPlayerIdx, m_depthHeight*m_depthWidth, 0);

	// loop over each row and column of the color
    for (int y = 0; y < m_depthHeight; ++y)
    {
		const USHORT* sp = /*(const USHORT*)fakeDepthMat.data*/ depth + m_depthWidth * y;
        for (int x = 0; x < m_depthWidth; ++x, ++sp)
        {
			LONG colorInDepthX, colorInDepthY;
			USHORT d = *(sp);

			m_pNuiSensor->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
				cColorResolution,
				cDepthResolution,
				NULL,
				x, y,
				(d & ~NUI_IMAGE_PLAYER_INDEX_MASK),
				&colorInDepthX, &colorInDepthY);
			
            // make sure the depth pixel maps to a valid point in color space
            if ( colorInDepthX >= 0 && colorInDepthX < m_colorWidth && colorInDepthY >= 0 && colorInDepthY < m_colorHeight )
            {
				if (!rmPlayerBits)
					*(alignedDepth + colorInDepthY * (m_depthWidth) + colorInDepthX)	 = d;
				else
					*(alignedDepth + colorInDepthY * (m_depthWidth) + colorInDepthX)	 = (d & ~NUI_IMAGE_PLAYER_INDEX_MASK);

				*(alignedPlayerIdx + colorInDepthY * (m_depthWidth) + colorInDepthX) = (d & NUI_IMAGE_PLAYER_INDEX_MASK);
            }
        }
    }

    return;
}


/// <summary>
/// Register depth to color
/// </summary>
void CColorBasics::MapDepthToColor(const USHORT* depth, USHORT* alignedDepth)
{
	HRESULT hr;
    
	// loop over each row and column of the color
    for (int y = 0; y < m_depthHeight; ++y)
    {
		const USHORT* sp = depth + m_depthWidth * y;
        for (int x = 0; x < m_depthWidth; ++x, ++sp)
        {
			LONG colorInDepthX, colorInDepthY;
			USHORT d = *(sp);

			m_pNuiSensor->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
				cColorResolution,
				cDepthResolution,
				NULL,
				x, y,
				(d & ~NUI_IMAGE_PLAYER_INDEX_MASK),
				&colorInDepthX, &colorInDepthY);
			
            // make sure the depth pixel maps to a valid point in color space
            if ( colorInDepthX >= 0 && colorInDepthX < m_colorWidth && colorInDepthY >= 0 && colorInDepthY < m_colorHeight )
            {
				*(alignedDepth + colorInDepthY * (m_depthWidth) + colorInDepthX) = d;
            }
        }
    }

    return;
}


/// <summary>
/// Process color data received from Kinect
/// </summary>
/// <returns>S_OK for success, or failure code</returns>
void CColorBasics::MapColorToDepth()
{
  //  HRESULT hr;

  //  // Get of x, y coordinates for color in depth space
  //  // This will allow us to later compensate for the differences in location, angle, etc between the depth and color cameras
  //  m_pNuiSensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
  //      cColorResolution,
  //      cDepthResolution,
  //      m_depthWidth*m_depthHeight,
  //      m_depthD16,
  //      m_depthWidth*m_depthHeight*2,
  //      m_colorCoordinates
  //      );
  //  
  //  // loop over each row and column of the color
  //  for (int y = 0; y < m_colorHeight; ++y)
  //  {
		//BYTE* pDest = m_alignedColorRGBX + (m_colorWidth*cBytesPerPixel) * y;
  //      for (int x = 0; x < m_colorWidth; ++x)
  //      {
  //          // calculate index into depth array
  //          LONG depthIndex = x/m_colorToDepthDivisor + y/m_colorToDepthDivisor * m_depthWidth;

  //          // retrieve the depth to color mapping for the current depth pixel
  //          LONG colorInDepthX = m_colorCoordinates[depthIndex * 2];
  //          LONG colorInDepthY = m_colorCoordinates[depthIndex * 2 + 1];
		//	
  //          // make sure the depth pixel maps to a valid point in color space
  //          if ( colorInDepthX >= 0 && colorInDepthX < m_colorWidth && colorInDepthY >= 0 && colorInDepthY < m_colorHeight )
  //          {
		//		BYTE* pSrc = m_colorRGBX + colorInDepthY * (m_colorWidth*cBytesPerPixel) + colorInDepthX * cBytesPerPixel;
		//		*(pDest) = *(pSrc);
		//		*(pDest + 1) = *(pSrc + 1);
		//		*(pDest + 2) = *(pSrc + 2);
		//		*(pDest + 3) = *(pSrc + 3);
  //          }
  //          else
  //          {
		//		*pDest = 0;
		//		*(pDest+1) = 0;
		//		*(pDest+2) = 0;
		//		*(pDest+3) = 0;
  //          }

  //          pDest+=cBytesPerPixel;
  //      }
  //  }

  //  return;
}

/// <summary>
/// Create a client
/// </summary>
/// <param name="host">Host to communicate with</param>
/// <param name="host">Port to communicate to in the host part</param>
void CColorBasics::CreateClient(const char* host, const char* port)
{
	try
	{
		boost::asio::io_service* io_service = new boost::asio::io_service();

		tcp::resolver resolver(*io_service);
		tcp::resolver::query query(tcp::v4(), host, port);
		tcp::resolver::iterator iterator = resolver.resolve(query);

		//tcp::socket s(io_service);
		m_pSocket = new tcp::socket(*io_service);
	
		boost::asio::connect(*m_pSocket, iterator);
	}
	catch (std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
	}
}

/// <summary>
/// Listen a socket in a thread
/// </summary>
/// <param name="sock">Socket pointer</param>
void CColorBasics::Session(boost::shared_ptr<boost::asio::ip::tcp::socket> sock)
{
  try
  {
    for (;;)
    {
      char data[max_length];

      boost::system::error_code error;
      size_t length = sock->read_some(boost::asio::buffer(data), error);
      if (error == boost::asio::error::eof)
        return; // Connection closed cleanly by peer.
      else if (error)
        throw boost::system::system_error(error); // Some other error.

      boost::asio::write(*sock, boost::asio::buffer(data, length));

	  if (strcmp(data, "tk") == 0) // Token
	  {
		  m_pNuiSensor->NuiSetForceInfraredEmitterOff(true);
	  }
    }
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception in thread: " << e.what() << "\n";
  }
}

/// <summary>
/// Put the created server to listen
/// </summary>
void CColorBasics::Server(boost::asio::io_service& io_service, unsigned short port)
{
	tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), port));

	m_pSocket = new tcp::socket(io_service);
	a.accept(*m_pSocket);
	//boost::thread t( boost::bind(&CColorBasics::Session, this, sock) );

	//try
	//{
	//	char data[max_length];

	//	boost::system::error_code error;
	//	size_t length = sock->read_some(boost::asio::buffer(data), error);
	//	if (error == boost::asio::error::eof)
	//		return; // Connection closed cleanly by peer.
	//	else if (error)
	//		throw boost::system::system_error(error); // Some other error.
	//}
	//catch (std::exception& e)
	//{
	//	std::cerr << "Exception in thread: " << e.what() << "\n";
	//}
}

/// <summary>
/// Create a server
/// </summary>
/// <param name="port">Port to listen</param>
void CColorBasics::CreateServer(const char* port)
{
	boost::asio::io_service* io_service = new boost::asio::io_service();
	//boost::thread t ( boost::bind(&CColorBasics::Server, this, io_service, std::atoi(port)) );
	Server(*io_service, std::atoi(port));
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
void CColorBasics::SetStatusMessage(WCHAR * szMessage)
{
    SendDlgItemMessageW(m_hWnd, IDC_STATUS, WM_SETTEXT, 0, (LPARAM)szMessage);
}

/// <summary>
/// Save passed in image data to disk as a bitmap
/// </summary>
/// <param name="pBitmapBits">image data to save</param>
/// <param name="lWidth">width (in pixels) of input image data</param>
/// <param name="lHeight">height (in pixels) of input image data</param>
/// <param name="wBitsPerPixel">bits per pixel of image data</param>
/// <param name="lpszFilePath">full file path to output bitmap to</param>
/// <returns>indicates success or failure</returns>
HRESULT CColorBasics::SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath)
{
    DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

    BITMAPINFOHEADER bmpInfoHeader = {0};

    bmpInfoHeader.biSize        = sizeof(BITMAPINFOHEADER);  // Size of the header
    bmpInfoHeader.biBitCount    = wBitsPerPixel;             // Bit count
    bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression
    bmpInfoHeader.biWidth       = lWidth;                    // Width in pixels
    bmpInfoHeader.biHeight      = -lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
    bmpInfoHeader.biPlanes      = 1;                         // Default
    bmpInfoHeader.biSizeImage   = dwByteCount;               // Image size in bytes

    BITMAPFILEHEADER bfh = {0};

    bfh.bfType    = 0x4D42;                                           // 'M''B', indicates bitmap
    bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
    bfh.bfSize    = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers

    // Create the file on disk to write to
    HANDLE hFile = CreateFileW(lpszFilePath, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

    // Return if error opening file
    if (NULL == hFile) 
    {
        return E_ACCESSDENIED;
    }

    DWORD dwBytesWritten = 0;
    
    // Write the bitmap file header
    if ( !WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL) )
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the bitmap info header
    if ( !WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL) )
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the RGB Data
    if ( !WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL) )
    {
        CloseHandle(hFile);
        return E_FAIL;
    }    

    // Close the file
    CloseHandle(hFile);
    return S_OK;
}

void CColorBasics::peredelafavera()
{
	using namespace boost::gregorian; 
    using namespace boost::local_time;
    using namespace boost::posix_time;
    
	tz_database tz_db;
    try {
      tz_db.load_from_file("clock/date_time_zonespec.csv");
    }catch(data_not_accessible dna) {
      std::cerr << "Error with time zone data file: " << dna.what() << std::endl;
      exit(EXIT_FAILURE);
    }catch(bad_field_count bfc) {
      std::cerr << "Error with time zone data file: " << bfc.what() << std::endl;
      exit(EXIT_FAILURE);
    }

    time_zone_ptr region_tz = tz_db.time_zone_from_region("Europe/Madrid");
 //   date in_date(;
 //   time_duration td(12,14,32);
 //   // construct with local time value
 //   // create not-a-date-time if invalid (eg: in dst transition)
    local_date_time region_time = local_microsec_clock::local_time(region_tz); //,                              local_date_time::NOT_DATE_TIME_ON_ERROR);

    std::cout << region_time << std::endl;

    ptime time_t_epoch(date(1970,1,1)); 
    std::cout << time_t_epoch << std::endl;

    // first convert nyc_time to utc via the utc_time() 
    // call and subtract the ptime.
    time_duration localTime = region_time.utc_time() - time_t_epoch;
	
	//double correction = getTimeDiffRespectToServer();
	cv::FileStorage initTime;
	initTime.open("data/initTime.yml", cv::FileStorage::WRITE);
	std::cout << localTime.total_milliseconds() << std::endl;
	initTime << "initTime" << ((double) localTime.total_milliseconds() /*- correction*/);
	initTime.release();
}