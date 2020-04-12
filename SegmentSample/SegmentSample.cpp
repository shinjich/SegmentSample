#ifndef STRICT
#define STRICT	// 厳密なコードを型を要求する
#endif
#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#include <k4a/k4a.h>
#include <k4abt.h>

#pragma comment( lib, "k4a.lib" )
#pragma comment( lib, "k4abt.lib" )

#define ENABLE_CSV_OUTPUT		1			// 1=CSV 出力を有効にする

// イメージの解像度
#define RESOLUTION_WIDTH		(640)
#define RESOLUTION_HEIGHT		(576)

#define MAX_BODIES				8

// アプリケーションのタイトル名
static const TCHAR szClassName[] = TEXT("セグメンテーションサンプル");
HWND g_hWnd = NULL;							// アプリケーションのウィンドウ
HBITMAP g_hBMP = NULL, g_hBMPold = NULL;	// 表示するビットマップのハンドル
HDC g_hDCBMP = NULL;						// 表示するビットマップのコンテキスト
BITMAPINFO g_biBMP = { 0, };				// ビットマップの情報 (解像度やフォーマット)
LPDWORD g_pdwPixel = NULL;					// ビットマップの中身の先頭 (ピクセル情報)
LPWORD g_pDepthMap = NULL;					// 深度マップバッファのポインタ
LPBYTE g_pSegmentMap = NULL;				// セグメントマップバッファのポインタ

k4a_device_t g_hAzureKinect = nullptr;		// Azure Kinect のデバイスハンドル
k4abt_tracker_t g_hTracker = nullptr;		// ボディトラッカーのハンドル
k4a_calibration_t g_Calibration;			// Azure Kinect のキャリブレーションデータ

// Kinect を初期化する
k4a_result_t CreateKinect()
{
	k4a_result_t hr;

	// Azure Kinect を初期化する
	hr = k4a_device_open( K4A_DEVICE_DEFAULT, &g_hAzureKinect );
	if ( hr == K4A_RESULT_SUCCEEDED )
	{
		// カメラを開始する
		k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
		config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
		config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		config.camera_fps = K4A_FRAMES_PER_SECOND_30;
		config.synchronized_images_only = false;
		config.depth_delay_off_color_usec = 0;
		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
		config.subordinate_delay_off_master_usec = 0;
		config.disable_streaming_indicator = false;

		// Azure Kinect を開始する
		hr = k4a_device_start_cameras( g_hAzureKinect, &config );
		if ( hr == K4A_RESULT_SUCCEEDED )
		{
			// キャリブレーションデータを取得する
			hr = k4a_device_get_calibration( g_hAzureKinect, config.depth_mode, config.color_resolution, &g_Calibration );
			if ( hr == K4A_RESULT_SUCCEEDED )
			{
				// 骨格追跡を開始する
				hr = k4abt_tracker_create( &g_Calibration, K4ABT_TRACKER_CONFIG_DEFAULT, &g_hTracker );
				if ( hr == K4A_RESULT_SUCCEEDED )
				{
					return hr;
				}
				else
				{
					MessageBox( NULL, TEXT("骨格追跡を開始できませんでした"), TEXT("エラー"), MB_OK );
				}
			}
			else
			{
				MessageBox( NULL, TEXT("キャリブレーション情報を取得できませんでした"), TEXT("エラー"), MB_OK );
			}
			// Azure Kinect を停止する
			k4a_device_stop_cameras( g_hAzureKinect );
		}
		else
		{
			MessageBox( NULL, TEXT("Azure Kinect を開始できませんでした"), TEXT("エラー"), MB_OK );
		}
		// Azure Kinect の使用をやめる
		k4a_device_close( g_hAzureKinect );
	}
	else
	{
		MessageBox( NULL, TEXT("Azure Kinect の初期化に失敗 - カメラの状態を確認してください"), TEXT("エラー"), MB_OK );
	}
	return hr;
}

// Kinect を終了する
void DestroyKinect()
{
	// 骨格追跡を無効にする
	if ( g_hTracker )
	{
		k4abt_tracker_destroy( g_hTracker );
		g_hTracker = nullptr;
	}

	if ( g_hAzureKinect )
	{
		// Azure Kinect を停止する
		k4a_device_stop_cameras( g_hAzureKinect );

		// Azure Kinect の使用をやめる
		k4a_device_close( g_hAzureKinect );
		g_hAzureKinect = nullptr;
	}
}

// KINECT のメインループ処理
uint32_t KinectProc()
{
	k4a_wait_result_t hr;
	uint32_t uImageSize = 0;
	uint32_t uBodies = 0;

	k4a_capture_t hCapture = nullptr;
	// カメラでキャプチャーする
	hr = k4a_device_get_capture( g_hAzureKinect, &hCapture, K4A_WAIT_INFINITE );
	if ( hr == K4A_WAIT_RESULT_SUCCEEDED )
	{
		k4a_image_t hImage;

		// 深度イメージを取得する
		hImage = k4a_capture_get_depth_image( hCapture );

		// 骨格追跡をキューする
		hr = k4abt_tracker_enqueue_capture( g_hTracker, hCapture, K4A_WAIT_INFINITE );
		// カメラキャプチャーを解放する
		k4a_capture_release( hCapture );

		if ( hImage )
		{
			// イメージピクセルの先頭ポインタを取得する
			uint8_t* p = k4a_image_get_buffer( hImage );
			if ( p )
			{
				// イメージサイズを取得する
				uImageSize = (uint32_t) k4a_image_get_size( hImage );
				CopyMemory( g_pDepthMap, p, uImageSize );
			}
			// イメージを解放する
			k4a_image_release( hImage );
		}

		if ( hr == K4A_WAIT_RESULT_SUCCEEDED )
		{
			// 骨格追跡の結果を取得する
			k4abt_frame_t hBodyFrame = nullptr;
			hr = k4abt_tracker_pop_result( g_hTracker, &hBodyFrame, K4A_WAIT_INFINITE );
			if ( hr == K4A_WAIT_RESULT_SUCCEEDED )
			{
				// ボディインデックスイメージを取得する
				hImage = k4abt_frame_get_body_index_map( hBodyFrame );
				if ( hImage )
				{
					// イメージピクセルの先頭ポインタを取得する
					uint8_t* p = k4a_image_get_buffer( hImage );
					if ( p )
					{
						// イメージサイズを取得する
						uImageSize = (uint32_t) k4a_image_get_size( hImage );
						CopyMemory( g_pSegmentMap, p, uImageSize );
					}
					// イメージを解放する
					k4a_image_release( hImage );
				}
				// 骨格追跡フレームを解放する
				k4abt_frame_release( hBodyFrame );
			}
		}
	}
	return uImageSize;
}

#if ENABLE_CSV_OUTPUT
// CSV ファイルにデータを出力
void WriteCSV()
{
	// CSV を作成する
	HANDLE hFile = CreateFileA( "segment.csv", GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL );
	if ( hFile != INVALID_HANDLE_VALUE )
	{
		for( int y = 0; y < RESOLUTION_HEIGHT; y++ )
		{
			// セグメント情報を CSV に出力
			char szTmp[8];
			char szText[RESOLUTION_WIDTH * sizeof(szTmp)] = "";
			for( int x = 0; x < RESOLUTION_WIDTH; x++ )
			{
				const BYTE c = g_pSegmentMap[y * RESOLUTION_WIDTH + x];
				sprintf_s( szTmp, 8, "%d,", c );
				strcat_s( szText, RESOLUTION_WIDTH * sizeof(szTmp), szTmp );
			}

			// 改行してファイル出力
			strcat_s( szText, RESOLUTION_WIDTH * sizeof(szTmp), "\r\n" );
			const DWORD dwLen = (DWORD) strlen( szText );
			DWORD dwWritten;
			WriteFile( hFile, szText, dwLen, &dwWritten, NULL );
		}
		CloseHandle( hFile );
	}
}
#endif

LRESULT CALLBACK WndProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	switch( uMsg )
	{
	case WM_PAINT:
		{
			// 画面表示処理
			PAINTSTRUCT ps;
			HDC hDC = BeginPaint( hWnd, &ps );

			// 画面サイズを取得する
			RECT rect;
			GetClientRect( hWnd, &rect );

			// 8-bit セグメントマップから 32-bit カラービットマップに変換する
			for( uint32_t u = 0; u < RESOLUTION_WIDTH * RESOLUTION_HEIGHT; u++ )
			{
				if ( g_pDepthMap[u] )
				{
					const BYTE c = g_pSegmentMap[u] + 1;
					if ( c )
						g_pdwPixel[u] = 0xFF000000 | ((c & 4)?0xFF0000:0) | ((c & 2)?0xFF00:0) | ((c & 1)?0xFF:0);
					else
						g_pdwPixel[u] = 0xFF808080;
				}
				else
				{
					g_pdwPixel[u] = 0xFF000000;
				}
			}

			// カラー化した深度の表示
			StretchBlt( hDC, 0, 0, rect.right, rect.bottom, g_hDCBMP, 0, 0, RESOLUTION_WIDTH, RESOLUTION_HEIGHT, SRCCOPY );
			EndPaint( hWnd, &ps );
		}
		return 0;
#if ENABLE_CSV_OUTPUT
	case WM_KEYDOWN:
		// スペースキーが押されたら CSV 出力する
		if ( wParam == VK_SPACE )
			WriteCSV();
		break;
#endif
	case WM_CLOSE:
		DestroyWindow( hWnd );
	case WM_DESTROY:
		PostQuitMessage( 0 );
		break;
	default:
		return DefWindowProc( hWnd, uMsg, wParam, lParam );
	}
	return 0;
}

// アプリケーションの初期化 (ウィンドウや描画用のペンを作成)
HRESULT InitApp( HINSTANCE hInst, int nCmdShow )
{
	WNDCLASSEX wc = { 0, };
	wc.cbSize = sizeof(WNDCLASSEX);
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.lpfnWndProc = WndProc;
	wc.hInstance = hInst;
	wc.hIcon = LoadIcon( NULL, IDI_APPLICATION );
	wc.hCursor = LoadCursor( NULL, IDC_ARROW );
	wc.hbrBackground = (HBRUSH) GetStockObject( NULL_BRUSH );
	wc.lpszClassName = szClassName;
	wc.hIconSm = LoadIcon( NULL, IDI_APPLICATION );
	if ( ! RegisterClassEx( &wc ) )
	{
		MessageBox( NULL, TEXT("アプリケーションクラスの初期化に失敗"), TEXT("エラー"), MB_OK );
		return E_FAIL;
	}

	// アプリケーションウィンドウを作成
	g_hWnd = CreateWindow( szClassName, szClassName, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, NULL, NULL, hInst, NULL );
	if ( ! g_hWnd )
	{
		MessageBox( NULL, TEXT("ウィンドウの初期化に失敗"), TEXT("エラー"), MB_OK );
		return E_FAIL;
	}

	// 画面表示用のビットマップを作成する
	ZeroMemory( &g_biBMP, sizeof(g_biBMP) );
	g_biBMP.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	g_biBMP.bmiHeader.biBitCount = 32;
	g_biBMP.bmiHeader.biPlanes = 1;
	g_biBMP.bmiHeader.biWidth = RESOLUTION_WIDTH;
	g_biBMP.bmiHeader.biHeight = -(int) RESOLUTION_HEIGHT;
	g_hBMP = CreateDIBSection( NULL, &g_biBMP, DIB_RGB_COLORS, (LPVOID*) (&g_pdwPixel), NULL, 0 );
	HDC hDC = GetDC( g_hWnd );
	g_hDCBMP = CreateCompatibleDC( hDC );
	ReleaseDC( g_hWnd, hDC );
	g_hBMPold = (HBITMAP) SelectObject( g_hDCBMP, g_hBMP );

	// 深度マップ用バッファを作成
	g_pDepthMap = new UINT16[RESOLUTION_WIDTH * RESOLUTION_HEIGHT];
	memset( g_pDepthMap, 0, RESOLUTION_WIDTH * RESOLUTION_HEIGHT * sizeof(UINT16) );

	// セグメントマップ用バッファを作成
	g_pSegmentMap = new BYTE[RESOLUTION_WIDTH * RESOLUTION_HEIGHT];
	memset( g_pSegmentMap, K4ABT_BODY_INDEX_MAP_BACKGROUND, RESOLUTION_WIDTH * RESOLUTION_HEIGHT * sizeof(BYTE) );

	// ウィンドウを表示する
	ShowWindow( g_hWnd, nCmdShow );
	UpdateWindow( g_hWnd );

	return S_OK;
}

// アプリケーションの後始末
HRESULT UninitApp()
{
	// セグメントマップを解放する
	if ( g_pSegmentMap )
	{
		delete [] g_pSegmentMap;
		g_pSegmentMap = NULL;
	}

	// 深度マップを解放する
	if ( g_pDepthMap )
	{
		delete [] g_pDepthMap;
		g_pDepthMap = NULL;
	}

	// 画面表示用のビットマップを解放する
	if ( g_hDCBMP || g_hBMP )
	{
		SelectObject( g_hDCBMP, g_hBMPold );
		DeleteObject( g_hBMP );
		DeleteDC( g_hDCBMP );
		g_hBMP = NULL;
		g_hDCBMP = NULL;
	}
	return S_OK;
}

// エントリーポイント
int WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR, int nCmdShow )
{
	// アプリケーションを初期化する
	if ( FAILED( InitApp( hInst, nCmdShow ) ) )
		return 1;

	// KINECT を初期化する
	if ( FAILED( CreateKinect() ) )
		return 1;

	// アプリケーションループ
	MSG msg;
	while( GetMessage( &msg, NULL, 0, 0 ) )
	{
		// ウィンドウメッセージを処理
		TranslateMessage( &msg );
		DispatchMessage( &msg );

		// Kinect 情報に更新があれば描画
		if ( KinectProc() )
		{
			// 描画
			InvalidateRect( g_hWnd, NULL, TRUE );
		}
	}

	// KINECT を終了する
	DestroyKinect();

	// アプリケーションを終了する
	UninitApp();

	return (int) msg.wParam;
}
