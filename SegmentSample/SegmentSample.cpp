#ifndef STRICT
#define STRICT	// �����ȃR�[�h���^��v������
#endif
#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#include <k4a/k4a.h>
#include <k4abt.h>

#pragma comment( lib, "k4a.lib" )
#pragma comment( lib, "k4abt.lib" )

#define ENABLE_CSV_OUTPUT		1			// 1=CSV �o�͂�L���ɂ���

// �C���[�W�̉𑜓x
#define RESOLUTION_WIDTH		(640)
#define RESOLUTION_HEIGHT		(576)

#define MAX_BODIES				8

// �A�v���P�[�V�����̃^�C�g����
static const TCHAR szClassName[] = TEXT("�Z�O�����e�[�V�����T���v��");
HWND g_hWnd = NULL;							// �A�v���P�[�V�����̃E�B���h�E
HBITMAP g_hBMP = NULL, g_hBMPold = NULL;	// �\������r�b�g�}�b�v�̃n���h��
HDC g_hDCBMP = NULL;						// �\������r�b�g�}�b�v�̃R���e�L�X�g
BITMAPINFO g_biBMP = { 0, };				// �r�b�g�}�b�v�̏�� (�𑜓x��t�H�[�}�b�g)
LPDWORD g_pdwPixel = NULL;					// �r�b�g�}�b�v�̒��g�̐擪 (�s�N�Z�����)
LPWORD g_pDepthMap = NULL;					// �[�x�}�b�v�o�b�t�@�̃|�C���^
LPBYTE g_pSegmentMap = NULL;				// �Z�O�����g�}�b�v�o�b�t�@�̃|�C���^

k4a_device_t g_hAzureKinect = nullptr;		// Azure Kinect �̃f�o�C�X�n���h��
k4abt_tracker_t g_hTracker = nullptr;		// �{�f�B�g���b�J�[�̃n���h��
k4a_calibration_t g_Calibration;			// Azure Kinect �̃L�����u���[�V�����f�[�^

// Kinect ������������
k4a_result_t CreateKinect()
{
	k4a_result_t hr;

	// Azure Kinect ������������
	hr = k4a_device_open( K4A_DEVICE_DEFAULT, &g_hAzureKinect );
	if ( hr == K4A_RESULT_SUCCEEDED )
	{
		// �J�������J�n����
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

		// Azure Kinect ���J�n����
		hr = k4a_device_start_cameras( g_hAzureKinect, &config );
		if ( hr == K4A_RESULT_SUCCEEDED )
		{
			// �L�����u���[�V�����f�[�^���擾����
			hr = k4a_device_get_calibration( g_hAzureKinect, config.depth_mode, config.color_resolution, &g_Calibration );
			if ( hr == K4A_RESULT_SUCCEEDED )
			{
				// ���i�ǐՂ��J�n����
				hr = k4abt_tracker_create( &g_Calibration, K4ABT_TRACKER_CONFIG_DEFAULT, &g_hTracker );
				if ( hr == K4A_RESULT_SUCCEEDED )
				{
					return hr;
				}
				else
				{
					MessageBox( NULL, TEXT("���i�ǐՂ��J�n�ł��܂���ł���"), TEXT("�G���["), MB_OK );
				}
			}
			else
			{
				MessageBox( NULL, TEXT("�L�����u���[�V���������擾�ł��܂���ł���"), TEXT("�G���["), MB_OK );
			}
			// Azure Kinect ���~����
			k4a_device_stop_cameras( g_hAzureKinect );
		}
		else
		{
			MessageBox( NULL, TEXT("Azure Kinect ���J�n�ł��܂���ł���"), TEXT("�G���["), MB_OK );
		}
		// Azure Kinect �̎g�p����߂�
		k4a_device_close( g_hAzureKinect );
	}
	else
	{
		MessageBox( NULL, TEXT("Azure Kinect �̏������Ɏ��s - �J�����̏�Ԃ��m�F���Ă�������"), TEXT("�G���["), MB_OK );
	}
	return hr;
}

// Kinect ���I������
void DestroyKinect()
{
	// ���i�ǐՂ𖳌��ɂ���
	if ( g_hTracker )
	{
		k4abt_tracker_destroy( g_hTracker );
		g_hTracker = nullptr;
	}

	if ( g_hAzureKinect )
	{
		// Azure Kinect ���~����
		k4a_device_stop_cameras( g_hAzureKinect );

		// Azure Kinect �̎g�p����߂�
		k4a_device_close( g_hAzureKinect );
		g_hAzureKinect = nullptr;
	}
}

// KINECT �̃��C�����[�v����
uint32_t KinectProc()
{
	k4a_wait_result_t hr;
	uint32_t uImageSize = 0;
	uint32_t uBodies = 0;

	k4a_capture_t hCapture = nullptr;
	// �J�����ŃL���v�`���[����
	hr = k4a_device_get_capture( g_hAzureKinect, &hCapture, K4A_WAIT_INFINITE );
	if ( hr == K4A_WAIT_RESULT_SUCCEEDED )
	{
		k4a_image_t hImage;

		// �[�x�C���[�W���擾����
		hImage = k4a_capture_get_depth_image( hCapture );

		// ���i�ǐՂ��L���[����
		hr = k4abt_tracker_enqueue_capture( g_hTracker, hCapture, K4A_WAIT_INFINITE );
		// �J�����L���v�`���[���������
		k4a_capture_release( hCapture );

		if ( hImage )
		{
			// �C���[�W�s�N�Z���̐擪�|�C���^���擾����
			uint8_t* p = k4a_image_get_buffer( hImage );
			if ( p )
			{
				// �C���[�W�T�C�Y���擾����
				uImageSize = (uint32_t) k4a_image_get_size( hImage );
				CopyMemory( g_pDepthMap, p, uImageSize );
			}
			// �C���[�W���������
			k4a_image_release( hImage );
		}

		if ( hr == K4A_WAIT_RESULT_SUCCEEDED )
		{
			// ���i�ǐՂ̌��ʂ��擾����
			k4abt_frame_t hBodyFrame = nullptr;
			hr = k4abt_tracker_pop_result( g_hTracker, &hBodyFrame, K4A_WAIT_INFINITE );
			if ( hr == K4A_WAIT_RESULT_SUCCEEDED )
			{
				// �{�f�B�C���f�b�N�X�C���[�W���擾����
				hImage = k4abt_frame_get_body_index_map( hBodyFrame );
				if ( hImage )
				{
					// �C���[�W�s�N�Z���̐擪�|�C���^���擾����
					uint8_t* p = k4a_image_get_buffer( hImage );
					if ( p )
					{
						// �C���[�W�T�C�Y���擾����
						uImageSize = (uint32_t) k4a_image_get_size( hImage );
						CopyMemory( g_pSegmentMap, p, uImageSize );
					}
					// �C���[�W���������
					k4a_image_release( hImage );
				}
				// ���i�ǐՃt���[�����������
				k4abt_frame_release( hBodyFrame );
			}
		}
	}
	return uImageSize;
}

#if ENABLE_CSV_OUTPUT
// CSV �t�@�C���Ƀf�[�^���o��
void WriteCSV()
{
	// CSV ���쐬����
	HANDLE hFile = CreateFileA( "segment.csv", GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL );
	if ( hFile != INVALID_HANDLE_VALUE )
	{
		for( int y = 0; y < RESOLUTION_HEIGHT; y++ )
		{
			// �Z�O�����g���� CSV �ɏo��
			char szTmp[8];
			char szText[RESOLUTION_WIDTH * sizeof(szTmp)] = "";
			for( int x = 0; x < RESOLUTION_WIDTH; x++ )
			{
				const BYTE c = g_pSegmentMap[y * RESOLUTION_WIDTH + x];
				sprintf_s( szTmp, 8, "%d,", c );
				strcat_s( szText, RESOLUTION_WIDTH * sizeof(szTmp), szTmp );
			}

			// ���s���ăt�@�C���o��
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
			// ��ʕ\������
			PAINTSTRUCT ps;
			HDC hDC = BeginPaint( hWnd, &ps );

			// ��ʃT�C�Y���擾����
			RECT rect;
			GetClientRect( hWnd, &rect );

			// 8-bit �Z�O�����g�}�b�v���� 32-bit �J���[�r�b�g�}�b�v�ɕϊ�����
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

			// �J���[�������[�x�̕\��
			StretchBlt( hDC, 0, 0, rect.right, rect.bottom, g_hDCBMP, 0, 0, RESOLUTION_WIDTH, RESOLUTION_HEIGHT, SRCCOPY );
			EndPaint( hWnd, &ps );
		}
		return 0;
#if ENABLE_CSV_OUTPUT
	case WM_KEYDOWN:
		// �X�y�[�X�L�[�������ꂽ�� CSV �o�͂���
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

// �A�v���P�[�V�����̏����� (�E�B���h�E��`��p�̃y�����쐬)
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
		MessageBox( NULL, TEXT("�A�v���P�[�V�����N���X�̏������Ɏ��s"), TEXT("�G���["), MB_OK );
		return E_FAIL;
	}

	// �A�v���P�[�V�����E�B���h�E���쐬
	g_hWnd = CreateWindow( szClassName, szClassName, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, NULL, NULL, hInst, NULL );
	if ( ! g_hWnd )
	{
		MessageBox( NULL, TEXT("�E�B���h�E�̏������Ɏ��s"), TEXT("�G���["), MB_OK );
		return E_FAIL;
	}

	// ��ʕ\���p�̃r�b�g�}�b�v���쐬����
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

	// �[�x�}�b�v�p�o�b�t�@���쐬
	g_pDepthMap = new UINT16[RESOLUTION_WIDTH * RESOLUTION_HEIGHT];
	memset( g_pDepthMap, 0, RESOLUTION_WIDTH * RESOLUTION_HEIGHT * sizeof(UINT16) );

	// �Z�O�����g�}�b�v�p�o�b�t�@���쐬
	g_pSegmentMap = new BYTE[RESOLUTION_WIDTH * RESOLUTION_HEIGHT];
	memset( g_pSegmentMap, K4ABT_BODY_INDEX_MAP_BACKGROUND, RESOLUTION_WIDTH * RESOLUTION_HEIGHT * sizeof(BYTE) );

	// �E�B���h�E��\������
	ShowWindow( g_hWnd, nCmdShow );
	UpdateWindow( g_hWnd );

	return S_OK;
}

// �A�v���P�[�V�����̌�n��
HRESULT UninitApp()
{
	// �Z�O�����g�}�b�v���������
	if ( g_pSegmentMap )
	{
		delete [] g_pSegmentMap;
		g_pSegmentMap = NULL;
	}

	// �[�x�}�b�v���������
	if ( g_pDepthMap )
	{
		delete [] g_pDepthMap;
		g_pDepthMap = NULL;
	}

	// ��ʕ\���p�̃r�b�g�}�b�v���������
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

// �G���g���[�|�C���g
int WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR, int nCmdShow )
{
	// �A�v���P�[�V����������������
	if ( FAILED( InitApp( hInst, nCmdShow ) ) )
		return 1;

	// KINECT ������������
	if ( FAILED( CreateKinect() ) )
		return 1;

	// �A�v���P�[�V�������[�v
	MSG msg;
	while( GetMessage( &msg, NULL, 0, 0 ) )
	{
		// �E�B���h�E���b�Z�[�W������
		TranslateMessage( &msg );
		DispatchMessage( &msg );

		// Kinect ���ɍX�V������Ε`��
		if ( KinectProc() )
		{
			// �`��
			InvalidateRect( g_hWnd, NULL, TRUE );
		}
	}

	// KINECT ���I������
	DestroyKinect();

	// �A�v���P�[�V�������I������
	UninitApp();

	return (int) msg.wParam;
}
